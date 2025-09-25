"""
This script trains a reinforcement learning agent to control a balancing robot
in a MuJoCo simulation environment. The training is performed using the
Proximal Policy Optimization (PPO) algorithm.

The script leverages the JAX ecosystem for high-performance computation:
- JAX: For automatic differentiation and JIT compilation.
- Flax: For defining the neural network (Actor-Critic model).
- Optax: For implementing the Adam optimizer.
- MuJoCo/MJX: For running the physics simulation in a JAX-native way,
  allowing the entire training loop to be JIT-compiled and run on accelerators
  like GPUs or TPUs.
- Orbax: For saving the trained model parameters (checkpoints).

The agent learns to apply forces to the cart to keep the pole upright and
centered, even when subjected to random external forces. The training loop
is parallelized across many environments using `jax.vmap` to stabilize and
speed up learning.
"""

import jax
import jax.numpy as jnp
import mujoco
from mujoco import mjx
import flax.linen as nn
import optax
from orbax import checkpoint as ocp
import os
import time
import functools
from jax import tree as jax_tree

def env_reset(mjx_model, rng):
    """
    Resets the environment to an initial state.

    The cart and pole start at the origin (0,0) with zero velocity.
    A small random nudge is applied to the pole's initial angle to
    introduce variability and prevent the policy from learning a trivial
    "do nothing" strategy.

    Args:
        mjx_model: The compiled MJX model of the environment.
        rng: A JAX random number generator key.

    Returns:
        A tuple containing:
        - data (mjx.Data): The initial state of the MJX simulation.
        - obs (jnp.ndarray): The initial observation vector.
    """
    # Split the random key for subsequent use.
    rng, key = jax.random.split(rng)

    # Initialize joint positions (qpos) to zero.
    qpos = jnp.zeros(mjx_model.nq)

    # Generate a small random nudge for the pole's starting angle.
    nudge = jax.random.uniform(key, (), minval=-0.05, maxval=0.05)

    # Apply the nudge to the second joint (the pole's hinge).
    qpos = qpos.at[1].set(nudge)

    # Initialize joint velocities (qvel) to zero.
    qvel = jnp.zeros(mjx_model.nv)

    # Create the initial MJX data structure.
    data = mjx.make_data(mjx_model)

    # Update the data structure with the new initial qpos and qvel.
    data = data.replace(qpos=qpos, qvel=qvel)

    # The observation is the concatenation of joint positions and velocities.
    obs = jnp.concatenate([data.qpos, data.qvel])

    return data, obs

def env_step(mjx_model, data, action, rng, config):
    """
    Advances the environment by one simulation step.

    Applies the given action, simulates physics, calculates the reward,
    and determines if the episode has ended. A random external force is also
    applied to the pole to make the task more challenging and promote a more
    robust policy.

    Args:
        mjx_model: The compiled MJX model of the environment.
        data: The current mjx.Data state of the simulation.
        action: The action to be applied by the agent.
        rng: A JAX random number generator key.
        config: A dictionary containing configuration parameters like reward weights.

    Returns:
        A tuple containing:
        - next_data (mjx.Data): The simulation state after the step.
        - next_obs (jnp.ndarray): The observation vector for the next state.
        - reward (float): The reward received for the current step.
        - done (float): A flag (1.0 if terminated, 0.0 otherwise).
    """
    # Clip the action to prevent excessively large forces.
    action = jnp.clip(action, -50.0, 50.0)

    # Split the random key for generating the external force.
    rng, key = jax.random.split(rng)

    # Define the magnitude of the random external force.
    force_magnitude = 0.5

    # Generate a random 3D force vector.
    push = jax.random.uniform(key, (3,), minval=-force_magnitude, maxval=force_magnitude)

    # Initialize the external force array (for all bodies).
    xfrc = jnp.zeros((mjx_model.nbody, 6)) # 6D: 3 for force, 3 for torque

    # Apply the random force to the pole (body index 2).
    xfrc = xfrc.at[2, 0:3].set(push)

    # Update the MJX data with the agent's action (control) and the external force.
    data = data.replace(ctrl=action, xfrc_applied=xfrc)

    # Perform one physics step.
    data = mjx.step(mjx_model, data)

    # Construct the next observation from the new state.
    obs = jnp.concatenate([data.qpos, data.qvel])

    # Extract cart position and pole angle for reward calculation.
    cart_pos = data.qpos[0]
    pole_angle = data.qpos[1]

    # Reward for keeping the pole upright. `cos(angle)` is 1 when upright (0 rad), -1 when upside down.
    # We scale it to be in the range [0, 1].
    reward_upright = (1 + jnp.cos(pole_angle)) / 2

    # Reward for keeping the cart centered at the origin.
    reward_centered = config["centered_reward_weight"] * (1.0 - jnp.abs(cart_pos))

    # Cost for using large actions to encourage energy efficiency.
    action_cost = config["action_cost_weight"] * jnp.sum(jnp.square(action))

    # The total reward is a combination of the above components.
    reward = reward_upright - reward_centered - action_cost

    # Termination condition: if the pole falls over too far.
    done = jnp.where(jnp.abs(pole_angle) > 0.5, 1.0, 0.0)

    # Termination condition: if the cart moves too far from the center.
    done = jnp.where(jnp.abs(cart_pos) > 2.5, 1.0, done)

    # If the episode is done, the reward for this step is zeroed out.
    reward = reward * (1.0 - done)

    return data, obs, reward, done


class ActorCritic(nn.Module):
    """
    An Actor-Critic neural network model.

    This network takes an observation as input and outputs:
    1.  The mean of a Gaussian policy (the actor's action distribution).
    2.  The log standard deviation of the policy.
    3.  The estimated state value (the critic's prediction).

    The actor and critic share the initial hidden layers.

    Attributes:
        action_dim (int): The dimensionality of the action space.
    """
    action_dim: int

    @nn.compact
    def __call__(self, x):
        """Forward pass of the network."""
        # Shared hidden layers with tanh activation. Orthogonal initialization is used for stability.
        x = nn.Dense(256, kernel_init=nn.initializers.orthogonal(2))(x)
        x = nn.tanh(x)
        x = nn.Dense(256, kernel_init=nn.initializers.orthogonal(2))(x)
        x = nn.tanh(x)

        # Actor head: outputs the mean of the action distribution.
        # Small initialization weight for the final layer is a common practice.
        mean = nn.Dense(self.action_dim, kernel_init=nn.initializers.orthogonal(0.01))(x)

        # Actor head: outputs the log standard deviation of the action distribution.
        # This is a learnable parameter, independent of the input state.
        log_std = self.param('log_std', nn.initializers.zeros, (self.action_dim,))

        # Critic head: outputs the estimated state value.
        value = nn.Dense(1, kernel_init=nn.initializers.orthogonal(1))(x)

        # Squeeze the last dimension of the value output to get a scalar.
        return mean, log_std, value.squeeze(-1)


def train(config):
    """
    Main training function for the PPO agent.

    This function sets up the environment, initializes the network and optimizer,
    and runs the main training loop. The loop consists of collecting trajectories
    by interacting with parallel environments, calculating advantages using GAE,

    and updating the network parameters using the PPO loss function.

    Args:
        config (dict): A dictionary containing all hyperparameters for training.
    """
    # Load the MuJoCo model from an XML file.
    mj_model = mujoco.MjModel.from_xml_path(config["model_path"])
    # Compile the model for JAX (MJX).
    mjx_model = mjx.put_model(mj_model)

    # --- JAX Function Transformations ---
    # `functools.partial` pre-fills some arguments of the functions.
    partial_reset = functools.partial(env_reset, mjx_model)
    partial_step = functools.partial(env_step, mjx_model, config=config)

    # `jax.vmap` vectorizes the functions to run them over many environments in parallel.
    vmap_reset = jax.vmap(partial_reset)
    vmap_step = jax.vmap(partial_step)

    # `jax.jit` just-in-time compiles the functions for maximum performance.
    jit_reset = jax.jit(vmap_reset)
    jit_step = jax.jit(vmap_step)

    # --- Initialization ---
    # Initialize the Actor-Critic network.
    network = ActorCritic(action_dim=mjx_model.nu)

    # Initialize the Adam optimizer.
    optimizer = optax.adam(config["learning_rate"])

    # Initialize the main random number generator key.
    rng = jax.random.PRNGKey(0)

    # Create a dummy observation to initialize the network parameters.
    rng, key = jax.random.split(rng)
    init_obs = jnp.zeros((config["num_envs"], mjx_model.nq + mjx_model.nv))
    network_params = network.init(key, init_obs)['params']

    # Initialize the optimizer state.
    opt_state = optimizer.init(network_params)

    # Create separate random keys for each parallel environment.
    rngs = jax.random.split(rng, config["num_envs"])

    # Reset all parallel environments to get the initial states and observations.
    data, obs = jit_reset(rngs)

    print("Starting training...")
    start_time = time.time()

    # Calculate the total number of update steps.
    num_updates = config["total_timesteps"] // (config["num_envs"] * config["unroll_length"])

    # Main training loop.
    for update_step in range(1, num_updates + 1):
        # --- Trajectory Collection (Rollout) Phase ---
        trajectory = []
        for _ in range(config["unroll_length"]):
            # Split RNG key for action sampling and environment stepping.
            rng, key, step_key = jax.random.split(rng, 3)

            # Get policy and value predictions from the network.
            mean, log_std, value = network.apply({'params': network_params}, obs)

            # Sample an action from the policy's Gaussian distribution.
            std = jnp.exp(log_std)
            action = mean + jax.random.normal(key, shape=mean.shape) * std

            # Calculate the log probability of the sampled action.
            # This is needed for the PPO loss calculation.
            log_prob = -0.5 * (((action - mean) / std)**2 + 2 * jnp.log(std) + jnp.log(2 * jnp.pi)).sum(-1)

            # Step the parallel environments.
            step_rngs = jax.random.split(step_key, config["num_envs"])
            next_data, next_obs, reward, done = jit_step(data, action, step_rngs)

            # Store the transition in the trajectory buffer.
            trajectory.append((obs, action, reward, done, value, log_prob))

            # Update the state for the next iteration.
            data, obs = next_data, next_obs

        # --- Advantage Calculation Phase ---
        # Get the value of the last state to bootstrap the advantage calculation.
        _, _, last_val = network.apply({'params': network_params}, obs)
        advantages_list = []
        # Generalized Advantage Estimation (GAE).
        gae = 0
        # Iterate backwards through the collected trajectory.
        for i in reversed(range(config["unroll_length"])):
            _, _, reward_t, done_t, value_t, _ = trajectory[i]
            # Calculate the TD error (delta).
            delta = reward_t + config["gamma"] * last_val * (1 - done_t) - value_t
            # Update GAE: $A_{t}^{GAE} = \delta_t + \gamma \lambda (1-d_{t+1}) A_{t+1}^{GAE}$
            gae = delta + config["gamma"] * config["gae_lambda"] * (1 - done_t) * gae
            advantages_list.append(gae)
            # The value of the current state becomes the 'last_val' for the previous timestep.
            last_val = value_t

        # Reverse the list of advantages to match the trajectory order and stack into a JAX array.
        advantages = jnp.stack(advantages_list[::-1])

        # Stack all other trajectory data into JAX arrays.
        traj_obs, traj_actions, _, _, traj_values, traj_log_probs = [jnp.stack(x) for x in zip(*trajectory)]

        # Calculate returns (target values for the critic) by adding advantages to the values.
        traj_returns = advantages + traj_values

        # --- PPO Update Phase ---
        def update_epoch(carry, _):
            """
            Performs one epoch of updates over the collected trajectory data.
            This function is scanned over using `jax.lax.scan`.
            """
            params, opt_state, rng_epoch = carry
            rng_epoch, key_epoch = jax.random.split(rng_epoch)

            total_batch_size = config["num_envs"] * config["unroll_length"]
            # Generate a random permutation to shuffle the data for each epoch.
            permutation = jax.random.permutation(key_epoch, total_batch_size)

            def loss_fn(params_update):
                """Calculates the PPO loss for a batch of data."""
                # Flatten the trajectory data from [unroll_length, num_envs, ...] to [total_batch_size, ...].
                flat_obs, flat_actions = traj_obs.reshape((total_batch_size, -1)), traj_actions.reshape((total_batch_size, -1))
                flat_advantages, flat_returns, flat_log_probs = advantages.reshape(-1), traj_returns.reshape(-1), traj_log_probs.reshape(-1)

                # Get network outputs for the shuffled observations.
                mean, log_std, value = network.apply({'params': params_update}, flat_obs[permutation])

                # Calculate the log probability of actions from the trajectory under the *current* policy.
                new_log_prob = -0.5 * (((flat_actions[permutation] - mean) / jnp.exp(log_std))**2 + 2 * log_std + jnp.log(2 * jnp.pi)).sum(-1)

                # Calculate the importance sampling ratio.
                ratio = jnp.exp(new_log_prob - flat_log_probs[permutation])

                # PPO's clipped surrogate objective for the actor.
                clip_adv = jnp.clip(ratio, 1 - config["clip_coef"], 1 + config["clip_coef"]) * flat_advantages[permutation]
                actor_loss = -jnp.minimum(ratio * flat_advantages[permutation], clip_adv).mean()

                # Mean squared error for the critic (value function).
                critic_loss = ((value - flat_returns[permutation])**2).mean()

                # Total loss is a weighted sum of actor and critic losses.
                return actor_loss + config["vf_coef"] * critic_loss

            # Calculate gradients of the loss with respect to the network parameters.
            grads = jax.grad(loss_fn)(params)
            # Update the optimizer state.
            updates, opt_state = optimizer.update(grads, opt_state)
            # Apply the updates to the network parameters.
            params = optax.apply_updates(params, updates)

            return (params, opt_state, rng_epoch), None

        # Run the update over multiple epochs using `jax.lax.scan` for efficiency.
        (network_params, opt_state, rng), _ = jax.lax.scan(
            update_epoch,
            (network_params, opt_state, rng),
            None,
            length=config["num_update_epochs"]
        )

        # --- Logging ---
        if update_step % 10 == 0:
            total_steps = update_step * config["num_envs"] * config["unroll_length"]
            elapsed_time = time.time() - start_time
            steps_per_sec = total_steps / elapsed_time if elapsed_time > 0 else 0
            print(f"Update: {update_step}/{num_updates}, Steps: {total_steps:,}, SPS: {steps_per_sec:,.0f}, Time: {elapsed_time:.2f}s")

    # --- Save Final Policy ---
    # Use Orbax to save the trained network parameters.
    checkpointer = ocp.PyTreeCheckpointer()
    checkpointer.save(config["save_path"], {'params': network_params}, force=True)
    print(f"\nTraining complete. Policy saved to {config['save_path']}")

if __name__ == '__main__':
    # Define the root directory for model and policy paths.
    workspace_root = "/root/ros2_ws"

    # Print the default JAX backend (e.g., 'gpu', 'cpu', 'tpu').
    print(f"JAX backend: {jax.default_backend()}")

    # Configuration dictionary for all hyperparameters.
    config = {
        # PPO Hyperparameters
        "learning_rate": 3e-4,
        "num_envs": 16384,          # Number of parallel environments
        "total_timesteps": 30_000_000,
        "unroll_length": 128,       # Number of steps per environment per update
        "num_update_epochs": 8,     # Number of optimization epochs per update
        "gamma": 0.99,              # Discount factor
        "gae_lambda": 0.95,         # GAE lambda parameter
        "clip_coef": 0.2,           # PPO clipping coefficient
        "vf_coef": 0.5,             # Value function loss coefficient

        # Environment/Reward Hyperparameters
        "action_repeat": 4,         # (Note: Not used in this version of env_step)
        "centered_reward_weight": 0.1,
        "action_cost_weight": 0.001,

        # File Paths
        "model_path": os.path.join(workspace_root, 'src/mujoco_ros2_control_demos/mujoco_models/balance_bot.xml'),
        "save_path": os.path.join(workspace_root, "src/mujoco_ros2_control_demos/mujoco_models/balance_bot_policy_mjx")
    }

    # Start the training process.
    train(config)
