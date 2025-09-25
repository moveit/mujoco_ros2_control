## Reinforcement Learning Example

### Description
The balance_bot.launch.py example is a demo that showcases:

GPU Accelerated 
- Physics with Mujoco Warp
- Graphics & Rendering with Mujoco
- Machine Learning with MJX (JAX)

The three example files are under mujoco_ros2_control_demos/scripts/
- train_agent.py
    - A python script to train the model in a MJX simulation
- rl_controller_agent.py
    - A ROS2 Node that uses the trained model to try to balance a pole on a cart
- rl_controller_PID.py
    - A ROS2 Node that uses a Proportional Integral to try to balance the pole on a cart

*Currently neither the agent or the PID controller are tuned and this is just an example of the possibilities. 
**If you tune either the agent or PID to balance the pole, please send me a message! -ogordill (Orlando Gordillo)

### How to run

```
cd ~/ros2_ws/src/mujoco_ros2_control_demos/scripts
python3 train_agent.py
```

After training is complete, run the alias command:

```
balance_bot
```

This will start the sim. There is a mouse input coded into mujoco_ros2_control/src/mujoco_rendering.cpp that pushes the pole when it is clicked to get the pole off balance. 

### Documentation
Read the (Mujoco Open Source documentation / API)[https://mujoco.readthedocs.io/en/stable/APIreference/index.html]

