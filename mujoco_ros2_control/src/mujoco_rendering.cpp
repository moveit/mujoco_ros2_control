#include "mujoco_ros2_control/mujoco_rendering.hpp"

namespace mujoco_ros2_control
{
MujocoRendering* MujocoRendering::instance_ = nullptr;

MujocoRendering* MujocoRendering::get_instance()
{
  if(instance_ == nullptr)
  {
    instance_ = new MujocoRendering();
  }

  return instance_;
}

MujocoRendering::MujocoRendering()
  : mj_model_(nullptr), mj_data_(nullptr),
    button_left_(false), button_middle_(false), button_right_(false),
    lastx_(0.0), lasty_(0.0)
{
}

void MujocoRendering::init(rclcpp::Node::SharedPtr & node, mjModel* mujoco_model, mjData* mujoco_data)
{
  node_ = node;
  mj_model_ = mujoco_model;
  mj_data_ = mujoco_data;

  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  window_ = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&mjv_cam_);
  mjv_defaultOption(&mjv_opt_);
  mjv_defaultScene(&mjv_scn_);
  mjr_defaultContext(&mjr_con_);

  mjv_cam_.distance = 10.;

  // create scene and context
  mjv_makeScene(mj_model_, &mjv_scn_, 2000);
  mjr_makeContext(mj_model_, &mjr_con_, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window_, &MujocoRendering::keyboard_callback);
  glfwSetCursorPosCallback(window_, &MujocoRendering::mouse_move_callback);
  glfwSetMouseButtonCallback(window_, &MujocoRendering::mouse_button_callback);
  glfwSetScrollCallback(window_, &MujocoRendering::scroll_callback);
}

bool MujocoRendering::is_close_flag_raised()
{
  return glfwWindowShouldClose(window_);
}

void MujocoRendering::update()
{
  // get framebuffer viewport
  mjrRect viewport = {0, 0, 0, 0};
  glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

  // update scene and render
  mjv_updateScene(mj_model_, mj_data_, &mjv_opt_, NULL, &mjv_cam_, mjCAT_ALL, &mjv_scn_);
  mjr_render(viewport, &mjv_scn_, &mjr_con_);

  // swap OpenGL buffers (blocking call due to v-sync)
  glfwSwapBuffers(window_);

  // process pending GUI events, call GLFW callbacks
  glfwPollEvents();
}

void MujocoRendering::close()
{
  //free visualization storage
  mjv_freeScene(&mjv_scn_);
  mjr_freeContext(&mjr_con_);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif
}

void MujocoRendering::keyboard_callback(GLFWwindow* window, int key, int scancode, int act, int mods)
{
  get_instance()->keyboard_callback_impl(window, key, scancode, act, mods);
}

void MujocoRendering::mouse_button_callback(GLFWwindow* window, int button, int act, int mods)
{
  get_instance()->mouse_button_callback_impl(window, button, act, mods);
}

void MujocoRendering::mouse_move_callback(GLFWwindow* window, double xpos, double ypos)
{
  get_instance()->mouse_move_callback_impl(window, xpos, ypos);
}

void MujocoRendering::scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
  get_instance()->scroll_callback_impl(window, xoffset, yoffset);
}

void MujocoRendering::keyboard_callback_impl(GLFWwindow* window, int key, int scancode, int act, int mods)
{
  // backspace: reset simulation
  if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
    mj_resetData(mj_model_, mj_data_);
    mj_forward(mj_model_, mj_data_);
  }
}

void MujocoRendering::mouse_button_callback_impl(GLFWwindow* window, int button, int act, int mods)
{
  // update button state
  button_left_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
  button_middle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
  button_right_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx_, &lasty_);
}

void MujocoRendering::mouse_move_callback_impl(GLFWwindow* window, double xpos, double ypos)
{
  // no buttons down: nothing to do
  if (!button_left_ && !button_middle_ && !button_right_) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx_;
  double dy = ypos - lasty_;
  lastx_ = xpos;
  lasty_ = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right_)
  {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  }
  else if (button_left_)
  {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  }
  else
  {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(mj_model_, action, dx/height, dy/height, &mjv_scn_, &mjv_cam_);
}

void MujocoRendering::scroll_callback_impl(GLFWwindow* window, double xoffset, double yoffset)
{
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(mj_model_, mjMOUSE_ZOOM, 0, -0.05*yoffset, &mjv_scn_, &mjv_cam_);
}
} // namespace mujoco_ros2_control
