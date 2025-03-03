// Copyright (c) 2025 Sangtaek Lee
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "mujoco_ros2_control/mujoco_rendering.hpp"

#include "sensor_msgs/image_encodings.hpp"

namespace mujoco_ros2_control
{
MujocoRendering *MujocoRendering::instance_ = nullptr;

MujocoRendering *MujocoRendering::get_instance()
{
  if (instance_ == nullptr)
  {
    instance_ = new MujocoRendering();
  }

  return instance_;
}

MujocoRendering::MujocoRendering()
    : mj_model_(nullptr),
      mj_data_(nullptr),
      button_left_(false),
      button_middle_(false),
      button_right_(false),
      lastx_(0.0),
      lasty_(0.0)
{
}

void MujocoRendering::init(
  rclcpp::Node::SharedPtr &node, mjModel *mujoco_model, mjData *mujoco_data)
{
  node_ = node;
  mj_model_ = mujoco_model;
  mj_data_ = mujoco_data;

  // init GLFW
  if (!glfwInit())
  {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
  glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
  window_ = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window_);

  // initialize visualization data structures
  mjv_defaultCamera(&mjv_cam_);
  mjv_defaultOption(&mjv_opt_);
  mjv_defaultScene(&mjv_scn_);
  mjr_defaultContext(&mjr_con_);

  mjv_cam_.type = mjCAMERA_FREE;
  mjv_cam_.distance = 8.;

  // create scene and context
  mjv_makeScene(mj_model_, &mjv_scn_, 2000);
  mjr_makeContext(mj_model_, &mjr_con_, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window_, &MujocoRendering::keyboard_callback);
  glfwSetCursorPosCallback(window_, &MujocoRendering::mouse_move_callback);
  glfwSetMouseButtonCallback(window_, &MujocoRendering::mouse_button_callback);
  glfwSetScrollCallback(window_, &MujocoRendering::scroll_callback);

  // Add user cameras
  register_cameras();

  // This might cause tearing, but having RViz and the renderer both open can
  // wreak havoc on the rendering process.
  glfwSwapInterval(0);
}

bool MujocoRendering::is_close_flag_raised() { return glfwWindowShouldClose(window_); }

void MujocoRendering::update()
{
  // get framebuffer viewport
  mjrRect viewport = {0, 0, 0, 0};
  glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
  glfwMakeContextCurrent(window_);

  // Reset the buffer
  mjr_setBuffer(mjFB_WINDOW, &mjr_con_);

  // update scene and render
  mjv_updateScene(mj_model_, mj_data_, &mjv_opt_, NULL, &mjv_cam_, mjCAT_ALL, &mjv_scn_);
  mjr_render(viewport, &mjv_scn_, &mjr_con_);

  // swap OpenGL buffers (blocking call due to v-sync)
  glfwSwapBuffers(window_);

  // process pending GUI events, call GLFW callbacks
  glfwPollEvents();
}

void MujocoRendering::update_cameras()
{
  // Rendering is done offscreen
  mjr_setBuffer(mjFB_OFFSCREEN, &mjr_con_);

  for (auto &camera : cameras_)
  {
    // Render simple RGB data for all cameras
    mjv_updateScene(mj_model_, mj_data_, &mjv_opt_, NULL, &camera.mjv_cam, mjCAT_ALL, &mjv_scn_);
    mjr_render(camera.viewport, &mjv_scn_, &mjr_con_);

    // Copy image into the ROS message
    mjr_readPixels(camera.image.data.data(), nullptr, camera.viewport, &mjr_con_);

    // Publish
    auto time = node_->now();
    camera.image.header.stamp = time;
    camera.camera_info.header.stamp = time;
    camera.image_pub->publish(camera.image);
    camera.camera_info_pub->publish(camera.camera_info);
  }
}

void MujocoRendering::close()
{
  // free visualization storage
  mjv_freeScene(&mjv_scn_);
  mjr_freeContext(&mjr_con_);
  glfwDestroyWindow(window_);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif
}

void MujocoRendering::keyboard_callback(
  GLFWwindow *window, int key, int scancode, int act, int mods)
{
  get_instance()->keyboard_callback_impl(window, key, scancode, act, mods);
}

void MujocoRendering::mouse_button_callback(GLFWwindow *window, int button, int act, int mods)
{
  get_instance()->mouse_button_callback_impl(window, button, act, mods);
}

void MujocoRendering::mouse_move_callback(GLFWwindow *window, double xpos, double ypos)
{
  get_instance()->mouse_move_callback_impl(window, xpos, ypos);
}

void MujocoRendering::scroll_callback(GLFWwindow *window, double xoffset, double yoffset)
{
  get_instance()->scroll_callback_impl(window, xoffset, yoffset);
}

void MujocoRendering::keyboard_callback_impl(
  GLFWwindow * /* window */, int key, int /* scancode */, int act, int /* mods */)
{
  // backspace: reset simulation
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
  {
    mj_resetData(mj_model_, mj_data_);
    mj_forward(mj_model_, mj_data_);
  }
}

void MujocoRendering::mouse_button_callback_impl(
  GLFWwindow *window, int /* button */, int /* act */, int /* mods */)
{
  // update button state
  button_left_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx_, &lasty_);
}

void MujocoRendering::mouse_move_callback_impl(GLFWwindow *window, double xpos, double ypos)
{
  // no buttons down: nothing to do
  if (!button_left_ && !button_middle_ && !button_right_)
  {
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
  bool mod_shift =
    (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
     glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

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
  mjv_moveCamera(mj_model_, action, dx / height, dy / height, &mjv_scn_, &mjv_cam_);
}

void MujocoRendering::scroll_callback_impl(
  GLFWwindow * /* window */, double /* xoffset */, double yoffset)
{
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(mj_model_, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &mjv_scn_, &mjv_cam_);
}

void MujocoRendering::register_cameras()
{
  cameras_.resize(0);
  for (auto i = 0; i < mj_model_->ncam; ++i)
  {
    const char *cam_name = mj_model_->names + mj_model_->name_camadr[i];
    const int *cam_resolution = mj_model_->cam_resolution + 2 * i;
    const float *cam_intrinsic = mj_model_->cam_intrinsic + 4 * i;

    auto fx = static_cast<double>(cam_intrinsic[0]);
    auto fy = static_cast<double>(cam_intrinsic[1]);
    auto cx = static_cast<double>(cam_intrinsic[2]);
    auto cy = static_cast<double>(cam_intrinsic[3]);

    // Construct CameraData wrapper and set defaults
    CameraData camera;
    camera.name = cam_name;
    camera.mjv_cam.type = mjCAMERA_FIXED;
    camera.mjv_cam.fixedcamid = i;
    camera.width = static_cast<uint32_t>(cam_resolution[0]);
    camera.height = static_cast<uint32_t>(cam_resolution[1]);
    camera.viewport = {0, 0, cam_resolution[0], cam_resolution[1]};

    // TODO(eholum): Ensure that the camera is attached to the expected pose.
    // For now assume that's the case.
    camera.frame_name = camera.name + "_optical_frame";
    camera.image.header.frame_id = camera.frame_name;
    camera.camera_info.header.frame_id = camera.frame_name;

    // Configure publishers
    camera.image_pub = node_->create_publisher<sensor_msgs::msg::Image>(camera.name + "/color", 10);
    camera.camera_info_pub =
      node_->create_publisher<sensor_msgs::msg::CameraInfo>(camera.name + "/camera_info", 10);

    // Set defaults for the image and camera_info, hardcoding for now
    camera.image.data.resize(camera.width * camera.height * 3);
    camera.image.width = camera.width;
    camera.image.height = camera.height;
    camera.image.step = camera.width * 3;

    camera.camera_info.width = camera.width;
    camera.camera_info.height = camera.height;
    camera.image.encoding = sensor_msgs::image_encodings::RGB8;
    camera.camera_info.distortion_model = "plumb_bob";
    camera.camera_info.d.resize(5, 0.0);

    camera.camera_info.k.fill(0.0);
    camera.camera_info.k[0] = fx;
    camera.camera_info.k[2] = cx;
    camera.camera_info.k[4] = fy;
    camera.camera_info.k[5] = cy;
    camera.camera_info.k[8] = 1.0;

    camera.camera_info.p.fill(0.0);
    camera.camera_info.p[0] = fx;
    camera.camera_info.p[2] = cx;
    camera.camera_info.p[5] = fy;
    camera.camera_info.p[6] = cy;
    camera.camera_info.p[10] = 1.0;

    camera.camera_info.r.fill(0.0);
    camera.camera_info.r[0] = 1.0;
    camera.camera_info.r[4] = 1.0;
    camera.camera_info.r[8] = 1.0;

    // Add to list of cameras
    cameras_.push_back(camera);
  }
}

}  // namespace mujoco_ros2_control
