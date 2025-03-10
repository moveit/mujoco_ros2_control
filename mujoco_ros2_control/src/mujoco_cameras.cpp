// Copyright (c) 2025 Erik Holum
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

#include "mujoco_ros2_control/mujoco_cameras.hpp"

#include "sensor_msgs/image_encodings.hpp"

namespace mujoco_ros2_control
{

MujocoCameras::MujocoCameras(rclcpp::Node::SharedPtr &node) : node_(node) {}

void MujocoCameras::init(mjModel *mujoco_model)
{
  // initialize visualization data structures
  mjv_defaultOption(&mjv_opt_);
  mjv_defaultScene(&mjv_scn_);
  mjr_defaultContext(&mjr_con_);

  // create scene and context
  mjv_makeScene(mujoco_model, &mjv_scn_, 2000);
  mjr_makeContext(mujoco_model, &mjr_con_, mjFONTSCALE_150);

  // Add user cameras
  register_cameras(mujoco_model);
}

void MujocoCameras::update(mjModel *mujoco_model, mjData *mujoco_data)
{
  // Rendering is done offscreen
  mjr_setBuffer(mjFB_OFFSCREEN, &mjr_con_);

  for (auto &camera : cameras_)
  {
    // Render simple RGB data for all cameras
    mjv_updateScene(
      mujoco_model, mujoco_data, &mjv_opt_, NULL, &camera.mjv_cam, mjCAT_ALL, &mjv_scn_);
    mjr_render(camera.viewport, &mjv_scn_, &mjr_con_);

    // Copy image into relevant buffers
    mjr_readPixels(
      camera.image_buffer.data(), camera.depth_buffer.data(), camera.viewport, &mjr_con_);

    // Fix non-linear projections in the depth image and flip the data.
    // https://github.com/google-deepmind/mujoco/blob/3.2.7/python/mujoco/renderer.py#L190
    float near = static_cast<float>(mujoco_model->vis.map.znear * mujoco_model->stat.extent);
    float far = static_cast<float>(mujoco_model->vis.map.zfar * mujoco_model->stat.extent);
    for (uint32_t h = 0; h < camera.height; h++)
    {
      for (uint32_t w = 0; w < camera.width; w++)
      {
        auto idx = h * camera.width + w;
        auto idx_flipped = (camera.height - 1 - h) * camera.width + w;
        camera.depth_buffer[idx] = near / (1.0f - camera.depth_buffer[idx] * (1.0f - near / far));
        camera.depth_buffer_flipped[idx_flipped] = camera.depth_buffer[idx];
      }
    }
    // Copy flipped data into the depth image message, floats -> unsigned chars
    std::memcpy(
      &camera.depth_image.data[0], camera.depth_buffer_flipped.data(),
      camera.depth_image.data.size());

    // OpenGL's coordinate system's origin is in the bottom left, so we invert the images row-by-row
    auto row_size = camera.width * 3;
    for (uint32_t h = 0; h < camera.height; h++)
    {
      auto src_idx = h * row_size;
      auto dest_idx = (camera.height - 1 - h) * row_size;
      std::memcpy(&camera.image.data[dest_idx], &camera.image_buffer[src_idx], row_size);
    }

    // Publish images and camera info
    auto time = node_->now();
    camera.image.header.stamp = time;
    camera.depth_image.header.stamp = time;
    camera.camera_info.header.stamp = time;

    camera.image_pub->publish(camera.image);
    camera.depth_image_pub->publish(camera.depth_image);
    camera.camera_info_pub->publish(camera.camera_info);
  }
}

void MujocoCameras::close()
{
  mjv_freeScene(&mjv_scn_);
  mjr_freeContext(&mjr_con_);
}

void MujocoCameras::register_cameras(const mjModel *mujoco_model)
{
  cameras_.resize(0);
  for (auto i = 0; i < mujoco_model->ncam; ++i)
  {
    const char *cam_name = mujoco_model->names + mujoco_model->name_camadr[i];
    const int *cam_resolution = mujoco_model->cam_resolution + 2 * i;
    const mjtNum cam_fovy = mujoco_model->cam_fovy[i];

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

    // Configure publishers
    camera.image_pub = node_->create_publisher<sensor_msgs::msg::Image>(camera.name + "/color", 1);
    camera.depth_image_pub =
      node_->create_publisher<sensor_msgs::msg::Image>(camera.name + "/depth", 1);
    camera.camera_info_pub =
      node_->create_publisher<sensor_msgs::msg::CameraInfo>(camera.name + "/camera_info", 1);

    // Setup contaienrs for color image data
    camera.image.header.frame_id = camera.frame_name;

    const auto image_size = camera.width * camera.height * 3;
    camera.image_buffer.resize(image_size);
    camera.image.data.resize(image_size);
    camera.image.width = camera.width;
    camera.image.height = camera.height;
    camera.image.step = camera.width * 3;
    camera.image.encoding = sensor_msgs::image_encodings::RGB8;

    // Depth image data
    camera.depth_image.header.frame_id = camera.frame_name;
    camera.depth_buffer.resize(camera.width * camera.height);
    camera.depth_buffer_flipped.resize(camera.width * camera.height);
    camera.depth_image.data.resize(camera.width * camera.height * sizeof(float));
    camera.depth_image.width = camera.width;
    camera.depth_image.height = camera.height;
    camera.depth_image.step = camera.width * sizeof(float);
    camera.depth_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

    // Camera info
    camera.camera_info.header.frame_id = camera.frame_name;
    camera.camera_info.width = camera.width;
    camera.camera_info.height = camera.height;
    camera.camera_info.distortion_model = "plumb_bob";
    camera.camera_info.k.fill(0.0);
    camera.camera_info.r.fill(0.0);
    camera.camera_info.p.fill(0.0);
    camera.camera_info.d.resize(5, 0.0);

    double focal_scaling = (1.0 / std::tan((cam_fovy * M_PI / 180.0) / 2.0)) * camera.height / 2.0;
    camera.camera_info.k[0] = camera.camera_info.p[0] = focal_scaling;
    camera.camera_info.k[2] = camera.camera_info.p[2] = static_cast<double>(camera.width) / 2.0;
    camera.camera_info.k[4] = camera.camera_info.p[5] = focal_scaling;
    camera.camera_info.k[5] = camera.camera_info.p[6] = static_cast<double>(camera.height) / 2.0;
    camera.camera_info.k[8] = camera.camera_info.p[10] = 1.0;

    // Add to list of cameras
    cameras_.push_back(camera);
  }
}

}  // namespace mujoco_ros2_control
