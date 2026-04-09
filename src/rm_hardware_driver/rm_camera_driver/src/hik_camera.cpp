// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rm_camera_driver/hik_camera.hpp"
// std
#include <chrono>
#include <thread>
// project
#include "rm_utils/logger/log.hpp"

namespace fyt::camera_driver {

HikCameraNode::HikCameraNode(const rclcpp::NodeOptions & options)
: Node("camera_driver", options)
{
  FYT_REGISTER_LOGGER("camera_driver", "~/fyt2024-log", INFO);
  FYT_INFO("camera_driver", "Starting HikCameraNode!");

  MV_CC_DEVICE_INFO_LIST device_list;
  // enum device
  nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  FYT_INFO("camera_driver", "Found camera count = {}", device_list.nDeviceNum);

  while (device_list.nDeviceNum == 0 && rclcpp::ok()) {
    FYT_ERROR("camera_driver", "No camera found!");
    FYT_INFO("camera_driver", "Enum state: [{}]", nRet);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  }

  MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
  MV_CC_OpenDevice(camera_handle_);

  // Get camera information
  MV_CC_GetImageInfo(camera_handle_, &img_info_);
  image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);

  // Init convert param
  convert_param_.nWidth = img_info_.nWidthValue;
  convert_param_.nHeight = img_info_.nHeightValue;
  convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

  bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
  auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
  camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

  // Heartbeat
  heartbeat_ = HeartBeatPublisher::create(this);

  declareParameters();

  MV_CC_StartGrabbing(camera_handle_);

  // Load camera info
  camera_name_ = this->declare_parameter("camera_name", "hikvision");
  camera_info_manager_ =
    std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
  camera_info_url_ =
    this->declare_parameter("camera_info_url", "package://rm_bringup/config/camera_info.yaml");
  if (camera_info_manager_->validateURL(camera_info_url_)) {
    camera_info_manager_->loadCameraInfo(camera_info_url_);
    camera_info_msg_ = camera_info_manager_->getCameraInfo();
  } else {
    FYT_WARN("camera_driver", "Invalid camera info URL: {}", camera_info_url_);
  }
  camera_info_msg_.header.stamp = this->now();

  // Check if camera is alive every second
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000), std::bind(&HikCameraNode::timerCallback, this));

  params_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

  capture_thread_ = std::thread{[this]() -> void {
    MV_FRAME_OUT out_frame;

    FYT_INFO("camera_driver", "Publishing image!");

    image_msg_.header.frame_id = "camera_optical_frame";
    image_msg_.encoding = "rgb8";

    while (rclcpp::ok()) {
      nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
      if (MV_OK == nRet) {
        convert_param_.pDstBuffer = image_msg_.data.data();
        convert_param_.nDstBufferSize = image_msg_.data.size();
        convert_param_.pSrcData = out_frame.pBufAddr;
        convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
        convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

        MV_CC_ConvertPixelType(camera_handle_, &convert_param_);

        image_msg_.header.stamp = this->now();
        image_msg_.height = out_frame.stFrameInfo.nHeight;
        image_msg_.width = out_frame.stFrameInfo.nWidth;
        image_msg_.step = out_frame.stFrameInfo.nWidth * 3;
        image_msg_.data.resize(image_msg_.width * image_msg_.height * 3);

        camera_info_msg_.header = image_msg_.header;
        camera_pub_.publish(image_msg_, camera_info_msg_);

        MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
        fail_count_ = 0;
      } else {
        FYT_WARN("camera_driver", "Get buffer failed! nRet: [{}]", nRet);
        MV_CC_StopGrabbing(camera_handle_);
        MV_CC_StartGrabbing(camera_handle_);
        fail_count_++;
      }

      if (fail_count_ > 5) {
        FYT_FATAL("camera_driver", "Camera failed!");
        rclcpp::shutdown();
      }
    }
  }};
}

HikCameraNode::~HikCameraNode()
{
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  if (camera_handle_) {
    MV_CC_StopGrabbing(camera_handle_);
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(&camera_handle_);
  }
  FYT_INFO("camera_driver", "HikCameraNode destroyed!");
}

void HikCameraNode::timerCallback()
{
  const double dt = (this->now() - rclcpp::Time(camera_info_msg_.header.stamp)).seconds();

  if (dt > 5.0) {
    FYT_WARN("camera_driver", "Camera is not alive! lost frame for {:.2f} seconds", dt);
    MV_CC_StopGrabbing(camera_handle_);
    MV_CC_StartGrabbing(camera_handle_);
    fail_count_ = 0;
  }
}

void HikCameraNode::declareParameters()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  MVCC_FLOATVALUE f_value;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  
  // Exposure time
  param_desc.description = "Exposure time in microseconds";
  MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
  param_desc.integer_range[0].from_value = f_value.fMin;
  param_desc.integer_range[0].to_value = f_value.fMax;
  double exposure_time = this->declare_parameter("exposure_time", 5000, param_desc);
  MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
  FYT_INFO("camera_driver", "Exposure time: {}", exposure_time);

  // Gain
  param_desc.description = "Gain";
  MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
  param_desc.integer_range[0].from_value = f_value.fMin;
  param_desc.integer_range[0].to_value = f_value.fMax;
  double gain = this->declare_parameter("gain", f_value.fCurValue, param_desc);
  MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
  FYT_INFO("camera_driver", "Gain: {}", gain);
}

rcl_interfaces::msg::SetParametersResult HikCameraNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & param : parameters) {
    if (param.get_name() == "exposure_time") {
      int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
      if (MV_OK != status) {
        result.successful = false;
        result.reason = "Failed to set exposure time, status = " + std::to_string(status);
      }
    } else if (param.get_name() == "gain") {
      int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
      if (MV_OK != status) {
        result.successful = false;
        result.reason = "Failed to set gain, status = " + std::to_string(status);
      }
    } else {
      result.successful = false;
      result.reason = "Unknown parameter: " + param.get_name();
    }
  }
  return result;
}

}  // namespace fyt::camera_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(fyt::camera_driver::HikCameraNode)
