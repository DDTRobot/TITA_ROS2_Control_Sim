// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
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

#include "titati_webots_bridge/webots_keyboard.hpp"

void WebotsKeyboard::init(int sampling_period)
{
  wb_keyboard_enable(sampling_period);
  cmd_vel_msg_ = std::make_shared<geometry_msgs::msg::Twist>();
  posestampd_msg_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
  fsm_msg_ = std::make_shared<std_msgs::msg::String>();
  fsm_msg_->data = "idle";
  speed_psc = speed_rate_psc = 1.0f;
  for (size_t i = 0; i < 3; i++) {
    rpy_[i] = 0.f;
  }
}

void WebotsKeyboard::update(int sampling_period)
{
  float dt = static_cast<float>(sampling_period / 1000.0f);

  int16_t key_status = wb_keyboard_get_key();
  float speed_psc = 1.0f;
  float speed_psc_rotate = 3.0f;
  switch (key_status) {
    case -1:
      cmd_vel_msg_->linear.x = 0.f;
      cmd_vel_msg_->angular.z = 0.f;
      rpy_[ROLL] = rpy_[PITCH] = rpy_[YAW] = 0.f;
      posestampd_msg_->pose.position.y = 0.f;
      break;
    case WebotsKeyboard::robot_key::SPEED_RATE_UP:
      if (key_status != this->key_status_pre) {
        speed_rate_psc /= 0.9f;
      }
      break;
    case WebotsKeyboard::robot_key::SPEED_RATE_DOWN:
      if (key_status != this->key_status_pre) {
        speed_rate_psc *= 0.9f;
      }
      break;
    case WebotsKeyboard::robot_key::SPEED_UP:
      if (key_status != this->key_status_pre) {
        speed_psc /= 0.9f;
      }
      break;
    case WebotsKeyboard::robot_key::SPEED_DOWN:
      if (key_status != this->key_status_pre) {
        speed_psc *= 0.9f;
      }
      break;
    case WebotsKeyboard::robot_key::FOWARD:
      cmd_vel_msg_->linear.x += speed_rate_psc * dt;
      break;
    case WebotsKeyboard::robot_key::BACK:
      cmd_vel_msg_->linear.x -= speed_rate_psc * dt;
      break;
    case WebotsKeyboard::robot_key::LEFT:
      cmd_vel_msg_->angular.z += speed_rate_psc * speed_psc_rotate * dt;
      break;
    case WebotsKeyboard::robot_key::RIGHT:
      cmd_vel_msg_->angular.z -= speed_rate_psc * speed_psc_rotate * dt;
      break;
    /*Head motion instruction:make the robot head turn up,dowm and turn left and right.*/
    case WebotsKeyboard::robot_key::HEAD_HEIGHT_UP:
      posestampd_msg_->pose.position.z += speed_rate_psc * dt;
      break;
    case WebotsKeyboard::robot_key::HEAD_HEIGHT_DOWN:
      posestampd_msg_->pose.position.z -= speed_rate_psc * dt;
      break;
    case WebotsKeyboard::robot_key::HEAD_Y_LEFT:
      posestampd_msg_->pose.position.y += speed_rate_psc * dt;
      break;
    case WebotsKeyboard::robot_key::HEAD_Y_RIGHT:
      posestampd_msg_->pose.position.y -= speed_rate_psc * dt;
      break;
    case WebotsKeyboard::robot_key::HEAD_PITCH_UP:
      rpy_[PITCH] += speed_rate_psc * dt;
      break;
    case WebotsKeyboard::robot_key::HEAD_PITCH_DOWN:
      rpy_[PITCH] -= speed_rate_psc * dt;
      break;
    case WebotsKeyboard::robot_key::HEAD_ROLL_LEFT:
      rpy_[ROLL] += speed_rate_psc * dt;
      break;
    case WebotsKeyboard::robot_key::HEAD_ROLL_RIGHT:
      rpy_[ROLL] -= speed_rate_psc * dt;
      break;
    case WebotsKeyboard::robot_key::LEG_SPLIT_X_UP:
      rpy_[YAW] += speed_rate_psc * dt;
      break;
    case WebotsKeyboard::robot_key::LEG_SPLIT_X_DOWN:
      rpy_[YAW] -= speed_rate_psc * dt;
      break;
    // case WebotsKeyboard::robot_key::LEG_SPLIT_Y_UP:
    //   rc.leg_split_in_y += speed_rate_psc * dt;
    //   bound(rc.leg_split_in_y, MIN_SPLIT_Y);
    //   break;
    // case WebotsKeyboard::robot_key::LEG_SPLIT_Y_DOWN:
    //   rc.leg_split_in_y -= speed_rate_psc * dt;
    //   bound(rc.leg_split_in_y, MIN_SPLIT_Y);
    //   break;
    case WebotsKeyboard::robot_key::KEY_0:
      fsm_msg_->data = "idle";
      break;
    case WebotsKeyboard::robot_key::KEY_1:  // TODO(lkx): delete
      fsm_msg_->data = "transform_up";
      break;
    case WebotsKeyboard::robot_key::KEY_2:
      fsm_msg_->data = "balance_stand";
      break;
    case WebotsKeyboard::robot_key::KEY_3:
      fsm_msg_->data = "jump";
      break;
    case WebotsKeyboard::robot_key::KEY_4:
      fsm_msg_->data = "emergency_stop";
      break;
    case WebotsKeyboard::robot_key::KEY_5:
      // rc.FSM = 5;
      break;
    case WebotsKeyboard::robot_key::KEY_6:
      // rc.FSM = 6;
      break;

    default:
      break;
  }
  bound(cmd_vel_msg_->linear.x, MAX_FORWARD_SPEED);
  bound(cmd_vel_msg_->angular.z, MAX_ROTATION_SPEED);
  bound(posestampd_msg_->pose.position.z, MAX_HEIGHT, MIN_HEIGHT);
  bound(posestampd_msg_->pose.position.y, MAX_Y);
  bound(rpy_[PITCH], MAX_PITCH_ANGLE);
  bound(rpy_[ROLL], MAX_ROLL_ANGLE);
  bound(rpy_[YAW], MAX_SPLIT_X);
  // from rpy to quaternion
  tf2::Quaternion q = tf2::Quaternion();
  q.setRPY(rpy_[ROLL], rpy_[PITCH], rpy_[YAW]);
  posestampd_msg_->pose.orientation.x = q.x();
  posestampd_msg_->pose.orientation.y = q.y();
  posestampd_msg_->pose.orientation.z = q.z();
  posestampd_msg_->pose.orientation.w = q.w();

  key_status_pre = key_status;
}
