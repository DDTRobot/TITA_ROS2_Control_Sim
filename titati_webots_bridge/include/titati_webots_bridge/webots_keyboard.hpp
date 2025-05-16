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

#ifndef WEBOTS_BRIDGE__WEBOTS_KEYBOARD_HPP_
#define WEBOTS_BRIDGE__WEBOTS_KEYBOARD_HPP_
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "webots/keyboard.h"

#define MAX_FORWARD_SPEED 2.5f
#define MAX_ROTATION_SPEED 10.0f
#define MAX_ROLL_ANGLE 0.2f
#define MAX_PITCH_ANGLE 0.2f
#define MIN_HEIGHT 0.1f
#define MAX_HEIGHT 0.4f
#define MAX_SPLIT_X 0.1f
#define MAX_Y 0.05f
#define MIN_SPLIT_Y 0.1f

template <typename T1, typename T2>
static inline void bound(T1 & input, const T2 max, const T2 min)
{
  if (input > max) {
    input = max;
  } else if (input < min) {
    input = min;
  }
}

template <typename T1, typename T2>
static inline void bound(T1 & input, const T2 max)
{
  bound(input, max, -max);
}

class WebotsKeyboard
{
public:
  WebotsKeyboard() {}
  enum { ROLL, PITCH, YAW };

  enum robot_key {
    /*Motion instruction*/
    FOWARD = 87,  // W
    BACK = 83,    // S
    LEFT = 65,    // A
    RIGHT = 68,   // D

    /*head instrunction*/
    HEAD_HEIGHT_UP = 73,    // I
    HEAD_HEIGHT_DOWN = 75,  // K
    HEAD_Y_LEFT = 74,       // J
    HEAD_Y_RIGHT = 76,      // L
    HEAD_PITCH_UP = 315,    // up arrow
    HEAD_PITCH_DOWN = 317,  // down arrow
    HEAD_ROLL_LEFT = 314,   // left arrow
    HEAD_ROLL_RIGHT = 316,  // right arrow

    LEG_SPLIT_X_UP = 85,    // U
    LEG_SPLIT_X_DOWN = 79,  // O
    LEG_SPLIT_Y_UP = 77,    // M
    LEG_SPLIT_Y_DOWN = 46,  // .
    /*other instruction*/
    SPEED_RATE_UP = 43,    // +
    SPEED_RATE_DOWN = 45,  // -
    SPEED_UP = 42,         // *
    SPEED_DOWN = 47,       // /
    // STATE MACHINE
    KEY_0 = 48,  // 0
    KEY_1 = 49,  // 1
    KEY_2 = 50,  // 0
    KEY_3 = 51,  // 1
    KEY_4 = 52,  // 0
    KEY_5 = 53,  // 1
    KEY_6 = 54,  // 1

    // RESET = 82,  // r
    // JUMP = 32,   // SPACE
    // // JUMP_STEP = 54,                      //6
    // /*robot mode instruction*/
    // DANCE_MODE = 55,  // 7
    // WALK_MODE = 57,   // 9
    // AUTO_DANCE = 80,
    // EMERGENCY_STOP = 4,  // enter
    // BRAKE = 51,          // 3
  };
  void init(int sampling_period);
  void update(int sampling_period);
  std::shared_ptr<geometry_msgs::msg::Twist> cmd_vel_msg_;
  std::shared_ptr<geometry_msgs::msg::PoseStamped> posestampd_msg_;
  std::shared_ptr<std_msgs::msg::String> fsm_msg_;

private:
  float rpy_[3];
  int16_t key_status_pre;
  float speed_psc;
  float speed_rate_psc;
};

#endif  // WEBOTS_BRIDGE__WEBOTS_KEYBOARD_HPP_
