// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// You may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************
// This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
// For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
// To test this example, please follow the commands below.
//
// Open terminal #1
// $ ros2 run dynamixel_sdk_examples read_write_node
//
// Open terminal #2 (run one of below commands at a time)
// $ ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/SetPosition "{id: 1, position: 1000}"
// $ ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition "id: 1"
//
// Author: Will Son
*******************************************************************************/

#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_current.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_current.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "current_read_write_node.hpp"

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE     11
#define ADDR_TORQUE_ENABLE      64
#define ADDR_GOAL_POSITION      116
#define ADDR_PRESENT_POSITION   132

// smh I added these for current control ----------------------------------------------------
#define ADDR_GOAL_CURRENT       102
#define ADDR_PRESENT_CURRENT    126

// Protocol version
#define PROTOCOL_VERSION        2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE                1000000
#define DEVICE_NAME             "/dev/ttyUSB0"

// Joint IDs for OpenManipulator-X (adjust if your IDs are different)
#define JOINT1_ID               11
#define JOINT2_ID               12
#define JOINT3_ID               13
#define JOINT4_ID               14  // actuator right before gripper
#define GRIPPER_ID              15

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
int dxl_comm_result = COMM_TX_FAIL;

// smh I added these for current control ----------------------------------------------------
uint16_t goal_current = 0;
uint16_t present_current = 0;
uint32_t present_position = 0;


// Constructor for the ReadWriteNode class, which is a subclass of Node
ReadWriteNode::ReadWriteNode()
: Node("read_write_node") // Initialize the Node with the name "read_write_node"
{
  RCLCPP_INFO(this->get_logger(), "Run read write node"); // Log the start of the node

  // Declare and initialize a ROS2 parameter 'qos_depth' with a default value of 10
  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0; // Variable to store the value of qos_depth
  this->get_parameter("qos_depth", qos_depth); // Retrieve the value of 'qos_depth' parameter

  // Define Quality of Service (QoS) settings for the ROS2 subscriber
  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  // ----------------------------------------------------------------------
  // set_position subscriber  (Position Control Mode joints)
  // ----------------------------------------------------------------------
  set_position_subscriber_ =
    this->create_subscription<SetPosition>(
      "set_position",
      QOS_RKL10V,
      [this](const SetPosition::SharedPtr msg) -> void
      {
        uint8_t dxl_error = 0;

        // Position Value of X series is 4 byte data.
        uint32_t goal_position_local = static_cast<uint32_t>(msg->position);

        // Write Goal Position (length : 4 bytes)
        int dxl_comm_result_local =
          packetHandler->write4ByteTxRx(
            portHandler,
            static_cast<uint8_t>(msg->id),
            ADDR_GOAL_POSITION,
            goal_position_local,
            &dxl_error
          );

        if (dxl_comm_result_local != COMM_SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result_local));
        } else if (dxl_error != 0) {
          RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
        } else {
          RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", msg->id, msg->position);
        }
      }
    );

  // ----------------------------------------------------------------------
  // set_current subscriber  (Current Control Mode joints, e.g. joint4)
  // ----------------------------------------------------------------------
  set_current_subscriber_ =
    this->create_subscription<SetCurrent>(
      "set_current",
      QOS_RKL10V,
      [this](const SetCurrent::SharedPtr msg) -> void
      {
        uint8_t dxl_error = 0; //Variable to store the error from the DYNAMIXEL

        // Current Value of X series is 2 byte data.
        uint16_t goal_current_local = static_cast<uint16_t>(msg->current);

        // Write Goal Current (length : 2 bytes)
        int dxl_comm_result_local =
          packetHandler->write2ByteTxRx(
            portHandler,
            static_cast<uint8_t>(msg->id),   // Dynamixel ID
            ADDR_GOAL_CURRENT,               // Address to write the goal current
            goal_current_local,              // Goal current value
            &dxl_error                       // Error storage
          );

        // Check and handle communication results or errors
        if (dxl_comm_result_local != COMM_SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result_local));
        } else if (dxl_error != 0) {
          RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
        } else {
          RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Current: %d]", msg->id, msg->current);
        }
      }
    );

  // ----------------------------------------------------------------------
  // get_position service
  // ----------------------------------------------------------------------
  auto get_present_position_srv =
    [this](
      const std::shared_ptr<GetPosition::Request> request,
      std::shared_ptr<GetPosition::Response> response) -> void
    {
      dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        static_cast<uint8_t>(request->id),
        ADDR_PRESENT_POSITION,
        &present_position,
        &dxl_error
      );

      RCLCPP_INFO(
        this->get_logger(),
        "Get [ID: %d] [Present Position: %u]",
        request->id,
        present_position
      );

      response->position = static_cast<int32_t>(present_position);
    };

  get_position_server_ = create_service<GetPosition>("get_position", get_present_position_srv);

  // ----------------------------------------------------------------------
  // get_current service
  // ----------------------------------------------------------------------
  auto get_present_current_srv =
    [this](
      const std::shared_ptr<GetCurrent::Request> request,
      std::shared_ptr<GetCurrent::Response> response) -> void
    {
      dxl_comm_result = packetHandler->read2ByteTxRx(
        portHandler,
        static_cast<uint8_t>(request->id),
        ADDR_PRESENT_CURRENT,
        &present_current,
        &dxl_error
      );

      RCLCPP_INFO(
        this->get_logger(),
        "Get [ID: %d] [Present Current: %d]",
        request->id,
        present_current
      );

      response->current = present_current;
    };

  get_current_server_ = create_service<GetCurrent>("get_current", get_present_current_srv);
}

ReadWriteNode::~ReadWriteNode()
{
}

// ----------------------------------------------------------------------
// Helper: set one Dynamixel into Position Control Mode (and enable torque)
// ----------------------------------------------------------------------
void setupDynamixelPosition(uint8_t dxl_id)
{
  // Use Position Control Mode 
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    3,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"),
                 "Failed to set Position Control Mode for ID %d.", dxl_id);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"),
                "Succeeded to set Position Control Mode for ID %d.", dxl_id);
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"),
                 "Failed to enable torque for ID %d.", dxl_id);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"),
                "Succeeded to enable torque for ID %d.", dxl_id);
  }
}

// ----------------------------------------------------------------------
// Helper: set one Dynamixel into Current Control Mode (and enable torque)
// ----------------------------------------------------------------------
void setupDynamixelCurrent(uint8_t dxl_id)
{
  // Use Current Control Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    0,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"),
                 "Failed to set Current Control Mode for ID %d.", dxl_id);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"),
                "Succeeded to set Current Control Mode for ID %d.", dxl_id);
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"),
                 "Failed to enable torque for ID %d.", dxl_id);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"),
                "Succeeded to enable torque for ID %d.", dxl_id);
  }
}

// ----------------------------------------------------------------------
// Helper: set an initial Goal Position for a joint in Position Mode
// ----------------------------------------------------------------------
void setInitialPosition(uint8_t dxl_id, uint32_t goal_position)
{
  uint8_t dxl_error_local = 0;
  int dxl_comm_result_local = packetHandler->write4ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_GOAL_POSITION,
    goal_position,
    &dxl_error_local
  );

  if (dxl_comm_result_local != COMM_SUCCESS) {
    RCLCPP_ERROR(
      rclcpp::get_logger("read_write_node"),
      "Failed to set initial position %u for ID %d: %s",
      goal_position,
      dxl_id,
      packetHandler->getTxRxResult(dxl_comm_result_local)
    );
  } else if (dxl_error_local != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("read_write_node"),
      "DYNAMIXEL error when setting initial position %u for ID %d: %s",
      goal_position,
      dxl_id,
      packetHandler->getRxPacketError(dxl_error_local)
    );
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("read_write_node"),
      "Set initial position %u for ID %d",
      goal_position,
      dxl_id
    );
  }
}


// ----------------------------------------------------------------------
// main
// ----------------------------------------------------------------------
int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set the baudrate.");
  }

  // ----------------------------------------------------------------------
  // Activate all joints: 1,2,3,5 in Position mode, 4 in Current mode
  // ----------------------------------------------------------------------
  setupDynamixelPosition(JOINT1_ID);
  setupDynamixelPosition(JOINT2_ID);
  setupDynamixelPosition(JOINT3_ID);
  setupDynamixelCurrent(JOINT4_ID);   // actuator right before gripper
  setupDynamixelPosition(GRIPPER_ID);

  // ----------------------------------------------------------------------
  // Set initial positions for joints 1,2,3 to 2000 ticks
  // ----------------------------------------------------------------------
  uint32_t init_pos = 2000;
  setInitialPosition(JOINT1_ID, init_pos);
  setInitialPosition(JOINT2_ID, init_pos);
  setInitialPosition(JOINT3_ID, init_pos);

  rclcpp::init(argc, argv);

  auto readwritenode = std::make_shared<ReadWriteNode>();
  std::cout << "Before spin" << std::endl;
  rclcpp::spin(readwritenode);
  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL (all joints)
  packetHandler->write1ByteTxRx(
    portHandler,
    JOINT1_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );
  packetHandler->write1ByteTxRx(
    portHandler,
    JOINT2_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );
  packetHandler->write1ByteTxRx(
    portHandler,
    JOINT3_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );
  packetHandler->write1ByteTxRx(
    portHandler,
    JOINT4_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );
  packetHandler->write1ByteTxRx(
    portHandler,
    GRIPPER_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  return 0;
}
