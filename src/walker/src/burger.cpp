/**
 * @file burger.cpp
 * @author Kshitij Aggarwal
 * @version 0.1
 */

/**
 * @copyright Copyright 2024 Kshitij Aggarwal

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

/**
 * @brief Base class for the Robot's state using the State Design Pattern.
 * Provides an interface for all specific state implementations.
 */
class ObstacleAvoider {
 public:
  virtual ~ObstacleAvoider() = default;

  /**
   * @brief Updates the state based on LaserScan message data.
   * @param msg Shared pointer to LaserScan message.
   */
  virtual void update(const sensor_msgs::msg::LaserScan::SharedPtr msg) = 0;

  /**
   * @brief Returns the angular velocity for the current state.
   * @return Angular velocity.
   */
  virtual double getAngularVel() const = 0;

  /**
   * @brief Returns the linear velocity for the current state.
   * @return Linear velocity.
   */
  virtual double getLinearVel() const = 0;

  ///Initialising Linear and angular velocity variables
  // with default value=0
  double linear_vel = 0.0;
  double angular_vel = 0.0;

  /**
   * @brief Determines the next state based on LaserScan data.
   * @param msg Input LaserScan message used to check for obstacles.
   * @return Shared pointer to the next state.
   */
  virtual std::shared_ptr<ObstacleAvoider> getNextState(
      const sensor_msgs::msg::LaserScan::SharedPtr msg) = 0;
};

/**
 * @brief State representing the robot's rotation behavior.
 *
 * The robot transitions to another state if it detects an obstacle within a threshold distance.
 */
class RotatingState : public ObstacleAvoider {
 public:
  /**
   * @brief Constructor to initialize the rotation direction.
   */
  RotatingState() {
    angular_vel = rotation_swap ? -0.5 : 0.5;  ///< Clockwise or counterclockwise rotation.
  }

  /**
   * @brief Updates velocities based on LaserScan data for rotation behavior.
   * @param msg Shared pointer to LaserScan message.
   */
  void update(const sensor_msgs::msg::LaserScan::SharedPtr msg) override {
    // Adjust velocities based on the distance to the nearest object.
    if (msg->ranges[0] < 0.7) {
      linear_vel = 0.0;
    } else {
      linear_vel = 0.2;
      angular_vel = 0.2;
    }
  }

  /**
   * @brief Determines the next state after completing rotation.
   * @param msg LaserScan message data.
   * @return Shared pointer to the next state.
   */
  std::shared_ptr<ObstacleAvoider> getNextState(
      const sensor_msgs::msg::LaserScan::SharedPtr msg) override;

  /**
   * @brief Returns the current linear and angular velocities.
   */
  double getLinearVel() const override { return linear_vel; }
  double getAngularVel() const override { return angular_vel; }

 private:
  static bool rotation_swap;  ///< Flag to alternate rotation directions.
};

// Initialize the rotation direction toggle flag.
bool RotatingState::rotation_swap = true;

/**
 * @brief State representing the robot's forward movement behavior.
 *
 * The robot moves forward until it detects an obstacle.
 */
class ForwardState : public ObstacleAvoider {
 public:
  /**
   * @brief Updates velocities for forward movement.
   */
  void update(const sensor_msgs::msg::LaserScan::SharedPtr /*msg*/) override {
    linear_vel = 0.2;
    angular_vel = 0.3;
  }

  /**
   * @brief Determines the next state when an obstacle is detected.
   * @param msg LaserScan message used to check for obstacles.
   * @return Shared pointer to the next state.
   */
  std::shared_ptr<ObstacleAvoider> getNextState(
      const sensor_msgs::msg::LaserScan::SharedPtr msg) override {
    if (msg->ranges[0] < 0.6) {
      return std::make_shared<RotatingState>();
    }
    return nullptr;
  }

  double getLinearVel() const override { return linear_vel; }
  double getAngularVel() const override { return angular_vel; }
};

// Implementation of the next state logic for RotatingState.
std::shared_ptr<ObstacleAvoider> RotatingState::getNextState(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (msg->ranges[0] >= 0.7) {
    rotation_swap = !rotation_swap;
    return std::make_shared<ForwardState>();  ///< Switch to forward movement.
  }
  return nullptr;
}

/**
 * @brief Class to manage transitions between robot states.
 */
class Robot {
 public:
  /**
   * @brief Constructor initializing the robot in the forward movement state.
   */
  Robot() : current_state_(std::make_shared<ForwardState>()) {}

  /**
   * @brief Updates the current state based on LaserScan data.
   */
  void update(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    current_state_->update(msg);

    auto next_state = current_state_->getNextState(msg);
    if (next_state) {
      current_state_ = next_state;  ///< Transition to the next state.
    }
  }

  /**
   * @brief Returns the linear and angular velocities from the current state.
   */
  double getLinearVel() const { return current_state_->getLinearVel(); }
  double getAngularVel() const { return current_state_->getAngularVel(); }

 private:
  std::shared_ptr<ObstacleAvoider> current_state_;  ///< Pointer to the current state.
};

/**
 * @brief ROS 2 Node managing velocity publishing and LaserScan subscription.
 */
class AvoiderNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor setting up publishers and subscribers.
   */
  AvoiderNode() : Node("roomba_node") {
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&AvoiderNode::laser_callback, this, std::placeholders::_1));
    state_ = std::make_shared<Robot>();
  }

 private:
  /**
   * @brief Callback to process LaserScan data and update velocities.
   */
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    state_->update(msg);

    twist_msg_.linear.x = state_->getLinearVel();
    twist_msg_.angular.z = state_->getAngularVel();
    vel_pub_->publish(twist_msg_);
  }

  std::shared_ptr<Robot> state_;  ///< Instance of the Robot class.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;  ///< Velocity publisher.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;  ///< LaserScan subscriber.
  geometry_msgs::msg::Twist twist_msg_;  ///< Velocity message to publish.
};

/**
 * @brief Main function to initialize the ROS 2 node and start its event loop.
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AvoiderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
