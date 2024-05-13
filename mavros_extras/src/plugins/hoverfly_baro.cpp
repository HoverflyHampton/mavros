/*
 * Copyright 2013-2017,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief IMU and attitude data parser plugin
 * @file imu.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <tf2_eigen/tf2_eigen.h>

#include <cmath>
#include <string>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

//! Gauss to Tesla coeff
static constexpr double GAUSS_TO_TESLA = 1.0e-4;
//! millTesla to Tesla coeff
static constexpr double MILLIT_TO_TESLA = 1000.0;
//! millRad/Sec to Rad/Sec coeff
static constexpr double MILLIRS_TO_RADSEC = 1.0e-3;
//! millG to m/s**2 coeff
static constexpr double MILLIG_TO_MS2 = 9.80665 / 1000.0;
//! millm/s**2 to m/s**2 coeff
static constexpr double MILLIMS2_TO_MS2 = 1.0e-3;
//! millBar to Pascal coeff
static constexpr double MILLIBAR_TO_PASCAL = 1.0e2;
//! Radians to degrees
static constexpr double RAD_TO_DEG = 180.0 / M_PI;


/**
 * @brief IMU and attitude data publication plugin
 * @plugin imu
 */
class HoverflyBaroPlugin : public plugin::Plugin
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit HoverflyBaroPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "hoverfly_baro")
  {
    auto sensor_qos = rclcpp::SensorDataQoS();

    baro_temp_pub1 = node->create_publisher<sensor_msgs::msg::Temperature>(
      "~/temperature_baro_1",
      sensor_qos);
    static_press_pub1 = node->create_publisher<sensor_msgs::msg::FluidPressure>(
      "~/static_pressure_1",
      sensor_qos);
    diff_press_pub1 = node->create_publisher<sensor_msgs::msg::FluidPressure>(
      "~/diff_pressure_1",
      sensor_qos);

    baro_temp_pub2 = node->create_publisher<sensor_msgs::msg::Temperature>(
      "~/temperature_baro_2",
      sensor_qos);
    static_press_pub2 = node->create_publisher<sensor_msgs::msg::FluidPressure>(
      "~/static_pressure_2",
      sensor_qos);
    diff_press_pub2 = node->create_publisher<sensor_msgs::msg::FluidPressure>(
      "~/diff_pressure_2",
      sensor_qos);

    baro_temp_pub3 = node->create_publisher<sensor_msgs::msg::Temperature>(
      "~/temperature_baro_3",
      sensor_qos);
    static_press_pub3 = node->create_publisher<sensor_msgs::msg::FluidPressure>(
      "~/static_pressure_3",
      sensor_qos);
    diff_press_pub3 = node->create_publisher<sensor_msgs::msg::FluidPressure>(
      "~/diff_pressure_3",
      sensor_qos);

  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&HoverflyBaroPlugin::handle_scaled_pressure1),
      make_handler(&HoverflyBaroPlugin::handle_scaled_pressure2),
      make_handler(&HoverflyBaroPlugin::handle_scaled_pressure3)
    };
  }

private:
  std::string frame_id;

  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr baro_temp_pub1;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr static_press_pub1;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr diff_press_pub1;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr baro_temp_pub2;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr static_press_pub2;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr diff_press_pub2;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr baro_temp_pub3;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr static_press_pub3;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr diff_press_pub3;

  /* -*- helpers -*- */

  /**
   * @brief Handle SCALED_PRESSURE MAVlink message.
   * Message specification: https://mavlink.io/en/messages/common.html/#SCALED_PRESSURE
   * @param msg		Received Mavlink msg
   * @param press		SCALED_PRESSURE msg
   */
  void handle_scaled_pressure1(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::SCALED_PRESSURE & press,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto header = uas->synchronized_header(frame_id, press.time_boot_ms);

    auto temp_msg = sensor_msgs::msg::Temperature();
    temp_msg.header = header;
    temp_msg.temperature = press.temperature / 100.0;
    baro_temp_pub1->publish(temp_msg);

    auto static_pressure_msg = sensor_msgs::msg::FluidPressure();
    static_pressure_msg.header = header;
    static_pressure_msg.fluid_pressure = press.press_abs * 100.0;
    static_press_pub1->publish(static_pressure_msg);

    auto differential_pressure_msg = sensor_msgs::msg::FluidPressure();
    differential_pressure_msg.header = header;
    differential_pressure_msg.fluid_pressure = press.press_diff * 100.0;
    diff_press_pub1->publish(differential_pressure_msg);
  }

  /**
   * @brief Handle SCALED_PRESSURE MAVlink message.
   * Message specification: https://mavlink.io/en/messages/common.html/#SCALED_PRESSURE
   * @param msg		Received Mavlink msg
   * @param press		SCALED_PRESSURE msg
   */
  void handle_scaled_pressure2(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::SCALED_PRESSURE2 & press,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto header = uas->synchronized_header(frame_id, press.time_boot_ms);

    auto temp_msg = sensor_msgs::msg::Temperature();
    temp_msg.header = header;
    temp_msg.temperature = press.temperature / 100.0;
    baro_temp_pub2->publish(temp_msg);

    auto static_pressure_msg = sensor_msgs::msg::FluidPressure();
    static_pressure_msg.header = header;
    static_pressure_msg.fluid_pressure = press.press_abs * 100.0;
    static_press_pub2->publish(static_pressure_msg);

    auto differential_pressure_msg = sensor_msgs::msg::FluidPressure();
    differential_pressure_msg.header = header;
    differential_pressure_msg.fluid_pressure = press.press_diff * 100.0;
    diff_press_pub2->publish(differential_pressure_msg);
  }

  /**
   * @brief Handle SCALED_PRESSURE MAVlink message.
   * Message specification: https://mavlink.io/en/messages/common.html/#SCALED_PRESSURE
   * @param msg		Received Mavlink msg
   * @param press		SCALED_PRESSURE msg
   */
  void handle_scaled_pressure3(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::SCALED_PRESSURE3 & press,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto header = uas->synchronized_header(frame_id, press.time_boot_ms);

    auto temp_msg = sensor_msgs::msg::Temperature();
    temp_msg.header = header;
    temp_msg.temperature = press.temperature / 100.0;
    baro_temp_pub3->publish(temp_msg);

    auto static_pressure_msg = sensor_msgs::msg::FluidPressure();
    static_pressure_msg.header = header;
    static_pressure_msg.fluid_pressure = press.press_abs * 100.0;
    static_press_pub3->publish(static_pressure_msg);

    auto differential_pressure_msg = sensor_msgs::msg::FluidPressure();
    differential_pressure_msg.header = header;
    differential_pressure_msg.fluid_pressure = press.press_diff * 100.0;
    diff_press_pub3->publish(differential_pressure_msg);
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::HoverflyBaroPlugin)
