#include <M5Unified.h>
#include <Avatar.h>

#include "micro_ros_arduino_simpler/simpler_base.h"
#include <std_msgs/msg/int32.h>
#include "ip.hpp"

// Avatar --------------------------------------------------------------------
using namespace m5avatar;
Avatar avatar;

const Expression expressions[] = {
  Expression::Angry,
  Expression::Sleepy,
  Expression::Happy,
  Expression::Sad,
  Expression::Doubt,
  Expression::Neutral
};
const int expressionsSize = sizeof(expressions) / sizeof(Expression);
int old_idx = 0;
int idx = 0;
// ---------------------------------------------------------------------------

// ==========================================================================
extern rclc_executor_t executor;
extern rclc_support_t support;
extern rcl_allocator_t allocator;
extern rcl_node_t node;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg_pub_int32;
std_msgs__msg__Int32 msg_sub_int32;

rcl_subscription_t subscription;
rcl_timer_t timer;

// subscription callback ====================================================
void int32_callback(const void *msgin)
{
  const std_msgs__msg__Int32 *_msg = (const std_msgs__msg__Int32 *)msgin;
  int _idx = _msg->data;
  if (_idx < 0 || _idx >= expressionsSize)
  {
    return;
  }
  idx = _idx;
}

// timer callback (10ms) ====================================================
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)last_call_time;
  (void)timer;

  msg_pub_int32.data = old_idx;
  rcl_publish(&publisher, &msg_pub_int32, NULL);
}
// ==========================================================================


void setup()
{
  auto cfg = M5.config();
  cfg.clear_display = false;
  cfg.output_power  = false;
  cfg.internal_imu  = false;
  cfg.internal_rtc  = false;
  cfg.internal_spk  = false;
  cfg.internal_mic  = false;
  cfg.external_imu  = false;
  cfg.external_rtc  = false;
  M5.begin(cfg);
  setup_microros_wifi("microros_node", "", 2, SSID, PASS, IP_ADDRESS, PORT);
  rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "face_feedback");

  rclc_create_timer_and_add(&timer, 10, timer_callback);
  rclc_create_subscription_and_add(&subscription, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), &msg_sub_int32, &int32_callback, "face_command");

  M5.Lcd.setBrightness(60);
  M5.Lcd.clear();
  avatar.init();
  printf("avatar init\n");
}

void loop()
{
  M5.update();
  if (idx != old_idx)
  {
    old_idx = idx;
    avatar.setExpression(expressions[idx]);
  }
  rclc_delay(10);
}
