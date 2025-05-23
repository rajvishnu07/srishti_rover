/* Swerve.ino This code was written by Vishnuraj A. Contains the velocity logic and control of Swerve drive on robots using ROS

Date uploaded: 20/05/2025
Date updated: 23/05/2025*/

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/quaternion.h>
#include <tf2_msgs/msg/tf_message.h>
#include <builtin_interfaces/msg/time.h>
#include <geometry_msgs/msg/transform_stamped.h>

// Robot dimensions
double wheelbase = 10;  // Distance between front and rear axles
double track_width = 1; // Distance between left and right wheels
double wheel_radius = 0.05; // Set appropriate value
float kp = 100;
unsigned long long time_offset = 0;
unsigned long currentMillis = 0;
double lastCmdVelReceived = 0;

// Velocity Parameters
double pwmFRZDriveReq = 0;
double pwmFRYDriveReq = 0;
double pwmFLZDriveReq = 0;
double pwmFLYDriveReq = 0;
double pwmRLZDriveReq = 0;
double pwmRLYDriveReq = 0;
double pwmRRZDriveReq = 0;
double pwmRRYDriveReq = 0;

#define LED_PIN 13
#define PI 3.14159265358979323846

// Pin definitions (adjust as per your hardware)
#define pwm1 0  // Front-right Z
#define dir1 1
#define pwm2 2  // Front-right Y
#define dir2 3
#define pwm3 4  // Front-left Z
#define dir3 5
#define pwm4 6  // Front-left Y
#define dir4 7
#define pwm5 8 // Rear-left Z
#define dir5 9
#define pwm6 10 // Rear-left Y
#define dir6 11
#define pwm7 12 // Rear-right Z
#define dir7 14
#define pwm8 15 // Rear-right Y
#define dir8 16

// Connection state management
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED,
  AGENT_RECONNECTING 
};

states state = WAITING_AGENT; // Initialize state

// microROS entities
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t cmd_vel_sub;
rcl_publisher_t tf_pub; // Transform publisher
geometry_msgs__msg__Twist twist_msg;
tf2_msgs__msg__TFMessage tf_message; // Transform message
geometry_msgs__msg__TransformStamped transform_stamped;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_timer_t timer;

// Define macros for connection management
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

// Error indicator function
void error_indicate() {
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

void set_pwm_values() {
  int pwmfrd = pwmFRZDriveReq;
  int pwmfrs = pwmFRYDriveReq;
  int pwmfld = pwmFLZDriveReq;
  int pwmfls = pwmFLYDriveReq;
  int pwmrrd = pwmRLZDriveReq;
  int pwmrrs = pwmRLYDriveReq;
  int pwmrld = pwmRRZDriveReq;
  int pwmrls = pwmRRYDriveReq;

  digitalWrite(dir1, pwmfrd >= 0 ? HIGH : LOW);
  analogWrite(pwm1, abs(pwmfrd));
  digitalWrite(dir2, pwmfrs >= 0 ? HIGH : LOW);
  analogWrite(pwm2, abs(pwmfrs));
  digitalWrite(dir3, pwmfld >= 0 ? HIGH : LOW);
  analogWrite(pwm3, abs(pwmfld));
  digitalWrite(dir4, pwmfls >= 0 ? HIGH : LOW);
  analogWrite(pwm4, abs(pwmfls));
  digitalWrite(dir5, pwmrrd >= 0 ? HIGH : LOW);
  analogWrite(pwm5, abs(pwmrrd));
  digitalWrite(dir6, pwmrrs >= 0 ? HIGH : LOW);
  analogWrite(pwm6, abs(pwmrrs));
  digitalWrite(dir7, pwmrld >= 0 ? HIGH : LOW);
  analogWrite(pwm7, abs(pwmrld));
  digitalWrite(dir8, pwmrls >= 0 ? HIGH : LOW);
  analogWrite(pwm8, abs(pwmrls));
}

void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * twist_msg = (const geometry_msgs__msg__Twist *)msgin;
  static double lastCmdVelReceived = 0;
  lastCmdVelReceived = millis() / 1000.0;

  double rx = wheelbase / 2.0;
  double ry = track_width / 2.0;
  double vx = twist_msg->linear.x;
  double vy = twist_msg->linear.y;
  double wz = twist_msg->angular.z;

  // Compute velocity components for each wheel (z: forward, y: sideways)
  double v_frz = vx + wz * rx; // Front-right wheel
  double v_fry = vy + wz * ry;
  double v_flz = vx - wz * rx; // Front-left wheel
  double v_fly = vy + wz * ry;
  double v_rlz = vx - wz * rx; // Rear-left wheel
  double v_rly = vy - wz * ry;
  double v_rrz = vx + wz * rx; // Rear-right wheel
  double v_rry = vy - wz * ry;

  // Convert speeds to wheel angular velocity (radians per second)
  double rps_frz = v_frz / (wheel_radius * 2 * PI);
  double rps_fry = v_fry / (wheel_radius * 2 * PI);
  double rps_flz = v_flz / (wheel_radius * 2 * PI);
  double rps_fly = v_fly / (wheel_radius * 2 * PI);
  double rps_rlz = v_rlz / (wheel_radius * 2 * PI);
  double rps_rly = v_rly / (wheel_radius * 2 * PI);
  double rps_rrz = v_rrz / (wheel_radius * 2 * PI);
  double rps_rry = v_rry / (wheel_radius * 2 * PI);

  // Map to PWM for driving motors
  pwmFRZDriveReq = rps_frz * kp;
  pwmFRYDriveReq = rps_fry * kp;
  pwmFLZDriveReq = rps_flz * kp;
  pwmFLYDriveReq = rps_fly * kp;
  pwmRLZDriveReq = rps_rlz * kp;
  pwmRLYDriveReq = rps_rly * kp;
  pwmRRZDriveReq = rps_rrz * kp;
  pwmRRYDriveReq = rps_rry * kp;

  set_pwm_values();
}

// Time synchronization function
void syncTime() {
    unsigned long now = millis();
    rmw_uros_sync_session(10);
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    time_offset = ros_time_ms - now;
}

struct timespec getTime() {
    struct timespec tp = { 0 };
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

// Timer callback
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    currentMillis = millis();
    
    // Update timeout to 1.0 seconds
    if ((millis() / 1000.0) - lastCmdVelReceived > 1.5) {  // Changed from 0.25 to 1.0
      pwmFRZDriveReq = 0;
      pwmFRYDriveReq = 0;
      pwmFLZDriveReq = 0;
      pwmFLYDriveReq = 0;
      pwmRLZDriveReq = 0;
      pwmRLYDriveReq = 0;
      pwmRRZDriveReq = 0;
      pwmRRYDriveReq = 0;
    }
    
    set_pwm_values();
  }
}

bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_motor_node", "", &support));
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));
  RCCHECK(rclc_publisher_init_default(
    &tf_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "tf"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));
  return true;
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  rcl_subscription_fini(&cmd_vel_sub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  pinMode(pwm1, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(pwm3, OUTPUT);
  pinMode(dir3, OUTPUT);
  pinMode(pwm4, OUTPUT);
  pinMode(dir4, OUTPUT);
  pinMode(pwm5, OUTPUT);
  pinMode(dir5, OUTPUT);
  pinMode(pwm6, OUTPUT);
  pinMode(dir6, OUTPUT);
  pinMode(pwm7, OUTPUT);
  pinMode(dir7, OUTPUT);
  pinMode(pwm8, OUTPUT);
  pinMode(dir8, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  analogWrite(pwm1, 0);
  analogWrite(pwm2, 0);
  analogWrite(pwm3, 0);
  analogWrite(pwm4, 0);
  analogWrite(pwm5, 0);
  analogWrite(pwm6, 0);
  analogWrite(pwm7, 0);
  analogWrite(pwm8, 0);
  
  set_microros_transports();
  state = WAITING_AGENT;
}

void loop() {
  static unsigned long lastCmdVelReceived = 0;
  static unsigned long lastSyncTime = 0;
  static int reconnectAttempts = 0;
  const int MAX_RECONNECT_ATTEMPTS = 5;

  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
        error_indicate();
      }
      reconnectAttempts = 0;
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_RECONNECTING;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      }
      break;
    case AGENT_RECONNECTING:
      EXECUTE_EVERY_N_MS(200, {
        if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) {
          state = AGENT_CONNECTED;
          reconnectAttempts = 0;
        } else {
          reconnectAttempts++;
          if (reconnectAttempts >= MAX_RECONNECT_ATTEMPTS) {
            destroy_entities();
            state = WAITING_AGENT;
            error_indicate();
          }
        }
      });
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  if ((millis() / 1000.0) - lastCmdVelReceived > 1.5) {
    pwmFRZDriveReq = 0;
    pwmFRYDriveReq = 0;
    pwmFLZDriveReq = 0;
    pwmFLYDriveReq = 0;
    pwmRLZDriveReq = 0;
    pwmRLYDriveReq = 0;
    pwmRRZDriveReq = 0;
    pwmRRYDriveReq = 0;
    set_pwm_values();
  }
}
