#include "microros_app_includes.h"
#if APP_ENABLE_UROS

#include <string.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

volatile uros_state_t g_uros_state      = UROS_STATE_WAIT_AGENT;
volatile uint32_t     g_uros_reconnects = 0;
volatile uint32_t     g_uros_pub_errors = 0;
volatile uint32_t     g_cmd_count       = 0;

static rclc_support_t    s_support;
static rcl_node_t        s_node;
static rclc_executor_t   s_exec;
static rcl_publisher_t   s_pub_odom;
static rcl_publisher_t   s_pub_imu;
static rcl_publisher_t   s_pub_wheel;
static rcl_subscription_t s_sub_cmd;
static bool              s_inited;

static nav_msgs__msg__Odometry              s_msg_odom;
static sensor_msgs__msg__Imu                s_msg_imu;
static std_msgs__msg__Float32MultiArray     s_msg_wheel;
static geometry_msgs__msg__Twist            s_msg_cmd;
static float s_wheel_data[2];

static char s_frame_odom[] = UROS_FRAME_ODOM;
static char s_frame_base[] = UROS_FRAME_BASE;
static char s_frame_imu[]  = UROS_FRAME_IMU;

/* 里程计状态：只在控制节拍里更新，发布时只读 */
static float s_pose_x, s_pose_y, s_pose_th;

/* cmd_vel 最新值 */
static float    s_cmd_v, s_cmd_w;
static uint32_t s_cmd_ms;
static bool     s_cmd_seen;

static uint32_t s_next_odom_ms, s_next_imu_ms, s_next_wheel_ms, s_next_ping_ms;
static uint32_t s_last_odom_ms;

/* ---------------- 工具 ---------------- */

static void set_stamp(builtin_interfaces__msg__Time *stamp)
{
    int64_t ms = rmw_uros_epoch_millis();
    if (ms <= 0) {
        ms = (int64_t)HAL_GetTick();   /* 未对时：用本地时基，至少单调 */
    }
    stamp->sec     = (int32_t)(ms / 1000);
    stamp->nanosec = (uint32_t)((ms % 1000) * 1000000);
}

static void set_str(rosidl_runtime_c__String *s, char *lit)
{
    s->data     = lit;
    s->size     = (size_t)strlen(lit);
    s->capacity = s->size + 1U;
}

static bool due(uint32_t *next, uint32_t period_ms)
{
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - *next) < 0) {
        return false;
    }
    *next = now + period_ms;
    return true;
}

/* ---------------- 实体 ---------------- */

static bool entities_create(void)
{
    rcl_allocator_t alloc = rcl_get_default_allocator();

    if (rclc_support_init(&s_support, 0, NULL, &alloc) != RCL_RET_OK) {
        return false;
    }
    if (rclc_node_init_default(&s_node, UROS_NODE_NAME, "", &s_support) != RCL_RET_OK) {
        return false;
    }
    if (rclc_publisher_init_default(&s_pub_odom, &s_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), UROS_TOPIC_ODOM) != RCL_RET_OK) {
        return false;
    }
    if (rclc_publisher_init_default(&s_pub_imu, &s_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), UROS_TOPIC_IMU) != RCL_RET_OK) {
        return false;
    }
    if (rclc_publisher_init_default(&s_pub_wheel, &s_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), UROS_TOPIC_WHEEL) != RCL_RET_OK) {
        return false;
    }
    if (rclc_subscription_init_default(&s_sub_cmd, &s_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), UROS_TOPIC_CMD_VEL) != RCL_RET_OK) {
        return false;
    }
    if (rclc_executor_init(&s_exec, &s_support.context, 1, &alloc) != RCL_RET_OK) {
        return false;
    }
    return true;
}

static void entities_destroy(void)
{
    (void)rclc_executor_fini(&s_exec);
    (void)rcl_subscription_fini(&s_sub_cmd, &s_node);
    (void)rcl_publisher_fini(&s_pub_wheel, &s_node);
    (void)rcl_publisher_fini(&s_pub_imu, &s_node);
    (void)rcl_publisher_fini(&s_pub_odom, &s_node);
    (void)rcl_node_fini(&s_node);
    (void)rclc_support_fini(&s_support);
}

static void cmd_vel_cb(const void *msgin)
{
    const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;

    s_cmd_v    = (float)m->linear.x;
    s_cmd_w    = (float)m->angular.z;
    s_cmd_ms   = HAL_GetTick();
    s_cmd_seen = true;
    g_cmd_count++;
}

/* ---------------- 发布 ---------------- */

static void publish_odom(void)
{
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - s_last_odom_ms) / 1000.0f;
    s_last_odom_ms = now;
    if (dt > 0.2f) {
        dt = 0.2f;                      /* 长时间没发布别一次补太多 */
    }

    odom_t od;
    odom_get(&od);
    s_pose_x  += od.linear_mps * cosf(s_pose_th) * dt;
    s_pose_y  += od.linear_mps * sinf(s_pose_th) * dt;
    s_pose_th += (od.yaw_rate_dps * 0.0174533f) * dt;

    set_stamp(&s_msg_odom.header.stamp);
    s_msg_odom.pose.pose.position.x = (double)s_pose_x;
    s_msg_odom.pose.pose.position.y = (double)s_pose_y;
    s_msg_odom.pose.pose.orientation.z = sinf(s_pose_th * 0.5f);
    s_msg_odom.pose.pose.orientation.w = cosf(s_pose_th * 0.5f);

    s_msg_odom.twist.twist.linear.x  = (double)od.linear_mps;
    s_msg_odom.twist.twist.angular.z = (double)(od.yaw_rate_dps * 0.0174533f);

    if (rcl_publish(&s_pub_odom, &s_msg_odom, NULL) != RCL_RET_OK) {
        g_uros_pub_errors++;
    }
}

static void publish_wheel(void)
{
    odom_t od;
    odom_get(&od);
    s_wheel_data[0] = g_wheel_l.speed_mps;
    s_wheel_data[1] = g_wheel_r.speed_mps;
    s_msg_wheel.data.size = 2U;

    if (rcl_publish(&s_pub_wheel, &s_msg_wheel, NULL) != RCL_RET_OK) {
        g_uros_pub_errors++;
    }
}

static void publish_imu(void)
{
    dev_imu_sample_t sample;

    if (!dev_imu_read(&sample)) {
        return;
    }
    set_stamp(&s_msg_imu.header.stamp);
    s_msg_imu.angular_velocity.x = (double)sample.gyro_rps[0];
    s_msg_imu.angular_velocity.y = (double)sample.gyro_rps[1];
    s_msg_imu.angular_velocity.z = (double)sample.gyro_rps[2];
    s_msg_imu.linear_acceleration.x = (double)sample.acc_mps2[0];
    s_msg_imu.linear_acceleration.y = (double)sample.acc_mps2[1];
    s_msg_imu.linear_acceleration.z = (double)sample.acc_mps2[2];

    if (rcl_publish(&s_pub_imu, &s_msg_imu, NULL) != RCL_RET_OK) {
        g_uros_pub_errors++;
    }
}

/* ---------------- 对外 ---------------- */

bool uros_app_get_cmd(float *v, float *w)
{
    if (!s_cmd_seen || (HAL_GetTick() - s_cmd_ms) > APP_CMD_TIMEOUT_MS) {
        return false;
    }
    *v = s_cmd_v;
    *w = s_cmd_w;
    return true;
}

static void msg_setup(void)
{
    nav_msgs__msg__Odometry__init(&s_msg_odom);
    sensor_msgs__msg__Imu__init(&s_msg_imu);
    std_msgs__msg__Float32MultiArray__init(&s_msg_wheel);
    geometry_msgs__msg__Twist__init(&s_msg_cmd);

    set_str(&s_msg_odom.header.frame_id, s_frame_odom);
    set_str(&s_msg_odom.child_frame_id, s_frame_base);
    set_str(&s_msg_imu.header.frame_id, s_frame_imu);

    /* REP 145：orientation_covariance[0] = -1 表示「不提供姿态」 */
    s_msg_imu.orientation_covariance[0] = -1.0;

    s_msg_odom.pose.covariance[0]  = APP_COV_ODOM_POS;
    s_msg_odom.pose.covariance[7]  = APP_COV_ODOM_POS;
    s_msg_odom.pose.covariance[35] = APP_COV_ODOM_YAW;
    s_msg_odom.twist.covariance[0]  = APP_COV_ODOM_VEL;
    s_msg_odom.twist.covariance[7]  = APP_COV_ODOM_VEL;
    s_msg_odom.twist.covariance[35] = APP_COV_ODOM_W;

    s_msg_imu.angular_velocity_covariance[0] = APP_COV_IMU_GYRO;
    s_msg_imu.angular_velocity_covariance[4] = APP_COV_IMU_GYRO;
    s_msg_imu.angular_velocity_covariance[8] = APP_COV_IMU_GYRO;
    s_msg_imu.linear_acceleration_covariance[0] = APP_COV_IMU_ACC;
    s_msg_imu.linear_acceleration_covariance[4] = APP_COV_IMU_ACC;
    s_msg_imu.linear_acceleration_covariance[8] = APP_COV_IMU_ACC;

    s_msg_wheel.data.data     = s_wheel_data;
    s_msg_wheel.data.size     = 0U;
    s_msg_wheel.data.capacity = 2U;
}

static void params_setup(void)
{
    uint32_t now = HAL_GetTick();
    s_next_odom_ms  = now;
    s_next_imu_ms   = now;
    s_next_wheel_ms = now;
    s_next_ping_ms  = now;
    s_last_odom_ms  = now;
}

static bool try_connect(void)
{
    if (!uros_transport_install()) {
        return false;
    }
    if (!entities_create()) {
        entities_destroy();
        return false;
    }
    if (rclc_executor_add_subscription(&s_exec, &s_sub_cmd, &s_msg_cmd,
                                       cmd_vel_cb, ON_NEW_DATA) != RCL_RET_OK) {
        entities_destroy();
        return false;
    }
    (void)rmw_uros_sync_session(1000);
    msg_setup();
    params_setup();
    return true;
}

void uros_app_loop(void)
{
    if (g_uros_state == UROS_STATE_WAIT_AGENT) {
        if (try_connect()) {
            g_uros_state = UROS_STATE_CONNECTED;
            g_uros_reconnects++;
            s_inited = true;
        }
        return;
    }

    (void)rclc_executor_spin_some(&s_exec, RCL_MS_TO_NS(2));

    if (due(&s_next_odom_ms, 1000U / UROS_ODOM_HZ)) {
        publish_odom();
    }
    if (due(&s_next_imu_ms, 1000U / UROS_IMU_HZ)) {
        publish_imu();
    }
    if (due(&s_next_wheel_ms, 1000U / UROS_WHEEL_HZ)) {
        publish_wheel();
    }

    /* 每秒 ping 一次，早期发现链路断，不用等发布失败 */
    if (due(&s_next_ping_ms, 1000U)) {
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
            if (s_inited) {
                entities_destroy();
                s_inited = false;
            }
            g_uros_state = UROS_STATE_WAIT_AGENT;
        }
    }

    if (g_uros_pub_errors > 0U && (g_uros_pub_errors % 50U) == 0U) {
        if (s_inited) {
            entities_destroy();
            s_inited = false;
        }
        g_uros_state = UROS_STATE_WAIT_AGENT;
    }
}

/* ---- link 抽象层的 micro-ROS 实现 ---- */
void link_loop(void)
{
    uros_app_loop();
}

bool link_get_cmd(float *v, float *w)
{
    return uros_app_get_cmd(v, w);
}

bool link_ready(void)
{
    return (g_uros_state == UROS_STATE_CONNECTED);
}

bool link_get_brake(void)
{
    return false;   /* micro-ROS 路径暂无制动指令，保持滑行语义 */
}

#endif /* APP_ENABLE_UROS */
