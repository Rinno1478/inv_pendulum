#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_BNO08x.h>
#include <AccelStepper.h>
#include <algorithm>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// #include <sensor_msgs/msg/imu.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_GIGA) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL) && !defined(ARDUINO_UNOR4_WIFI) && !defined(ARDUINO_OPTA)
#error This example is only available for Arduino Portenta, Arduino Giga R1, Arduino Nano RP2040 Connect, ESP32 Dev module, Wio Terminal, Arduino Uno R4 WiFi and Arduino OPTA WiFi 
#endif

#if (MICRO_ROS_TRANSPORT_ARDUINO_WIFI==1)
// Wifi関連
char ssid[] = "Buffalo-G-2E78";
char psk[] = "e477ttud6vekh";
IPAddress agent_ip(192, 168, 11, 15);
size_t agent_port = 8888;
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define PIN_A1 33
#define PIN_B1 25
#define PIN_C1 26
#define PIN_D1 27
#define PIN_A2 2
#define PIN_B2 0
#define PIN_C2 4
#define PIN_D2 16
#define PIN_START_BUTTON 34
#define PIN_START_LED 32
#define BNO08X_CS 5
#define BNO08X_INT 21
#define BNO08X_RESET 22

// #define CALIBRATION_ITERATIONS 1000 // ジャイロセンサのキャリブレーション回数
// #define CONTROL_CYCLE_MS 10 // 制御周期(ms)
#define SPR 400 // steps per revolution (ステッピングモータの1回転あたりのステップ数)
#define WHEEL_RADIUS 0.056 // 車輪の半径(m)
#define WHEEL_DISTANCE 0.110 // 車輪間の距離(m)

TaskHandle_t thp[2];

rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t pub_timer;
rclc_support_t support;
rclc_executor_t executor_pubsub;

// rcl_publisher_t pub_imu;
rcl_publisher_t pub_odom;
rcl_subscription_t sub_cmd_vel;

// sensor_msgs__msg__Imu imu_msg;
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

AccelStepper stepper1(AccelStepper::HALF4WIRE, PIN_A1, PIN_B1, PIN_C1, PIN_D1);
AccelStepper stepper2(AccelStepper::HALF4WIRE, PIN_A2, PIN_B2, PIN_C2, PIN_D2);

enum ANGLE {
    ROLL = 0,
    PITCH = 1,
    YAW = 2
};

struct Pos {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Vel {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct AngluarVel {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Acc {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Quat {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 1.0f;
};

struct Euler {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Pose {
    Pos position = Pos();
    Quat orientation = Quat();
};

struct IMU {
    Quat orientation = Quat();
    AngluarVel angular_velocity = AngluarVel();
    Acc linear_acceleration = Acc();
};

struct Twist {
    Vel linear = Vel();
    AngluarVel angular = AngluarVel();
};

struct Odometry {
    Pose pose = Pose();
    Twist twist = Twist();
};

IMU robot_imu;
Odometry robot_odom;
Euler robot_imu_euler;
Twist robot_cmd_vel;

float wheel_step_speed[2] = {0.0f, 0.0f};   

bool is_button_pressed = false;

float angle_yaw = 0.0f;
float last_imu_yaw = 0.0f;

float K[3] = { 1066.09 , 166.259 , 12.9099 }; // lqrで求めたゲイン

void quaternionToEuler(Quat q, Euler *euler) {
    euler->x = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
    euler->y = asin(2 * (q.w * q.y - q.z * q.x));
    euler->z = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.println("Could not enable accelerometer");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable gyroscope");
  }
//   if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
//     Serial.println("Could not enable magnetic field calibrated");
//   }
//   if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
//     Serial.println("Could not enable linear acceleration");
//   }
//   if (!bno08x.enableReport(SH2_GRAVITY)) {
//     Serial.println("Could not enable gravity vector");
//   }
//   if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
//     Serial.println("Could not enable rotation vector");
//   }
//   if (!bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR)) {
//     Serial.println("Could not enable geomagnetic rotation vector");
//   }
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game rotation vector");
  }
//   if (!bno08x.enableReport(SH2_STEP_COUNTER)) {
//     Serial.println("Could not enable step counter");
//   }
//   if (!bno08x.enableReport(SH2_STABILITY_CLASSIFIER)) {
//     Serial.println("Could not enable stability classifier");
//   }
//   if (!bno08x.enableReport(SH2_RAW_ACCELEROMETER)) {
//     Serial.println("Could not enable raw accelerometer");
//   }
//   if (!bno08x.enableReport(SH2_RAW_GYROSCOPE)) {
//     Serial.println("Could not enable raw gyroscope");
//   }
//   if (!bno08x.enableReport(SH2_RAW_MAGNETOMETER)) {
//     Serial.println("Could not enable raw magnetometer");
//   }
//   if (!bno08x.enableReport(SH2_SHAKE_DETECTOR)) {
//     Serial.println("Could not enable shake detector");
//   }
//   if (!bno08x.enableReport(SH2_PERSONAL_ACTIVITY_CLASSIFIER)) {
//     Serial.println("Could not enable personal activity classifier");
//   }
}

// Error handle loop
void error_loop() {
	while(true) {
		delay(100);
	}
}

unsigned long last_odom_update_time = 0;
void robot_odom_update(Odometry *odom) {
    float v =  (wheel_step_speed[0] + wheel_step_speed[1]) * PI / SPR * WHEEL_RADIUS;   // m/s
    float cos_z = cosf(robot_imu_euler.z);
    float sin_z = sinf(robot_imu_euler.z);
    odom->twist.linear.x = v * cos_z;
    odom->twist.linear.y = v * sin_z;
    odom->twist.angular.x = robot_imu.angular_velocity.x * cos_z - robot_imu.angular_velocity.y * sin_z;
    odom->twist.angular.y = robot_imu.angular_velocity.x * sin_z + robot_imu.angular_velocity.y * cos_z;
    odom->twist.angular.z = robot_imu.angular_velocity.z;

    unsigned long micro_dt = (micros() - last_odom_update_time);
    odom->pose.position.x += v * cos_z * (0.000001 * micro_dt);
    odom->pose.position.y += v * sin_z * (0.000001 * micro_dt);
    odom->pose.orientation = robot_imu.orientation;
    last_odom_update_time = micros();
}

void read_imu_data() {
    if (!bno08x.getSensorEvent(&sensorValue)) {
        return;
    }

    switch (sensorValue.sensorId) {
        case SH2_ACCELEROMETER:
            robot_imu.linear_acceleration.x = sensorValue.un.accelerometer.x;
            robot_imu.linear_acceleration.y = sensorValue.un.accelerometer.y;
            robot_imu.linear_acceleration.z = sensorValue.un.accelerometer.z;
            break;
        case SH2_GYROSCOPE_CALIBRATED:
            robot_imu.angular_velocity.x = sensorValue.un.gyroscope.x;
            robot_imu.angular_velocity.y = sensorValue.un.gyroscope.y;
            robot_imu.angular_velocity.z = sensorValue.un.gyroscope.z;
            break;
        case SH2_GAME_ROTATION_VECTOR:
            robot_imu.orientation.x = sensorValue.un.rotationVector.i;
            robot_imu.orientation.y = sensorValue.un.rotationVector.j;
            robot_imu.orientation.z = sensorValue.un.rotationVector.k;
            robot_imu.orientation.w = sensorValue.un.rotationVector.real;
            break;
    }
}

void update_angle_yaw(float imu_yaw) {
    if (imu_yaw - last_imu_yaw > PI) {
        angle_yaw += -2 * PI + (imu_yaw - last_imu_yaw);
    } else if (imu_yaw - last_imu_yaw < -PI) {
        angle_yaw += 2 * PI + (imu_yaw - last_imu_yaw);
    } else {
        angle_yaw += imu_yaw - last_imu_yaw;
    }
    last_imu_yaw = imu_yaw;
}

unsigned long last_control_update_time = 0;
void control() {
    read_imu_data();
    const ANGLE angle_type = ROLL;
    const ANGLE omega_type = YAW;
    const float balance_angle = 0.0f; // バランスを取るための角度
    // ロール、ピッチ、ヨー角の取得
    quaternionToEuler(robot_imu.orientation, &robot_imu_euler);
    update_angle_yaw(robot_imu_euler.z);
    float angle = 0.0f;
    float omega = 0.0f;
    if (angle_type == ROLL) {
        angle = robot_imu_euler.x;
        omega = robot_imu.angular_velocity.x;
    } else if (angle_type == PITCH) {
        angle = robot_imu_euler.y;
        omega = robot_imu.angular_velocity.y;
    } else if (angle_type == YAW) {
        angle = robot_imu_euler.z;
        omega = robot_imu.angular_velocity.z;
    }

    float rotation_omega = 0.0f;
    if (omega_type == ROLL) {
        rotation_omega = robot_imu.angular_velocity.x;
    } else if (omega_type == PITCH) {
        rotation_omega = robot_imu.angular_velocity.y;
    } else if (omega_type == YAW) {
        rotation_omega = robot_imu.angular_velocity.z;
    }

    if (std::abs(angle - balance_angle) > PI / 4) {
        // バランスを取るための角度が一定以上外れた場合は停止
        stepper1.setSpeed(0);
        stepper2.setSpeed(0);
        last_control_update_time = micros();
        return;
    }

    float linear_speed = (stepper1.speed() + stepper2.speed()) * PI / SPR; // rad/s
    // float angular_speed = ((stepper1.speed() - stepper2.speed()) * 2 * PI * WHEEL_RADIUS / WHEEL_DISTANCE / SPR); // rad/s

    // float goal_rot_speed = ((stepper1.speed() - stepper2.speed()) * 2 * PI / SPR * WHEEL_RADIUS / WHEEL_DISTANCE - robot_cmd_vel.angular.z);

    // lqrで求めたゲインで制御
    // float u1 = K[0] * (angle - balance_angle) + K[1] * omega + K[2] * (stepper1.speed() * 2 * PI / SPR - robot_cmd_vel.linear.x / WHEEL_RADIUS); // rad/s
    // float u2 = K[0] * (angle - balance_angle) + K[1] * omega + K[2] * (stepper2.speed() * 2 * PI / SPR - robot_cmd_vel.linear.x / WHEEL_RADIUS);

    float u_linear = K[0] * (angle - balance_angle) + K[1] * omega + K[2] * (linear_speed - robot_cmd_vel.linear.x / WHEEL_RADIUS);
    float u_angular = std::min(PI, std::max(-PI, (rotation_omega - robot_cmd_vel.angular.z) * WHEEL_DISTANCE / WHEEL_RADIUS));
    unsigned long micro_dt = (micros() - last_control_update_time);

    // ステッピングモータの速度制御
    wheel_step_speed[0] = (u_linear + u_angular / 2) * micro_dt * 0.000001 * SPR / 2 / PI + stepper1.speed(); // rad/s -> steps/s
    wheel_step_speed[1] = (u_linear - u_angular / 2) * micro_dt * 0.000001 * SPR / 2 / PI + stepper2.speed();

    last_control_update_time = micros();

    stepper1.setSpeed(int(wheel_step_speed[0])); //- robot_cmd_vel.angular.z * WHEEL_DISTANCE / WHEEL_RADIUS / 2));
    stepper2.setSpeed(int(wheel_step_speed[1])); //+ robot_cmd_vel.angular.z * WHEEL_DISTANCE / WHEEL_RADIUS / 2));

    robot_odom_update(&robot_odom);

    Serial.println(linear_speed);

}

// cmd_velコールバック
void cmd_vel_callback(const void *msgin) {
    geometry_msgs__msg__Twist *msg = (geometry_msgs__msg__Twist *)msgin;
    robot_cmd_vel.linear.x = msg->linear.x;
    robot_cmd_vel.linear.y = msg->linear.y;
    robot_cmd_vel.linear.z = msg->linear.z;
    robot_cmd_vel.angular.x = msg->angular.x;
    robot_cmd_vel.angular.y = msg->angular.y;
    robot_cmd_vel.angular.z = msg->angular.z;
}

// pub_timerコールバック
void pub_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        // message作成
        odom_msg.header.stamp.sec = millis() * 0.001;
        odom_msg.pose.pose.position.x = robot_odom.pose.position.x;
        odom_msg.pose.pose.position.y = robot_odom.pose.position.y;
        odom_msg.pose.pose.position.z = robot_odom.pose.position.z;
        odom_msg.pose.pose.orientation.x = robot_odom.pose.orientation.x;
        odom_msg.pose.pose.orientation.y = robot_odom.pose.orientation.y;
        odom_msg.pose.pose.orientation.z = robot_odom.pose.orientation.z;
        odom_msg.pose.pose.orientation.w = robot_odom.pose.orientation.w;
        odom_msg.twist.twist.linear.x = robot_odom.twist.linear.x;
        odom_msg.twist.twist.linear.y = robot_odom.twist.linear.y;
        odom_msg.twist.twist.linear.z = robot_odom.twist.linear.z;
        odom_msg.twist.twist.angular.x = robot_odom.twist.angular.x;
        odom_msg.twist.twist.angular.y = robot_odom.twist.angular.y;
        odom_msg.twist.twist.angular.z = robot_odom.twist.angular.z;

        // publish
        RCSOFTCHECK(rcl_publish(&pub_odom, &odom_msg, NULL));
    }
}

// コア0での動作(ROS2 Executor)
void core0a(void *pvParameters) {
    while (1) {
        RCSOFTCHECK(rclc_executor_spin_some(&executor_pubsub, RCL_MS_TO_NS(100)));
        delay(1);
    }
}

// コア1での動作(ROS2 Executor)
void core1a(void *pvParameters) {
    while (1) {
        control();
        delay(1);
    }
}

void setup() {
    // シリアル通信の初期化
    Serial.begin(115200);

    if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
		Serial.println("Failed to find BNO08x chip");
		while (1) { delay(10); }
	}
	setReports();

    // IMUのキャリブレーション用のピン設定
    pinMode(PIN_START_BUTTON, INPUT);
    pinMode(PIN_START_LED, OUTPUT);
    digitalWrite(PIN_START_LED, LOW);

    // ステッピングモータのピン設定
    pinMode(PIN_A1, OUTPUT);
    pinMode(PIN_B1, OUTPUT);
    pinMode(PIN_C1, OUTPUT);
    pinMode(PIN_D1, OUTPUT);
    pinMode(PIN_A2, OUTPUT);
    pinMode(PIN_B2, OUTPUT);
    pinMode(PIN_C2, OUTPUT);
    pinMode(PIN_D2, OUTPUT);

    // ステッピングモータの設定
    stepper1.setMaxSpeed(2000);
    stepper1.setAcceleration(1000);
    stepper1.setSpeed(0);
    stepper2.setMaxSpeed(2000);
    stepper2.setAcceleration(1000);
    stepper2.setSpeed(0);
    
    // micro-ROSのWiFi設定
   	set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
    // set_microros_serial_transports(Serial);

    // WiFi接続待ち
    delay(2000);

    // micro-ROSのためのメモリ管理
	allocator = rcl_get_default_allocator();

	// micro-ROSのためのサポートクラス
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // node作成
	RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));

    // Publisher作成
    // RCCHECK(rclc_publisher_init_default(
    //     &pub_imu,
    //     &node, 
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    //     "/esp32/imu"
    // ));
    RCCHECK(rclc_publisher_init_default(
        &pub_odom,
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/esp32/odom"
    ));

    // // Subscription作成
    RCCHECK(rclc_subscription_init_default(
        &sub_cmd_vel,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/esp32/cmd_vel"
    ));

    // Timer作成
    RCCHECK(rclc_timer_init_default(
        &pub_timer,
        &support,
        RCL_MS_TO_NS(20),
        pub_timer_callback
    ));

    executor_pubsub = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor_pubsub, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor_pubsub, &pub_timer));
    RCCHECK(rclc_executor_add_subscription(&executor_pubsub, &sub_cmd_vel, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));

    // スタートボタン
    int cnt = 0;
    while (1) {
        if (digitalRead(PIN_START_BUTTON) == LOW) {
            cnt++;
            digitalWrite(PIN_START_LED, HIGH);
            // break;
        } else {
            cnt = 0;
            digitalWrite(PIN_START_LED, LOW);
        }
        if (cnt > 5) {
            break;
        }
        delay(5);
    }

    // デュアルコアでの動作
    xTaskCreatePinnedToCore(core0a, "Sub-Pub Exe", 4096, NULL, 1, &thp[0], 0);
    xTaskCreatePinnedToCore(core1a, "Control Exe", 8192, NULL, 1, &thp[1], 1);
}

//  メインループ
void loop() {
    stepper1.runSpeed();
    stepper2.runSpeed();
}