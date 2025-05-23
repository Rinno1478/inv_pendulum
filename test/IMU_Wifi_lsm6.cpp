#include <Arduino.h>
#include <Wire.h>
#include <LSM6.h>
#include <MadgwickAHRS.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/quaternion.h>

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

#define interval_ms 10 // LSM6DSOからデータを取得する間隔[ms]

rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_support_t support;
rclc_executor_t executor;

rcl_publisher_t pub_imu;
// rcl_publisher_t pub_quat;

sensor_msgs__msg__Imu imu_msg;
// geometry_msgs__msg__Quaternion quat_msg;

LSM6 imu;

Madgwick MadgwickFilter;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
	while(true) {
		// Serial.println("ROS Error");
		delay(100);
	}
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
		imu.read();
		float imu_acc[3] = {imu.a.x * 0.061f, imu.a.y * 0.061f, imu.a.z * 0.061f};	// 1LSB = 0.061mg
		float imu_gyro[3] = {imu.g.x * 8.75f, imu.g.y * 8.75f, imu.g.z * 8.75f};	// 1LSB = 8.75mdps
		MadgwickFilter.updateIMU(
			imu_acc[0], imu_acc[1], imu_acc[2],
			imu_gyro[0], imu_gyro[1], imu_gyro[2]
		);
		imu_msg.linear_acceleration.x = imu_acc[0] * 0.0098f; // 1 mg = 0.0098 m/s^2
		imu_msg.linear_acceleration.y = imu_acc[1] * 0.0098f;
		imu_msg.linear_acceleration.z = imu_acc[2] * 0.0098f;
		imu_msg.angular_velocity.x = imu_gyro[0] * 0.000017453f; // 1 mdps = 0.000017453 rad/s
		imu_msg.angular_velocity.y = imu_gyro[1] * 0.000017453f;
		imu_msg.angular_velocity.z = imu_gyro[2] * 0.000017453f;
		imu_msg.orientation.x = MadgwickFilter.getQuaternionX();
		imu_msg.orientation.y = MadgwickFilter.getQuaternionY();
		imu_msg.orientation.z = MadgwickFilter.getQuaternionZ();
		imu_msg.orientation.w = MadgwickFilter.getQuaternionW();
		RCCHECK(rcl_publish(&pub_imu, &imu_msg, NULL));
        // RCSOFTCHECK(rcl_publish(&pub_quat, &quat_msg, NULL));
    }
}

void setup(){

	pinMode(21, INPUT_PULLUP);
	pinMode(22, INPUT_PULLUP);

  	set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

	Wire.begin();
	Wire.setClock(400000);	// I2Cの通信速度を400kHzに設定(fast mode)

	if (!imu.init())
	{
		// Serial.println("Could not find a valid BNO055 sensor, check wiring!");
		while (true);
	}

	imu.enableDefault();

	MadgwickFilter.begin(1000 / interval_ms);
	// MadgwickFilter.setGain(0.5);

	delay(2000);
	
	// micro-ROSのためのメモリ管理
	allocator = rcl_get_default_allocator();

	// micro-ROSのためのサポートクラス
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	RCCHECK(rclc_node_init_default(&node, "imu_node", "", &support));

	// IMU Publisher 作成
	RCCHECK(rclc_publisher_init_default(
		&pub_imu,
		&node, 
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		"/imu"
	));

	// Quaternion Publisher 作成
	// RCCHECK(rclc_publisher_init_best_effort(
	// 	&pub_quat,
	// 	&node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
	// 	"/quat"
	// ));

	// Timer 作成
	const unsigned int timer_timeout = interval_ms;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback
	));

	// Executor 作成
	const unsigned int callback_size = 1;
	RCCHECK(rclc_executor_init(&executor, &support.context, callback_size, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

}

void loop(){
	RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(interval_ms)));
	delay(1);
}
