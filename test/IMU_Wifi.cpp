#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
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

#define BNO055interval_ms 100 // BNO055からデータを取得する間隔[ms]

rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_support_t support;
rclc_executor_t executor;

// rcl_publisher_t pub_imu;
rcl_publisher_t pub_quat;

// sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Quaternion quat_msg;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

imu::Vector<3> acc;
imu::Vector<3> gyro;
imu::Vector<3> mag;
imu::Quaternion quat;

Madgwick MadgwickFilter;

float euler[3];

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
	while(true) {
		// Serial.println("ROS Error");
		delay(100);
	}
}

void get_bno055_data(){
	acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
	gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
	mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
	// quat = bno.getQuat();
}

void calc_euler(){
	MadgwickFilter.update(
		gyro.x() * 57.29578, gyro.y() * 57.29578, gyro.z() * 57.29578,	// rad/s -> deg/s
		acc.x(), acc.y(), acc.z(),
		mag.x(), mag.y(), mag.z()
	);
	euler[0] = MadgwickFilter.getRoll();
	euler[1] = MadgwickFilter.getPitch();
	euler[2] = MadgwickFilter.getYaw();
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
		get_bno055_data();
		// imu_msg.linear_acceleration.x = acc.x();
		// imu_msg.linear_acceleration.y = acc.y();
		// imu_msg.linear_acceleration.z = acc.z();
		// imu_msg.angular_velocity.x = gyro.x();
		// imu_msg.angular_velocity.y = gyro.y();
		// imu_msg.angular_velocity.z = gyro.z();
		// imu_msg.orientation.x = quat.x();
		// imu_msg.orientation.y = quat.y();
		// imu_msg.orientation.z = quat.z();
		// imu_msg.orientation.w = quat.w();
		// quat_msg.x = quat.x();
		// quat_msg.y = quat.y();
		// quat_msg.z = quat.z();
		// quat_msg.w = quat.w();
		calc_euler();
		quat_msg.x = euler[0];
		quat_msg.y = euler[1];
		quat_msg.z = euler[2];
		quat_msg.w = 0;
        RCSOFTCHECK(rcl_publish(&pub_quat, &quat_msg, NULL));
    }
}

void setup(){

	pinMode(21, INPUT_PULLUP);
	pinMode(22, INPUT_PULLUP);

  	set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

	Wire.begin();
	Wire.setClock(400000);	// I2Cの通信速度を400kHzに設定(fast mode)

	if (!bno.begin())
	{
		// Serial.println("Could not find a valid BNO055 sensor, check wiring!");
		while (true);
	}

	MadgwickFilter.begin(1000 / BNO055interval_ms);
	// MadgwickFilter.setGain(0.5);

	delay(2000);
	
	// micro-ROSのためのメモリ管理
	allocator = rcl_get_default_allocator();

	// micro-ROSのためのサポートクラス
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	RCCHECK(rclc_node_init_default(&node, "bno055_node", "", &support));

	// IMU Publisher 作成
	// RCCHECK(rclc_publisher_init_default(
	// 	&pub_imu,
	// 	&node, 
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
	// 	"/bno055/imu"
	// ));

	// Quaternion Publisher 作成
	RCCHECK(rclc_publisher_init_best_effort(
		&pub_quat,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
		"/quat"
	));

	// Timer 作成
	const unsigned int timer_timeout = BNO055interval_ms;
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
	RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(BNO055interval_ms)));
	delay(BNO055interval_ms);
}
