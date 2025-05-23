#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BNO08x.h>
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

#define BNO08X_CS 5
#define BNO08X_INT 33
#define BNO08X_RESET 32

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#define interval_ms 100 // LSM6DSOからデータを取得する間隔[ms]

rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_support_t support;
rclc_executor_t executor;

rcl_publisher_t pub_imu;
// rcl_publisher_t pub_quat;

sensor_msgs__msg__Imu imu_msg;
// geometry_msgs__msg__Quaternion quat_msg;

// LSM6 imu;

// Madgwick MadgwickFilter;

float imu_acc[3];
float imu_gyro[3];
float imu_quat[4];

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
	while(true) {
		// Serial.println("ROS Error");
		delay(100);
	}
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

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
		if (bno08x.wasReset()) {
			Serial.print("sensor was reset ");
			setReports();
		}

		if (!bno08x.getSensorEvent(&sensorValue)) {
			return;
		}

		float imu_acc[3] = {sensorValue.un.accelerometer.x, sensorValue.un.accelerometer.y, sensorValue.un.accelerometer.z};
		float imu_gyro[3] = {sensorValue.un.gyroscope.x, sensorValue.un.gyroscope.y, sensorValue.un.gyroscope.z};
		// MadgwickFilter.updateIMU(
		// 	imu_acc[0], imu_acc[1], imu_acc[2],
		// 	imu_gyro[0], imu_gyro[1], imu_gyro[2]
		// );
		imu_msg.linear_acceleration.x = imu_acc[0];
		imu_msg.linear_acceleration.y = imu_acc[1];
		imu_msg.linear_acceleration.z = imu_acc[2];
		imu_msg.angular_velocity.x = imu_gyro[0];
		imu_msg.angular_velocity.y = imu_gyro[1];
		imu_msg.angular_velocity.z = imu_gyro[2];
		imu_msg.orientation.x = sensorValue.un.gameRotationVector.i;
		imu_msg.orientation.y = sensorValue.un.gameRotationVector.j;
		imu_msg.orientation.z = sensorValue.un.gameRotationVector.k;
		imu_msg.orientation.w = sensorValue.un.gameRotationVector.real;
		RCCHECK(rcl_publish(&pub_imu, &imu_msg, NULL));
        // RCSOFTCHECK(rcl_publish(&pub_quat, &quat_msg, NULL));
    }
}

void setup(){
	Serial.begin(115200);

	// pinMode(21, INPUT_PULLUP);
	// pinMode(22, INPUT_PULLUP);

  	set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

	// Wire.begin();
	// Wire.setClock(400000);	// I2Cの通信速度を400kHzに設定(fast mode)

	if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
		Serial.println("Failed to find BNO08x chip");
		while (1) { delay(10); }
	}

	// for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
	// 	Serial.print("Part ");
	// 	Serial.print(bno08x.prodIds.entry[n].swPartNumber);
	// 	Serial.print(": Version :");
	// 	Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
	// 	Serial.print(".");
	// 	Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
	// 	Serial.print(".");
	// 	Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
	// 	Serial.print(" Build ");
	// 	Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
	// }

	setReports();

	// MadgwickFilter.begin(1000 / interval_ms);
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

	// Timer 作成
	// const unsigned int timer_timeout = interval_ms;
	// RCCHECK(rclc_timer_init_default(
	// 	&timer,
	// 	&support,
	// 	RCL_MS_TO_NS(timer_timeout),
	// 	timer_callback
	// ));

	// Executor 作成
	// const unsigned int callback_size = 1;
	// RCCHECK(rclc_executor_init(&executor, &support.context, callback_size, &allocator));
	// RCCHECK(rclc_executor_add_timer(&executor, &timer));

}

void loop(){
	// RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(interval_ms)));
	// delay(1);
	if (bno08x.wasReset()) {
		Serial.print("sensor was reset ");
		setReports();
	}

	if (!bno08x.getSensorEvent(&sensorValue)) {
		return;
	}

	// MadgwickFilter.updateIMU(
	// 	imu_acc[0], imu_acc[1], imu_acc[2],
	// 	imu_gyro[0], imu_gyro[1], imu_gyro[2]
	// );
	switch (sensorValue.sensorId) {
		case SH2_ACCELEROMETER:
			imu_acc[0] = sensorValue.un.accelerometer.x;
			imu_acc[1] = sensorValue.un.accelerometer.y;
			imu_acc[2] = sensorValue.un.accelerometer.z;
			break;
		case SH2_GYROSCOPE_CALIBRATED:
			imu_gyro[0] = sensorValue.un.gyroscope.x;
			imu_gyro[1] = sensorValue.un.gyroscope.y;
			imu_gyro[2] = sensorValue.un.gyroscope.z;
			break;
		case SH2_GAME_ROTATION_VECTOR:
			imu_quat[0] = sensorValue.un.gameRotationVector.i;
			imu_quat[1] = sensorValue.un.gameRotationVector.j;
			imu_quat[2] = sensorValue.un.gameRotationVector.k;
			imu_quat[3] = sensorValue.un.gameRotationVector.real;
			break;
	}
	imu_msg.linear_acceleration.x = imu_acc[0];
	imu_msg.linear_acceleration.y = imu_acc[1];
	imu_msg.linear_acceleration.z = imu_acc[2];
	imu_msg.angular_velocity.x = imu_gyro[0];
	imu_msg.angular_velocity.y = imu_gyro[1];
	imu_msg.angular_velocity.z = imu_gyro[2];
	imu_msg.orientation.x = imu_quat[0];
	imu_msg.orientation.y = imu_quat[1];
	imu_msg.orientation.z = imu_quat[2];
	imu_msg.orientation.w = imu_quat[3];
	RCCHECK(rcl_publish(&pub_imu, &imu_msg, NULL));
}
