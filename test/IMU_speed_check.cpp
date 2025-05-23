#include <Arduino.h>
#include <Wire.h>
#include <LSM6.h>
#include <MadgwickAHRS.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#define INTERVAL_MS 50 //データを取得する間隔[ms]

TaskHandle_t thp[2];

rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_support_t support;
rclc_executor_t executor;
rcl_publisher_t pub;

LSM6 imu;

Madgwick MadgwickFilter;

std_msgs__msg__Int32 msg;

float imu_acc[3];
float imu_gyro[3];

int imu_hz;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
	while(true) {
		Serial.println("ROS Error");
		delay(100);
	}
}

void callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        msg.data = imu_hz;
		imu_hz = 0;
        RCSOFTCHECK(rcl_publish(&pub, &msg, NULL));
    }
}

// コア0での動作(ROS2 Executor)
void core0a(void *pvParameters) {
    while (1) {
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2000)));
        delay(1);
    }
}


void setup(){
  	Serial.begin(115200);
	set_microros_serial_transports(Serial);

	Wire.begin();
	Wire.setClock(1000000);

	if (!imu.init())
	{
		Serial.println("Could not find a valid IMU sensor, check wiring!");
		while (true);
	}
	imu.enableDefault();

	delay(10);
	
	// micro-ROSのためのメモリ管理
	allocator = rcl_get_default_allocator();

	// micro-ROSのためのサポートクラス
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));

	// Publisher 作成
	RCCHECK(rclc_publisher_init_default(
		&pub,
		&node, 
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"/imu_hz"
	));

	// Timer 作成
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		callback
	));

	// Executor 作成
	int callback_size = 1;	// コールバックを行う数
	executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, callback_size, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	xTaskCreatePinnedToCore(core0a, "ROS_Timer", 4096, NULL, 1, &thp[0], 0);
}

void loop(){
	imu.read();
	if (!(imu_acc[0] == imu.a.x * 0.000061f && imu_acc[1] == imu.a.y * 0.000061f && imu_acc[2] == imu.a.z * 0.000061f &&
		imu_gyro[0] == imu.g.x * 0.00875f && imu_gyro[1] == imu.g.y * 0.00875f && imu_gyro[2] == imu.g.z * 0.00875f)) {

		// 加速度センサの値を取得
		imu_acc[0] = imu.a.x * 0.000061f; // 1LSB = 0.000061G
		imu_acc[1] = imu.a.y * 0.000061f;
		imu_acc[2] = imu.a.z * 0.000061f;

		// ジャイロセンサの値を取得
		imu_gyro[0] = imu.g.x * 0.00875f; // 1LSB = 0.00875dps
		imu_gyro[1] = imu.g.y * 0.00875f;
		imu_gyro[2] = imu.g.z * 0.00875f;

		MadgwickFilter.updateIMU(
			imu_gyro[0], imu_gyro[1], imu_gyro[2], // deg/s
			imu_acc[0], imu_acc[1], imu_acc[2] // m/s^2
		);
		imu_hz++;
	}
}
