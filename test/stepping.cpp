#include <Arduino.h>
#include <AccelStepper.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

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

#define PIN_A 27
#define PIN_B 26
#define PIN_C 25
#define PIN_D 33
// #define PIN_A 19
// #define PIN_B 18
// #define PIN_C 5
// #define PIN_D 17

TaskHandle_t thp[2];

rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_support_t support;
rclc_executor_t executor;

rcl_subscription_t sub;

std_msgs__msg__Int32 msg;

AccelStepper stepper(AccelStepper::HALF4WIRE, PIN_A, PIN_B, PIN_C, PIN_D);

const int stepsPerRevolution = 400;
int stepCount = 400;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


// Error handle loop
void error_loop() {
	while(true) {
		delay(100);
	}
}

void callback(const void *msgin){
    std_msgs__msg__Int32 *msg = (std_msgs__msg__Int32 *)msgin;
    stepCount = msg->data;
    stepper.setSpeed(stepCount);
    // Serial.println(stepCount);
}

// コア0での動作
void core0a(void *pvParameters) {
    while (1) {
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
        delay(1);
        // Serial.println(xPortGetCoreID());
    }
}

void setup() {
    Serial.begin(115200);
    // デュアルコアでの動作
    xTaskCreatePinnedToCore(core0a, "core0a", 8192, NULL, 1, &thp[0], 0);

    pinMode(PIN_A, OUTPUT);
    pinMode(PIN_B, OUTPUT);
    pinMode(PIN_C, OUTPUT);
    pinMode(PIN_D, OUTPUT);

    stepper.setMaxSpeed(2000);
    stepper.setAcceleration(2000);
    stepper.setSpeed(stepCount);

   	set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
    delay(2000);

    // micro-ROSのためのメモリ管理
	allocator = rcl_get_default_allocator();

	// micro-ROSのためのサポートクラス
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	RCCHECK(rclc_node_init_default(&node, "stepper_node", "", &support));

    // Subscriber 作成
	RCCHECK(rclc_subscription_init_default(
        &sub,
        &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"motor_controller"
	));

    // Executor 作成
    const unsigned int callback_size = 1;
	executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, callback_size, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor, &sub, &msg, &callback, ON_NEW_DATA));
}

void loop() {
    stepper.runSpeed();
    // Serial.println(xPortGetCoreID());
}