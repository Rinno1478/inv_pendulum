#include <Arduino.h>
#include <Wire.h>
#include <LSM6.h>
#include <MadgwickAHRS.h>
#include <AccelStepper.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_GIGA) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL) && !defined(ARDUINO_UNOR4_WIFI) && !defined(ARDUINO_OPTA)
#error This example is only available for Arduino Portenta, Arduino Giga R1, Arduino Nano RP2040 Connect, ESP32 Dev module, Wio Terminal, Arduino Uno R4 WiFi and Arduino OPTA WiFi 
#endif

#if (MICRO_ROS_TRANSPORT_ARDUINO_WIFI==1)
// Wifi関連
char ssid[] = "Buffalo-G-2E78";
char psk[] = "e477ttud6vekh";
IPAddress agent_ip(192, 168, 11, 15);
size_t agent_port = 8888;
// char ssid[] = "RinnoiPhone";
// char psk[] = "87urg48ws6";
// IPAddress agent_ip(172, 20, 10, 2);
// size_t agent_port = 8888;
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// #define PIN_A1 27
// #define PIN_B1 26
// #define PIN_C1 25
// #define PIN_D1 33
#define PIN_A1 33
#define PIN_B1 25
#define PIN_C1 26
#define PIN_D1 27
#define PIN_A2 19
#define PIN_B2 18
#define PIN_C2 5
#define PIN_D2 17
#define PIN_IMU_CALIB_INT 4
#define PIN_CALIB_LED 13

#define CALIBRATION_ITERATIONS 1000 // ジャイロセンサのキャリブレーション回数
#define CONTROL_CYCLE_MS 10 // 制御周期(ms)
#define MADGWICK_CYCLE_MS 0.66 // Madgwickフィルタの更新周期(ms)
#define SPR 400 // steps per revolution (ステッピングモータの1回転あたりのステップ数)

TaskHandle_t thp[4];

rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t imu_timer;
rcl_timer_t control_timer;
rclc_support_t support;
rclc_executor_t executor;

rcl_publisher_t pub_imu;

sensor_msgs__msg__Imu imu_msg;

LSM6 imu;

Madgwick MadgwickFilter;

AccelStepper stepper1(AccelStepper::HALF4WIRE, PIN_A1, PIN_B1, PIN_C1, PIN_D1);
AccelStepper stepper2(AccelStepper::HALF4WIRE, PIN_A2, PIN_B2, PIN_C2, PIN_D2);

float imu_acc[3];
float imu_gyro[3];
volatile float gyro_offset[3] = {0.0f, 0.0f, 0.0f};
float gyro_cutoff = 300.0f;
float roll, pitch, yaw;
volatile bool is_button_pressed = false;

// float K[3] = {1000.0f,  541.0529f,  -10.0f}; // lqrで求めたゲイン
float K[3] = {1151.696f, 212.213f, -5.773503f}; // lqrで求めたゲイン

// Error handle loop
void error_loop() {
	while(true) {
		delay(100);
	}
}

// imu_timerコールバック
void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        // message作成
        imu_msg.linear_acceleration.x = imu_acc[0] * 9.8f; // 1G = 9.8 m/s^2
        imu_msg.linear_acceleration.y = imu_acc[1] * 9.8f;
        imu_msg.linear_acceleration.z = imu_acc[2] * 9.8f;
        imu_msg.angular_velocity.x = imu_gyro[0] * 0.017453f; // 1 dps = 0.0174533 rad/s
        imu_msg.angular_velocity.y = imu_gyro[1] * 0.017453f;
        imu_msg.angular_velocity.z = imu_gyro[2] * 0.017453f;
        imu_msg.orientation.x = MadgwickFilter.getQuaternionX();
        imu_msg.orientation.y = MadgwickFilter.getQuaternionY();
        imu_msg.orientation.z = MadgwickFilter.getQuaternionZ();
        imu_msg.orientation.w = MadgwickFilter.getQuaternionW();

        // publish
        RCSOFTCHECK(rcl_publish(&pub_imu, &imu_msg, NULL));
    }
}

// control_timerコールバック(ステッピングモータ制御)
void control_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        // ロール、ピッチ、ヨー角の取得
        roll = MadgwickFilter.getRollRadians();
        // pitch = MadgwickFilter.getPitchRadians();
        // yaw = MadgwickFilter.getYawRadians();

        // lqrで求めたゲインで制御
        float u1 = K[0] * (roll - 1.93) + K[1] * imu_gyro[0] * 0.017453f + K[2] * stepper1.speed() * 2 * PI / SPR;   // rad/s^2
        float u2 = K[0] * (roll - 1.93) + K[1] * imu_gyro[0] * 0.017453f + K[2] * stepper2.speed() * 2 * PI / SPR;

        // ステッピングモータの速度制御
        float speed1 = stepper1.speed() + u1 * CONTROL_CYCLE_MS * 0.001 * SPR / 2 / PI; // rad/s -> steps/s
        float speed2 = stepper2.speed() + u2 * CONTROL_CYCLE_MS * 0.001 * SPR / 2 / PI;

        stepper1.setSpeed(int(speed1));
        stepper2.setSpeed(int(speed2));
    }
}

// calibrate gyro
void IRAM_ATTR calibrate_gyro() {
    if (!is_button_pressed) {
        is_button_pressed = true;
        digitalWrite(PIN_CALIB_LED, HIGH);
        delay(1000);
        for (int i = 0; i < CALIBRATION_ITERATIONS; i++) {
            imu.read();
            gyro_offset[0] += imu.g.x;
            gyro_offset[1] += imu.g.y;
            gyro_offset[2] += imu.g.z;
            if (i % 200 < 100) {
                digitalWrite(PIN_CALIB_LED, LOW);
            } else {
                digitalWrite(PIN_CALIB_LED, HIGH);
            }
            delay(1);
        }
        gyro_offset[0] /= CALIBRATION_ITERATIONS;
        gyro_offset[1] /= CALIBRATION_ITERATIONS;
        gyro_offset[2] /= CALIBRATION_ITERATIONS;
        digitalWrite(PIN_CALIB_LED, LOW);
    }
}

// コア0での動作(ROS2 Executor)
void core0a(void *pvParameters) {
    while (1) {
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
        delay(1);
    }
}

// コア0での動作(LSM6DSOからデータ取得)
void core0b(void *pvParameters) {
    while (1) {
        float start_time = micros();
        // LSM6DSOからデータを取得
        imu.read();

        // 加速度センサの値を取得
        imu_acc[0] = imu.a.x * 0.000061f; // 1LSB = 0.000061G
        imu_acc[1] = imu.a.y * 0.000061f;
        imu_acc[2] = imu.a.z * 0.000061f;

        // ジャイロセンサの値を取得
        imu_gyro[0] = (imu.g.x - gyro_offset[0]) * 0.00875f; // 1LSB = 0.00875dps
        imu_gyro[1] = (imu.g.y - gyro_offset[1]) * 0.00875f;
        imu_gyro[2] = (imu.g.z - gyro_offset[2]) * 0.00875f;
        // if (abs(imu_gyro[0]) < gyro_cutoff) imu_gyro[0] = 0;
        // if (abs(imu_gyro[1]) < gyro_cutoff) imu_gyro[1] = 0;
        // if (abs(imu_gyro[2]) < gyro_cutoff) imu_gyro[2] = 0;
        // Serial.println(imu_gyro[1]);

        // Madgwickフィルタの更新
        MadgwickFilter.updateIMU(
            imu_gyro[0], imu_gyro[1], imu_gyro[2], // deg/s
            imu_acc[0], imu_acc[1], imu_acc[2] // m/s^2
        );

        // サンプリング周期を調整
        float elapsed_time = micros() - start_time;
        // Serial.println(1000000 / elapsed_time);
        if (elapsed_time < MADGWICK_CYCLE_MS * 1000) {
            delayMicroseconds(MADGWICK_CYCLE_MS * 1000 - elapsed_time); 
        }
    }
}

void setup() {
    // シリアル通信の初期化
    Serial.begin(115200);

    // i2Cの初期化
    Wire.begin();
    Wire.setClock(1000000);	// I2Cの通信速度を400kHzに設定(fast mode)

    if (!imu.init())
    {
        Serial.println("Could not find a valid lsm6 sensor, check wiring!");
        while (true);
    }
    imu.enableDefault();

    MadgwickFilter.begin(1000 / MADGWICK_CYCLE_MS);
    MadgwickFilter.setBeta(1.0f);

    // IMUのキャリブレーション用のピン設定
    pinMode(PIN_IMU_CALIB_INT, INPUT);
    pinMode(PIN_CALIB_LED, OUTPUT);
    digitalWrite(PIN_CALIB_LED, LOW);

    // 外部割り込みの設定
    // attachInterrupt(PIN_IMU_CALIB_INT, calibrate_gyro, FALLING);

    // ジャイロセンサのキャリブレーション
    calibrate_gyro();

    xTaskCreatePinnedToCore(core0b, "read_imu", 4096, NULL, 2, &thp[1], 1);


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
    RCCHECK(rclc_publisher_init_default(
        &pub_imu,
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/esp32/imu"
    ));

    // Timer作成
    // const unsigned int timer_timeout = CONTROL_CYCLE_MS;
    RCCHECK(rclc_timer_init_default(
        &imu_timer,
        &support,
        RCL_MS_TO_NS(CONTROL_CYCLE_MS),
        imu_timer_callback
    ));
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(CONTROL_CYCLE_MS),
        control_timer_callback
    ));

    // Executor作成
    const unsigned int handle_num = 2;
	executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, handle_num, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    // スタートボタン
    int cnt = 0;
    while (1) {
        if (digitalRead(PIN_IMU_CALIB_INT) == LOW) {
            cnt++;
            digitalWrite(PIN_CALIB_LED, HIGH);
            break;
        } else {
            cnt = 0;
            digitalWrite(PIN_CALIB_LED, LOW);
        }
        if (cnt > 5) {
            break;
        }
        delay(10);
    }

    // デュアルコアでの動作
    xTaskCreatePinnedToCore(core0a, "ROS_timer", 4096, NULL, 1, &thp[0], 0);
}

//  メインループ
void loop() {
    stepper1.runSpeed();
    stepper2.runSpeed();
}