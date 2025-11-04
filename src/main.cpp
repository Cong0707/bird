#include <../.pio/libdeps/esp32s3/lvgl/lv_conf.h>
#include <lvgl.h>
#include <TFT_eSPI.h>

#include "SensirionI2cSht4x.h"
#include "TAMC_GT911.h"
#include "indev/lv_indev_private.h"

#include <ESP32Servo.h>

Servo espservo;

TFT_eSPI tft = TFT_eSPI();

// 屏幕参数
#define SCREEN_WIDTH  320
#define SCREEN_HEIGHT 240

static lv_color_t buf1[25600];
static lv_color_t buf2[25600];
static lv_display_t * disp; // 显示驱动对象
static lv_indev_t * indev; // LVGL 输入设备句柄

int32_t lastX = 0;
int32_t lastY = 0;
long lastPressTime = 0;

volatile bool touch_pressed = false;

TAMC_GT911 tp = TAMC_GT911(48, 47, 14, 13, 240, 320);
SensirionI2cSht4x sensor;

void disp_flush(lv_display_t * disp_drv, const lv_area_t * area, uint8_t * px_map)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t *)px_map, w * h, true);
    tft.endWrite();

    lv_display_flush_ready(disp_drv); // 刷新完成
}

void input_read(lv_indev_t * indev, lv_indev_data_t * data)
{
    if(tp.isTouched) {
        data->point.x = tp.points[0].y;
        data->point.y = 240 - tp.points[0].x;
        data->state = LV_INDEV_STATE_PRESSED;
        touch_pressed = false;

        lastX = data->point.x;
        lastY = data->point.y;
        lastPressTime = millis();
    } else if (millis() - lastPressTime < 50) {
        data->point.x = lastX;
        data->point.y = lastY;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

[[noreturn]] void touch_task(void *pvParameter) {
    while (true) {
        tp.read();
        if (tp.isTouched) {
            touch_pressed = true;
        }
        vTaskDelay(1);
    }
}

bool servo = false;
bool lightopen = false;
int light_pwm_value = 128;  // 初始亮度 0~255

static void button_event(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_LONG_PRESSED) {
    servo = !servo;
    }
}

static void light_event(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        lightopen = !lightopen;
        if(lightopen) {
            ledcWrite(0, light_pwm_value);  // 恢复亮度
        } else {
            ledcWrite(0, 0);  // 熄灭
        }
    }
}

float tempset = 26;

static void left_event(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target_obj(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        char buf[32];
        lv_roller_get_selected_str(obj, buf, sizeof(buf));
        tempset = atoi(buf);
        LV_LOG_USER("Selected temp: %s\n", buf);
    }
}

static void right_event(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target_obj(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        char buf[32];
        lv_roller_get_selected_str(obj, buf, sizeof(buf));
        LV_LOG_USER("Selected water: %s\n", buf);
    }
}

lv_obj_t * label;
lv_obj_t * button;

lv_obj_t * temp;
lv_obj_t * humidify;

int64_t lastMoveTime;
bool servoAt180 = false;   // 当前状态

int servoIntervalHour = 3;

void moveServo() {
    if (servoAt180) {
        espservo.writeMicroseconds(1000);  // 回到起点（0度）
        Serial.println("舵机回到 0°");
    } else {
        espservo.writeMicroseconds(2000);  // 转到180°
        Serial.println("舵机转动到 180°");
    }
    servoAt180 = !servoAt180;
}

void checkServoTimer() {
    unsigned long now = millis();
    unsigned long interval = servoIntervalHour * 3600UL * 1000UL;
    if (now - lastMoveTime >= interval) {
        moveServo();
        lastMoveTime = now;
    }
}

void setup()
{
    Serial.begin(115200);

    pinMode(4, INPUT);
    pinMode(10, INPUT);

    pinMode(1, OUTPUT);
    pinMode(11, OUTPUT);
    pinMode(21, OUTPUT);   // 灯

    digitalWrite(1, HIGH);  //风扇常开

    ledcSetup(0, 500, 8);  // 5kHz, 8bit分辨率
    ledcAttachPin(21, 0);
    ledcWrite(0, 255);  // 初始亮度

    tft.init();
    tft.setRotation(1);

    // 初始化触摸 IC
    tp.begin(); //里面包括wire begin
    tp.setRotation(1);

    //sht45
    sensor.begin(Wire, SHT45_I2C_ADDR_44);
    sensor.softReset();

    espservo.attach(9);
    espservo.writeMicroseconds(1200);
    delay(500);
    espservo.writeMicroseconds(1800);

    lastMoveTime = millis();

    // 创建独立线程采集触摸数据
    xTaskCreatePinnedToCore(touch_task, "TouchTask", 4096, NULL, 1, NULL, 1);

    lv_init();

    // 创建显示对象（LVGL 9.x 新API）
    disp = lv_display_create(SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_display_set_flush_cb(disp, disp_flush);
    lv_display_set_buffers(disp, buf1, buf2, 25600, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_default(disp);

    lv_indev_t * indev = lv_indev_create();        /* Create input device connected to Default Display. */
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);   /* Touch pad is a pointer-like device. */
    lv_indev_set_read_cb(indev, input_read);    /* Set driver function. */

    label = lv_label_create(lv_screen_active()); // 创建标签
    lv_label_set_text(label, "voltage");  // 设置初始文本

    lv_obj_align(label, LV_ALIGN_BOTTOM_LEFT, 0, 0);


    /**左边滑条*///////////////////
    lv_obj_t * left = lv_roller_create(lv_screen_active());
    lv_roller_set_options(left,
    "26\n27\n28\n29\n30\n31\n32\n33\n34\n35",
        LV_ROLLER_MODE_INFINITE);

    lv_roller_set_visible_row_count(left, 4);
    lv_obj_align(left, LV_ALIGN_LEFT_MID, 10, 0);
    lv_obj_add_event_cb(left, left_event, LV_EVENT_ALL, NULL);

    /**右边滑条*///////////////////
    lv_obj_t * right = lv_roller_create(lv_screen_active());
    lv_roller_set_options(right,
        "1\n2\n3\n4\n5\n6\n7\n8\n9\n10",
        LV_ROLLER_MODE_INFINITE);

    lv_roller_set_visible_row_count(right, 4);
    lv_obj_align(right, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_add_event_cb(right, right_event, LV_EVENT_ALL, NULL);

    /**下面按钮*///////////////////
    lv_obj_t * btn = lv_button_create(lv_screen_active());     /*Add a button the current screen*/
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 70);     // 居中显示
    lv_obj_set_size(btn, 150, 40);                          /*Set its size*/
    lv_obj_add_event_cb(btn, button_event, LV_EVENT_ALL, NULL);           /*Assign a callback to the button*/

    button = lv_label_create(btn);          /*Add a label to the button*/
    lv_label_set_text(button, "fish");                     /*Set the labels text*/
    lv_obj_center(button);

    /**灯按钮*///////////////////
    lv_obj_t * light = lv_button_create(lv_screen_active());     /*Add a button the current screen*/
    lv_obj_align(light, LV_ALIGN_CENTER, -10, 0);     // 居中显示
    lv_obj_set_size(light, 40, 40);                          /*Set its size*/
    lv_obj_add_event_cb(light, light_event, LV_EVENT_ALL, NULL);           /*Assign a callback to the button*/

    lv_obj_t * lightbutton = lv_label_create(light);          /*Add a label to the button*/
    lv_label_set_text(lightbutton, "light");                     /*Set the labels text*/
    lv_obj_center(lightbutton);

    /**灯亮度滑条**/
    lv_obj_t * light_slider = lv_slider_create(lv_screen_active());
    lv_obj_set_width(light_slider, 75);
    lv_obj_align_to(light_slider, lightbutton, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_slider_set_range(light_slider, 0, 255);
    lv_slider_set_value(light_slider, light_pwm_value, LV_ANIM_OFF);
    lv_obj_add_event_cb(light_slider, [](lv_event_t * e) {
        lv_event_code_t code = lv_event_get_code(e);
        if(code == LV_EVENT_VALUE_CHANGED) {
            lv_obj_t * slider = (lv_obj_t *)lv_event_get_target(e);
            int value = lv_slider_get_value(slider);
            if (lightopen) ledcWrite(0, value);
            Serial.printf("灯亮度: %d\n", value);
        }
    }, LV_EVENT_ALL, NULL);

    /**温度*///////////////////
    temp = lv_label_create(lv_screen_active()); // 创建标签
    lv_label_set_text(temp, "temp");  // 设置初始文本

    lv_obj_align(temp, LV_ALIGN_CENTER, -10, -70);

    /**湿度*///////////////////
    humidify = lv_label_create(lv_screen_active()); // 创建标签
    lv_label_set_text(humidify, "humidify");  // 设置初始文本

    lv_obj_align(humidify, LV_ALIGN_CENTER, 10, -50);

    /** 舵机档位滑条（横向） **/
    lv_obj_t * servo_slider = lv_slider_create(lv_screen_active());
    lv_obj_set_size(servo_slider, 100, 10);
    lv_obj_align_to(servo_slider, lightbutton, LV_ALIGN_OUT_TOP_MID, 0, -10);
    lv_slider_set_range(servo_slider, 2, 5);               // 范围 2~5 小时
    lv_slider_set_value(servo_slider, 3, LV_ANIM_OFF);     // 默认 3 小时

    // 标签
    lv_obj_t * servo_label = lv_label_create(lv_screen_active());
    lv_obj_align_to(servo_label, servo_slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);
    lv_label_set_text(servo_label, "间隔: 3小时");

    // 回调函数
    lv_obj_add_event_cb(servo_slider, [](lv_event_t * e) {
        lv_event_code_t code = lv_event_get_code(e);
        if (code == LV_EVENT_VALUE_CHANGED) {
            lv_obj_t * slider = (lv_obj_t *)lv_event_get_target(e);
            int value = lv_slider_get_value(slider);

            // 更新标签显示
            static char buf[16];
            sprintf(buf, "间隔: %d小时", value);
            lv_label_set_text((lv_obj_t *)lv_event_get_user_data(e), buf);

            servoIntervalHour = value;
        }
    }, LV_EVENT_ALL, servo_label);
}

// ======= PID 参数 =======
float Kp = 80.0;
float Ki = 0.5;
float Kd = 2.0;

float integral = 0;
float lastError = 0;
unsigned long lastPidTime = 0;

int pid_compute(float currentTemp) {
    unsigned long now = millis();
    float dt = (now - lastPidTime) / 1000.0f;
    if (dt <= 0.001f) dt = 0.001f; // 防止除零或太小
    lastPidTime = now;

    float error = tempset - currentTemp;

    // 积分限幅（防止 windup）
    float integrator_limit = 200.0f;
    integral += error * dt;
    if (integral > integrator_limit) integral = integrator_limit;
    if (integral < -integrator_limit) integral = -integrator_limit;

    float derivative = (error - lastError) / dt;
    lastError = error;

    float output = Kp * error + Ki * integral + Kd * derivative;

    // 限制输出范围 0..255（直接返回控制信号）
    if (output > 255.0f) output = 255.0f;
    if (output < 0.0f) output = 0.0f;

    //Serial.printf("误差=%.2f, PID输出=%d\n", error, (int)output);

    return (int)(output);
}

void loop()
{
    lv_tick_inc(1);
    lv_timer_handler();
    if (touch_pressed) {
        lv_indev_read(indev);
    }

    checkServoTimer();

    if (millis() % 200 == 0) {
        double dc = analogRead(10) / 4095.0 * 3.3 * 18; //dc口电压
        double pd = analogRead(4) / 4095.0 * 3.3 * 18;  //pd口电压
        lv_label_set_text(label, ("dc: " + std::to_string(dc) + ";pd: " + std::to_string(pd)).c_str());

        float temperture = 0;
        float humidi = 0;
        sensor.measureLowestPrecision(temperture, humidi);

        lv_label_set_text(humidify, ("湿度" + std::to_string(humidi)).c_str());
        lv_label_set_text(temp, ("温度" + std::to_string(temperture)).c_str());
        //这两个就是sht45的温度和湿度
        //Serial.println(("温度" + std::to_string(temperture)).c_str());

        bool ptc = false;

        if (dc >= 1) {
            if (servo) {
                lv_label_set_text(button, "孵化模式");
            } else {
                lv_label_set_text(button, "高性能模式");
                ptc = true;
            }
        } else {
            lv_label_set_text(button, "低性能模式");
            if (millis() % 2 == 0) { ptc = true; }
        }

        if (!ptc) return;

        // === PID 控制输出 ===
        int pidOutput = pid_compute(temperture);

        //Serial.println(("温度设定" + std::to_string(tempset)).c_str());
        // === 基于PID输出的时间比例控制 ===
        static unsigned long windowStartTime = millis();
        const unsigned long windowSize = 1000;

        if (millis() - windowStartTime > windowSize) {
            windowStartTime += windowSize;  // 新一轮窗口
        }

        if ((unsigned long)pidOutput * windowSize / 255 > millis() - windowStartTime) {
            digitalWrite(11, HIGH);
            //Serial.println("加热开启");
        } else {
            digitalWrite(11, LOW);
            //Serial.println("加热关闭");
        }
    }
}

