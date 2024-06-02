///
/// Servo Ornithopter code for ESP32 WROOM
///

#include <stdio.h>
#include <iostream>
#include <cstring>
#include <tuple>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "rom/gpio.h"
#include "esp_timer.h"
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>

// EXT interrupt pin (PPM)
#define GPIO_PPM_PIN            GPIO_NUM_4

// PWM Settings
#define SERVO_LEFT_CHANNEL      LEDC_CHANNEL_0
#define SERVO_RIGHT_CHANNEL     LEDC_CHANNEL_1

#define PWM_FREQUENCY           (50)

#define SERVO_LEFT_PIN          GPIO_NUM_32
#define SERVO_RIGHT_PIN         GPIO_NUM_33

const uint32_t SMIN = 1000;
const uint32_t SMID = 1500;
const uint32_t SMAX = 2000;

constexpr int gGlideDeg = 0;
constexpr int MIN_THRO_THRESHOLD = 100; // 10%

SemaphoreHandle_t gMutex = nullptr;

TaskHandle_t mainTaskHandle;
QueueHandle_t gPPMQueue;

uint32_t millis() {
    return esp_timer_get_time() / 1000;
}

template <typename T>
T clamp(const T& value, const T& mi, const T& ma) {
    return  std::min(std::max(value, mi), ma);
}

double map(double input, double in_sta, double in_end, double out_sta, double out_end) {
    double slope = (out_end - out_sta) / (in_end - in_sta);
    if (out_sta < out_end)
        return clamp(out_sta + slope * (input - in_sta), out_sta, out_end);
    return clamp(out_sta + slope * (input - in_sta), out_end, out_sta);
}

void setPWMs(const uint32_t duty1, const uint32_t duty2) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, SERVO_LEFT_CHANNEL, duty1);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, SERVO_LEFT_CHANNEL);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, SERVO_RIGHT_CHANNEL, duty2);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, SERVO_RIGHT_CHANNEL);
}

struct PPM {
    PPM() = default;
    ~PPM() = default;

    uint32_t aileron()    { receive(); return clamp(aile, SMIN, SMAX) - SMID; }
    uint32_t elevator()   { receive(); return clamp(elev, SMIN, SMAX) - SMID; }
    uint32_t throttle()   { receive(); return clamp(thro, SMIN, SMAX) - SMIN; }
    uint32_t rudder()     { receive(); return clamp(rudd, SMIN, SMAX) - SMID; }
    uint32_t period()     { receive(); return map(clamp(ch5, SMIN, SMAX), SMIN, SMAX, 100, 300); }
    uint32_t channel6()   { receive(); return clamp(ch6, SMIN, SMAX) - SMID; } // clamp between -100 and 100

    void receive() {
        if(xQueueReceive(gPPMQueue, this, 0) == pdTRUE) {
            //std::cout << "got data from queue" << std::endl;
        } else {
            //std::cout << "no data from queue" << std::endl;
        }
    }

    uint32_t aile;
    uint32_t elev;
    uint32_t thro;
    uint32_t rudd;
    uint32_t ch5;
    uint32_t ch6;
};

class Bird {
public:
    enum class FlapState {
        DOWNSTROKE,
        UPSTROKE
    };

    Bird() {
        // stackSize = 4096; priority = 10;
        xTaskCreate(&Bird::mainTaskImpl, "mainTask", 4096, this, 10, nullptr); // pass this to the static task method as parameter!
    }

    ~Bird() = default;

    void mainTask() {
        uint32_t max_duty = (1 << LEDC_TIMER_13_BIT) - 1;

        while(1) {

            auto elapsed = 0;
            auto prevTime = millis();
            auto period = mRC.period();

            while(elapsed < period) {

                auto [ servo1Pos, servo2Pos ] = calculateServoPositions();

                uint32_t servo1Duty = (servo1Pos * max_duty) / (1000000 / PWM_FREQUENCY);
                uint32_t servo2Duty = (servo2Pos * max_duty) / (1000000 / PWM_FREQUENCY);

                setPWMs(servo1Duty, servo2Duty);

                vTaskDelay(period / 3 / portTICK_PERIOD_MS);

                elapsed = millis() - prevTime;
            }

            nextState(); // advance to next state: UPSTROKE / DOWNSTROKE

        } // while(1)
    } // mainTask

private:
    static void mainTaskImpl(void* pvParams) {
        auto* bird = static_cast<Bird*>(pvParams);
        bird->mainTask();
    }

    void nextState() {
        mState = static_cast<FlapState>((static_cast<int>(mState) + 1) % 2);
    }

    std::tuple<int, int> calculateServoPositions() {
        int thro = mRC.throttle();
        int aile = mRC.aileron();
        int elev = mRC.elevator();
        int rudd = mRC.rudder();
        int offset = clamp(static_cast<int>(mRC.channel6()), -100, 100); // between -100 and 100

        if(thro < MIN_THRO_THRESHOLD) {
            return { static_cast<int>(SMID + aile - elev + gGlideDeg + offset), static_cast<int>(SMID + aile + elev - gGlideDeg + offset) };
        } else {
            if(mState == FlapState::UPSTROKE) {
                return { static_cast<int>(thro / 2 + 1500 + aile - elev + rudd + offset), // add offset for both servos to have opposing effect
                    static_cast<int>(1000 + (2000 - (thro / 2 + 1500)) + aile + elev + rudd + offset) }; // add offset for both servos to have opposing effect
            } else { // DOWNSTROKE
                return { static_cast<int>(1000 + (2000 - (thro / 2 + 1500)) + aile - elev - rudd + offset),
                    static_cast<int>(thro / 2 + 1500 + aile + elev - rudd + offset) };
            }
        }
    }

private:
    PPM mRC;
    FlapState mState;
};

void setupGPIO() {
    gpio_config_t io_conf;
    // Configure the GPIO pin as input
    io_conf.intr_type = static_cast<gpio_int_type_t>(GPIO_PIN_INTR_ANYEDGE);
    io_conf.pin_bit_mask = (1ULL << GPIO_PPM_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&io_conf);
}

volatile PPM ppm;

void IRAM_ATTR gpio_isr_handler(void* arg) {

    static uint8_t channel = 0;
    static uint32_t pulse = 0;
    static uint32_t counter = 0;
    static uint32_t previousCounter = 0;
    static uint32_t currentMicros = 0;
    uint32_t tmpVal = 0;

    currentMicros = esp_timer_get_time();
    counter = currentMicros - previousCounter;
    previousCounter = currentMicros;

    if (counter < 510) { //must be a pulse
        pulse = counter;
    } else if (counter > 1910) { //sync
        channel = 0;
    } else { //servo values between 810 and 2210 will end up here
        tmpVal = counter + pulse;
        if (tmpVal > 810 && tmpVal < 2210) {
            switch(channel) {
                case 0:
                    ppm.aile = tmpVal;
                    break;
                case 1:
                    ppm.elev = tmpVal;
                    break;
                case 2:
                    ppm.thro = tmpVal;
                    break;
                case 3:
                    ppm.rudd = tmpVal;
                    break;
                case 4:
                    ppm.ch5 = tmpVal;
                    break;
                case 5:
                    ppm.ch6 = tmpVal;
                    break;
            }
        }
        channel++;
    }

    // copy ppm values to queue
    if(channel == 5) {
        BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

        // try this, but use queue with only one element capacity!
        //xQueueOverwriteFromISR(gPPMQueue, &ppm, &pxHigherPriorityTaskWoken);

        xQueueSendFromISR(gPPMQueue, (void*)&ppm, &pxHigherPriorityTaskWoken);
        if (pxHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

void setupInterrupt() {
    // Install the GPIO ISR service
    gpio_install_isr_service(0);

    // Hook ISR handler for specific GPIO pin
    gpio_isr_handler_add(GPIO_PPM_PIN, gpio_isr_handler, (void*)GPIO_PPM_PIN);
}

void setupPWM() {

    ledc_timer_config_t ledc_timer;
    memset(&ledc_timer, 0, sizeof(ledc_timer));
    ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_timer.duty_resolution = LEDC_TIMER_13_BIT;
    ledc_timer.timer_num = LEDC_TIMER_0;
    ledc_timer.freq_hz = PWM_FREQUENCY;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t servo_left;
    memset(&servo_left, 0, sizeof(servo_left));
    servo_left.gpio_num = SERVO_LEFT_PIN;
    servo_left.speed_mode = LEDC_HIGH_SPEED_MODE;
    servo_left.channel = SERVO_LEFT_CHANNEL;
    servo_left.intr_type = LEDC_INTR_DISABLE;
    servo_left.timer_sel = LEDC_TIMER_0;
    servo_left.duty = 0;
    ledc_channel_config(&servo_left);

    ledc_channel_config_t servo_right;
    memset(&servo_right, 0, sizeof(servo_right));
    servo_right.gpio_num = SERVO_RIGHT_PIN;
    servo_right.speed_mode = LEDC_HIGH_SPEED_MODE;
    servo_right.channel = SERVO_RIGHT_CHANNEL;
    servo_right.intr_type = LEDC_INTR_DISABLE;
    servo_right.timer_sel = LEDC_TIMER_0;
    servo_right.duty = 0;
    ledc_channel_config(&servo_right);
}

extern "C" void app_main(void) {
    std::cout << "Flappin'" << std::endl;

    gMutex = xSemaphoreCreateMutex();
    gPPMQueue = xQueueCreate(10, sizeof(ppm));

    setupGPIO();
    setupInterrupt();

    setupPWM();

    Bird bird; // will run forever
}
