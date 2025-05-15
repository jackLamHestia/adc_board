#include <Arduino.h>
#include "embedded_cli.h"
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include "Wire.h"

#define CORE_0 (1 << 0)
#define CORE_1 (1 << 1)

#define ADC_PORT i2c0
#define ADC_I2C_SDA 8
#define ADC_I2C_SCL 9
#define ADC_I2C_FREQ 400000
#define ADC_ADDRESS 0b1001000
#define ADC_I2C_ADDR_SELECT_PIN 25


EmbeddedCli *cli;
TaskHandle_t led_blink_loop_handle = NULL;
TwoWire ads_i2c(ADC_PORT, ADC_I2C_SDA, ADC_I2C_SCL);


void writeChar(EmbeddedCli *embeddedCli, char c) {
    Serial.write(c);
}

void led_blink_loop(void *pvPramaters) {
    (void)pvPramaters;
    while (1) {
        digitalWrite(25, HIGH);
        delay(1000);
        digitalWrite(25, LOW);
        delay(1000);
    }
}

void setup() {
    EmbeddedCliConfig *config = embeddedCliDefaultConfig();
    config->historyBufferSize = 1024;
    config->maxBindingCount = 32;
    cli = embeddedCliNew(config);
    cli->writeChar = writeChar;

    pinMode(ADC_I2C_ADDR_SELECT_PIN, OUTPUT);
    digitalWrite(ADC_I2C_ADDR_SELECT_PIN, LOW);
    Serial.begin(115200);
    xTaskCreate(led_blink_loop, "led_blink_loop", 128, NULL, 5, &led_blink_loop_handle);
    vTaskCoreAffinitySet(led_blink_loop_handle, CORE_0);
}

void loop() {

    while (Serial.available() > 0) {
        embeddedCliReceiveChar(cli, Serial.read());
    }

    embeddedCliProcess(cli);

    // delay(1000);
}

void setup1() {
    pinMode(25, OUTPUT);
}

void loop1() {
    
}