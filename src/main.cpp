#include <Arduino.h>
#include "embedded_cli.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#define CORE_0 (1 << 0)
#define CORE_1 (1 << 1)

#define ADC_PORT i2c0
#define ADC_I2C_SDA 8
#define ADC_I2C_SCL 9
#define ADC_I2C_FREQ 400000
#define ADC_ADDRESS 0x48
#define ADC_I2C_ADDR_SELECT_PIN 25

// LED related constants
#define LED_PIN 25
#define LED_BLINK_DELAY_MS 1000

// ADC command constants
#define ADC_CMD_START 0x01
#define ADC_CMD_CHANNEL_MASK 0xC1
#define ADC_CMD_END 0x83
#define ADC_REG_POINTER 0x00

// Function to scan I2C bus and check for device
static bool check_i2c_device(uint16_t address)
{
    uint8_t dummy = 0;
    int result = i2c_write_blocking_until(ADC_PORT, address, &dummy, 1, false, 10000);
    return (result != PICO_ERROR_GENERIC && result != PICO_ERROR_TIMEOUT);
}

static void scan_i2c_bus()
{
    Serial.println("\nI2C Bus Scan\n");
    Serial.printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr)
    {
        if (addr % 16 == 0)
        {
            Serial.printf("%02x ", addr);
        }

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        ret = i2c_read_blocking(ADC_PORT, addr, &rxdata, 1, false);

        if (ret >= 0)
        {
            Serial.printf("@");
            Serial.printf(" (dec: %d) ret: %d", addr, ret);
        }
        else
        {
            Serial.printf(".");
        }
        Serial.printf(addr % 16 == 15 ? "\n" : "  ");
    }
    Serial.println("\n");
}

EmbeddedCli *cli;
TaskHandle_t led_blink_loop_handle = NULL;

static void cmd_read_adc(EmbeddedCli *cli, char *args, void *context)
{
    (void)context;  // Cast unused parameter to void
    int16_t tokenCount = embeddedCliGetTokenCount(args);
    uint8_t channel = 0;
    if (tokenCount != 1)
    {
        Serial.println("Token Number is not correct");
        return;
    }
    else
    {
        channel = atoi(embeddedCliGetToken(args, 1));
    }

    uint8_t cmd[3] = {ADC_CMD_START, ADC_CMD_CHANNEL_MASK | (channel << 4), ADC_CMD_END};
    int msg;
    uint8_t data[2];

    // Print debug information
    Serial.printf("Sending command to ADC at address 0x%02X\n", ADC_ADDRESS);
    Serial.printf("Command bytes: 0x%02X 0x%02X 0x%02X\n", cmd[0], cmd[1], cmd[2]);

    // Increased timeout to 1000000μs (1s)
    msg = i2c_write_blocking(ADC_PORT, ADC_ADDRESS, cmd, 3, false);
    Serial.printf("msg: %d\n", msg);
    if (msg == PICO_ERROR_GENERIC)
    {
        Serial.printf("Device Not Exists - Check I2C address and connections\n");
        return;
    }
    else if (msg == PICO_ERROR_TIMEOUT)
    {
        Serial.printf("Device Timeout - Operation took longer than 1s\n");
        return;
    }
    else
    {
        Serial.printf("Successfully sent %d bytes\n", msg);
    }

    sleep_ms(10);

    // Write pointer register to read the conversion result
    msg = i2c_write_blocking(ADC_PORT, ADC_REG_POINTER, &cmd[0], 1, false);
    if (!msg)
    {
        return;
    }

    msg = i2c_read_blocking(ADC_PORT, ADC_ADDRESS, data, 2, false);
    if (!msg)
    {
        return;
    }

    int16_t result = (int16_t)(data[0] << 8 | data[1]);
    float voltage = (float)result;
    Serial.printf("Voltage Result: %d\n", voltage);
}

static void cmd_write_char(EmbeddedCli *embeddedCli, char c)
{
    Serial.write(c);
}

static void led_blink_thread(void *pvParameters)  // Fixed typo in parameter name
{
    (void)pvParameters;
    for (;;)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(LED_BLINK_DELAY_MS);
        digitalWrite(LED_PIN, LOW);
        delay(LED_BLINK_DELAY_MS);
    }
}

void cmd_scan_i2c_device(EmbeddedCli *cli, char *args, void *context)
{
    (void)cli;
    (void)args;
    (void)context;
    scan_i2c_bus();
}

void setup()
{
    // shell
    EmbeddedCliConfig *config = embeddedCliDefaultConfig();
    config->historyBufferSize = 1024;
    config->maxBindingCount = 32;
    cli = embeddedCliNew(config);
    cli->writeChar = cmd_write_char;

    pinMode(ADC_I2C_ADDR_SELECT_PIN, OUTPUT);
    digitalWrite(ADC_I2C_ADDR_SELECT_PIN, LOW);

    i2c_init(ADC_PORT, 100 * 1000);
    gpio_init(ADC_I2C_SDA);
    gpio_init(ADC_I2C_SCL);
    gpio_set_function(ADC_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(ADC_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(ADC_I2C_SDA);
    gpio_pull_up(ADC_I2C_SCL);
    bi_decl(bi_2pins_with_fucn(ADC_I2C_SDA, ADC_I2C_SCL, GPIO_FUNC_I2C));

    // Check if ADC is connected
    Serial.println("\nChecking ADC connection...");

    embeddedCliAddBinding(cli, {"adc", "Read ADC", false, nullptr, cmd_read_adc});
    embeddedCliAddBinding(cli, {"scan", "Scan I2C bus for devices", false, nullptr, cmd_scan_i2c_device});

    xTaskCreate(led_blink_thread, "led_blink_loop", 128, NULL, 5, &led_blink_loop_handle);
    vTaskCoreAffinitySet(led_blink_loop_handle, CORE_0);
}

void loop()
{

    while (Serial.available() > 0)
    {
        embeddedCliReceiveChar(cli, Serial.read());
    }

    embeddedCliProcess(cli);
    // delay(1000);
}

void setup1(void)  // Added void parameter
{
    pinMode(LED_PIN, OUTPUT);
}

void loop1(void)  // Added void parameter
{
}