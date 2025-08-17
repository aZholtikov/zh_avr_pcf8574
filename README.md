# FreeRTOS based AVR library for PCF8574(A) 8-bit I/O expander

## Features

1. Support of 16 expanders on one bus.
2. Support of output and input GPIO's work mode.
3. Support of interrupts from input GPIO's.

## Note

1. Enable interrupt support only if input GPIO's are used.
2. All the INT GPIO's on the extenders must be connected to the one GPIO on AVR. Only PORTD0 - PORTD7 are acceptable!
3. The input GPIO's are always pullup to the power supply.

## Dependencies

1. [zh_avr_free_rtos](http://git.zh.com.ru/avr_libraries/zh_avr_free_rtos)
2. [zh_avr_vector](http://git.zh.com.ru/avr_libraries/zh_avr_vector)
3. [zh_avr_common](http://git.zh.com.ru/avr_libraries/zh_avr_common)
4. [zh_avr_i2c](http://git.zh.com.ru/avr_libraries/zh_avr_i2c)

## Using

In an existing project, run the following command to install the components:

```text
cd ../your_project/lib
git clone http://git.zh.com.ru/avr_libraries/zh_avr_free_rtos
git clone http://git.zh.com.ru/avr_libraries/zh_avr_vector
git clone http://git.zh.com.ru/avr_libraries/zh_avr_i2c
git clone http://git.zh.com.ru/avr_libraries/zh_avr_common
git clone http://git.zh.com.ru/avr_libraries/zh_avr_pcf8574
```

In the application, add the component:

```c
#include "zh_avr_pcf8574.h"
```

## Examples

One expander on bus. All GPIO's as output (except P0 - input). Interrupt is enable:

```c
#include "avr/io.h"
#include "stdio.h"
#include "zh_avr_pcf8574.h"

#define BAUD_RATE 9600
#define BAUD_PRESCALE (F_CPU / 16 / BAUD_RATE - 1)

int usart(char byte, FILE *stream)
{
    while ((UCSR0A & (1 << UDRE0)) == 0)
    {
    }
    UDR0 = byte;
    return 0;
}
FILE uart = FDEV_SETUP_STREAM(usart, NULL, _FDEV_SETUP_WRITE);

zh_avr_pcf8574_handle_t pcf8574_handle = {0};

void print_gpio_status(const char *message, uint8_t reg)
{
    printf("%s", message);
    for (uint8_t i = 0; i < 8; ++i)
    {
        printf("%c", (reg & 0x80) ? '1' : '0');
        reg <<= 1;
    }
    printf(".\n");
}

void pcf8574_example_task(void *pvParameters)
{
    zh_avr_i2c_master_init(false);
    zh_avr_pcf8574_init_config_t pcf8574_init_config = ZH_AVR_PCF8574_INIT_CONFIG_DEFAULT();
    pcf8574_init_config.i2c_address = 0x38;
    pcf8574_init_config.p0_gpio_work_mode = true; // Required only for input GPIO.
    pcf8574_init_config.interrupt_gpio = PORTD4;  // Required only if used input GPIO interrupts.
    zh_avr_pcf8574_init(&pcf8574_init_config, &pcf8574_handle);
    uint8_t reg = 0;
    zh_avr_pcf8574_read(&pcf8574_handle, &reg);
    print_gpio_status("GPIO status: ", reg);
    printf("Set P7 to 1, P1 to 1 and P0 to 0.\n");
    zh_avr_pcf8574_write(&pcf8574_handle, 0b10000010); // GPIO P0 will not be changed because it is operating in input mode.
    zh_avr_pcf8574_read(&pcf8574_handle, &reg);
    print_gpio_status("GPIO status: ", reg);
    printf("Sets P0 to 0.\n");
    zh_avr_pcf8574_write_gpio(&pcf8574_handle, 0, false); // GPIO P0 will not be changed because it is operating in input mode.
    bool gpio = 0;
    zh_avr_pcf8574_read_gpio(&pcf8574_handle, 0, &gpio);
    printf("P0 status: %d.\n", gpio);
    printf("Set P1 to 0.\n");
    zh_avr_pcf8574_write_gpio(&pcf8574_handle, 1, false);
    zh_avr_pcf8574_read_gpio(&pcf8574_handle, 1, &gpio);
    printf("P1 status: %d.\n", gpio);
    zh_avr_pcf8574_read(&pcf8574_handle, &reg);
    print_gpio_status("GPIO status: ", reg);
    printf("Reset all GPIO.\n");
    zh_avr_pcf8574_reset(&pcf8574_handle);
    zh_avr_pcf8574_read(&pcf8574_handle, &reg);
    print_gpio_status("GPIO status: ", reg);
    printf("Task Remaining Stack Size %d.\n", uxTaskGetStackHighWaterMark(NULL));
    vTaskDelete(NULL);
}
int main(void)
{
    UBRR0H = (BAUD_PRESCALE >> 8);
    UBRR0L = BAUD_PRESCALE;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    stdout = &uart;
    xTaskCreate(pcf8574_example_task, "pcf8574 example task", 124, NULL, tskIDLE_PRIORITY, NULL);
    vTaskStartScheduler();
    return 0;
}

void zh_avr_pcf8574_event_handler(zh_avr_pcf8574_event_on_isr_t *event) // Do not delete! Leave blank if interrupts are not used.
{
    printf("Interrupt happened on device address 0x%02X on GPIO number %d at level %d.\n", event->i2c_address, event->gpio_number, event->gpio_level);
    printf("Interrupt Task Remaining Stack Size %d.\n", uxTaskGetStackHighWaterMark(NULL));
}

ISR(PCINT2_vect) // Required only if used input GPIO interrupts.
{
    zh_avr_pcf8574_isr_handler();
}
```
