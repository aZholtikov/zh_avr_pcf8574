#include "zh_avr_pcf8574.h"

TaskHandle_t zh_avr_pcf8574 = NULL;
static uint8_t _interrupt_gpio = 0xFF;
static uint8_t _interrupt_port = 0;
static SemaphoreHandle_t _interrupt_semaphore = NULL;
static const uint8_t _gpio_matrix[8] PROGMEM = {AVR_BIT0, AVR_BIT1, AVR_BIT2, AVR_BIT3, AVR_BIT4, AVR_BIT5, AVR_BIT6, AVR_BIT7};
volatile static bool _interrupt_on = false;

static zh_avr_vector_t _vector = {0};

static avr_err_t _zh_avr_pcf8574_validate_config(const zh_avr_pcf8574_init_config_t *config);
static avr_err_t _zh_avr_pcf8574_configure_i2c_device(const zh_avr_pcf8574_init_config_t *config, zh_avr_pcf8574_handle_t *handle);
static avr_err_t _zh_avr_pcf8574_configure_interrupts(const zh_avr_pcf8574_init_config_t *config, zh_avr_pcf8574_handle_t handle);
static void _zh_avr_pcf8574_isr_processing_task(void *pvParameter);
static avr_err_t _zh_avr_pcf8574_read_register(zh_avr_pcf8574_handle_t *handle, uint8_t *reg);
static avr_err_t _zh_avr_pcf8574_write_register(zh_avr_pcf8574_handle_t *handle, uint8_t reg);

avr_err_t zh_avr_pcf8574_init(const zh_avr_pcf8574_init_config_t *config, zh_avr_pcf8574_handle_t *handle)
{
    ZH_ERROR_CHECK(handle != NULL, AVR_ERR_INVALID_ARG);
    ZH_ERROR_CHECK(handle->is_initialized == false, AVR_ERR_INVALID_STATE);
    avr_err_t err = _zh_avr_pcf8574_validate_config(config);
    ZH_ERROR_CHECK(err == AVR_OK, err);
    err = _zh_avr_pcf8574_configure_i2c_device(config, handle);
    ZH_ERROR_CHECK(err == AVR_OK, err);
    err = _zh_avr_pcf8574_write_register(handle, handle->gpio_work_mode);
    if (err != AVR_OK)
    {
        handle->is_initialized = false;
        return err;
    }
    if (config->interrupt_gpio != 0xFF && handle->gpio_work_mode != 0)
    {
        err = _zh_avr_pcf8574_configure_interrupts(config, *handle);
        if (err != AVR_OK)
        {
            handle->is_initialized = false;
            return err;
        }
    }
    handle->is_initialized = true;
    return AVR_OK;
}

avr_err_t zh_avr_pcf8574_read(zh_avr_pcf8574_handle_t *handle, uint8_t *reg)
{
    return _zh_avr_pcf8574_read_register(handle, reg);
}

avr_err_t zh_avr_pcf8574_write(zh_avr_pcf8574_handle_t *handle, uint8_t reg)
{
    return _zh_avr_pcf8574_write_register(handle, (reg | handle->gpio_work_mode));
}

avr_err_t zh_avr_pcf8574_reset(zh_avr_pcf8574_handle_t *handle)
{
    return _zh_avr_pcf8574_write_register(handle, handle->gpio_work_mode);
}

avr_err_t zh_avr_pcf8574_read_gpio(zh_avr_pcf8574_handle_t *handle, uint8_t gpio, bool *status)
{
    ZH_ERROR_CHECK(gpio <= 7, AVR_FAIL);
    uint8_t gpio_temp = pgm_read_byte(&_gpio_matrix[gpio]);
    uint8_t reg_temp = 0;
    avr_err_t err = _zh_avr_pcf8574_read_register(handle, &reg_temp);
    *status = ((reg_temp & gpio_temp) ? 1 : 0);
    return err;
}

avr_err_t zh_avr_pcf8574_write_gpio(zh_avr_pcf8574_handle_t *handle, uint8_t gpio, bool status)
{
    ZH_ERROR_CHECK(gpio <= 7, AVR_FAIL);
    uint8_t gpio_temp = pgm_read_byte(&_gpio_matrix[gpio]);
    if (status == true)
    {
        return _zh_avr_pcf8574_write_register(handle, handle->gpio_status | handle->gpio_work_mode | gpio_temp);
    }
    return _zh_avr_pcf8574_write_register(handle, (handle->gpio_status ^ gpio_temp) | handle->gpio_work_mode);
}

static avr_err_t _zh_avr_pcf8574_validate_config(const zh_avr_pcf8574_init_config_t *config)
{
    ZH_ERROR_CHECK(config != NULL, AVR_ERR_INVALID_ARG);
    ZH_ERROR_CHECK((config->i2c_address >= 0x20 && config->i2c_address <= 0x27) || (config->i2c_address >= 0x38 && config->i2c_address <= 0x3F), AVR_ERR_INVALID_ARG);
    ZH_ERROR_CHECK(config->task_priority > tskIDLE_PRIORITY && config->stack_size >= configMINIMAL_STACK_SIZE, AVR_ERR_INVALID_ARG);
    ZH_ERROR_CHECK(config->interrupt_gpio == 0xFF || (config->interrupt_gpio >= 0 && config->interrupt_gpio <= 7), AVR_ERR_INVALID_ARG);
    if (config->interrupt_gpio != 0xFF)
    {
        ZH_ERROR_CHECK(config->interrupt_port >= AVR_PORTB && config->interrupt_port <= AVR_PORTD, AVR_ERR_INVALID_ARG);
        _interrupt_port = config->interrupt_port;
    }
    return AVR_OK;
}

static avr_err_t _zh_avr_pcf8574_configure_i2c_device(const zh_avr_pcf8574_init_config_t *config, zh_avr_pcf8574_handle_t *handle)
{
    avr_err_t err = zh_avr_i2c_master_probe(config->i2c_address, 1000 / portTICK_PERIOD_MS);
    if (err != AVR_OK)
    {
        return err;
    }
    handle->gpio_work_mode = (config->p7_gpio_work_mode << 7) | (config->p6_gpio_work_mode << 6) | (config->p5_gpio_work_mode << 5) |
                             (config->p4_gpio_work_mode << 4) | (config->p3_gpio_work_mode << 3) | (config->p2_gpio_work_mode << 2) |
                             (config->p1_gpio_work_mode << 1) | (config->p0_gpio_work_mode << 0);
    handle->gpio_status = handle->gpio_work_mode;
    handle->i2c_address = config->i2c_address;
    handle->is_initialized = true;
    return AVR_OK;
}

static avr_err_t _zh_avr_pcf8574_configure_interrupts(const zh_avr_pcf8574_init_config_t *config, zh_avr_pcf8574_handle_t handle)
{
    if (_interrupt_gpio != 0xFF)
    {
        avr_err_t err = zh_avr_vector_push_back(&_vector, &handle);
        ZH_ERROR_CHECK(err == AVR_OK, err)
        return AVR_OK;
    }
    _interrupt_gpio = config->interrupt_gpio;
    avr_err_t err = zh_avr_vector_init(&_vector, sizeof(zh_avr_pcf8574_handle_t));
    ZH_ERROR_CHECK(err == AVR_OK, err);
    err = zh_avr_vector_push_back(&_vector, &handle);
    ZH_ERROR_CHECK(err == AVR_OK, err);
    switch (_interrupt_port)
    {
    case AVR_PORTB:
        DDRB &= ~(1 << _interrupt_gpio);
        PORTB |= (1 << _interrupt_gpio);
        PCICR |= (1 << PCIE0);
        PCMSK0 |= (1 << _interrupt_gpio);
        break;
    case AVR_PORTC:
        DDRC &= ~(1 << _interrupt_gpio);
        PORTC |= (1 << _interrupt_gpio);
        PCICR |= (1 << PCIE1);
        PCMSK1 |= (1 << _interrupt_gpio);
        break;
    case AVR_PORTD:
        DDRD &= ~(1 << _interrupt_gpio);
        PORTD |= (1 << _interrupt_gpio);
        PCICR |= (1 << PCIE2);
        PCMSK2 |= (1 << _interrupt_gpio);
        break;
    default:
        break;
    }
    _interrupt_semaphore = xSemaphoreCreateBinary();
    ZH_ERROR_CHECK(_interrupt_semaphore != NULL, AVR_ERR_NO_MEM);
    BaseType_t x_err = xTaskCreate(_zh_avr_pcf8574_isr_processing_task, "zh_avr_pcf8574", config->stack_size, NULL, config->task_priority, &zh_avr_pcf8574);
    if (x_err != pdPASS)
    {
        vSemaphoreDelete(_interrupt_semaphore);
        return AVR_FAIL;
    }
    return AVR_OK;
}

BaseType_t zh_avr_pcf8574_isr_handler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    switch (_interrupt_port)
    {
    case AVR_PORTB:
        if ((PINB & (1 << _interrupt_gpio)) == (1 << _interrupt_gpio))
        {
            if (_interrupt_on == false)
            {
                _interrupt_on = true;
            }
            break;
        }
        if ((PINB & (1 << _interrupt_gpio)) == 0)
        {
            if (_interrupt_on == true)
            {
                _interrupt_on = false;
                xSemaphoreGiveFromISR(_interrupt_semaphore, &xHigherPriorityTaskWoken);
            }
            break;
        }
        break;
    case AVR_PORTC:
        if ((PINC & (1 << _interrupt_gpio)) == (1 << _interrupt_gpio))
        {
            if (_interrupt_on == false)
            {
                _interrupt_on = true;
            }
            break;
        }
        if ((PINC & (1 << _interrupt_gpio)) == 0)
        {
            if (_interrupt_on == true)
            {
                _interrupt_on = false;
                xSemaphoreGiveFromISR(_interrupt_semaphore, &xHigherPriorityTaskWoken);
            }
            break;
        }
        break;
    case AVR_PORTD:
        if ((PIND & (1 << _interrupt_gpio)) == (1 << _interrupt_gpio))
        {
            if (_interrupt_on == false)
            {
                _interrupt_on = true;
            }
            break;
        }
        if ((PIND & (1 << _interrupt_gpio)) == 0)
        {
            if (_interrupt_on == true)
            {
                _interrupt_on = false;
                xSemaphoreGiveFromISR(_interrupt_semaphore, &xHigherPriorityTaskWoken);
            }
            break;
        }
        break;
    default:
        break;
    }
    return xHigherPriorityTaskWoken;
}

static void _zh_avr_pcf8574_isr_processing_task(void *pvParameter)
{
    for (;;)
    {
        xSemaphoreTake(_interrupt_semaphore, portMAX_DELAY);
        for (uint8_t i = 0; i < zh_avr_vector_get_size(&_vector); ++i)
        {
            zh_avr_pcf8574_handle_t *handle = zh_avr_vector_get_item(&_vector, i);
            if (handle == NULL)
            {
                continue;
            }
            zh_avr_pcf8574_event_on_isr_t event = {0};
            event.i2c_address = handle->i2c_address;
            event.gpio_number = 0xFF;
            uint8_t old_reg = handle->gpio_status;
            uint8_t new_reg = 0;
            avr_err_t err = _zh_avr_pcf8574_read_register(handle, &new_reg);
            if (err != AVR_OK)
            {
                continue;
            }
            for (uint8_t j = 0; j <= 7; ++j)
            {
                if ((handle->gpio_work_mode & pgm_read_byte(&_gpio_matrix[j])) != 0)
                {
                    if ((old_reg & pgm_read_byte(&_gpio_matrix[j])) != (new_reg & pgm_read_byte(&_gpio_matrix[j])))
                    {
                        event.gpio_number = j;
                        event.gpio_level = new_reg & pgm_read_byte(&_gpio_matrix[j]);
                        extern void zh_avr_pcf8574_event_handler(zh_avr_pcf8574_event_on_isr_t * event);
                        zh_avr_pcf8574_event_handler(&event);
                    }
                }
            }
        }
    }
    vTaskDelete(NULL);
}

static avr_err_t _zh_avr_pcf8574_read_register(zh_avr_pcf8574_handle_t *handle, uint8_t *reg)
{
    ZH_ERROR_CHECK(handle != NULL || reg != NULL, AVR_ERR_INVALID_ARG);
    ZH_ERROR_CHECK(handle->is_initialized == true, AVR_ERR_NOT_FOUND);
    avr_err_t err = zh_avr_i2c_master_receive(handle->i2c_address, &handle->gpio_status, sizeof(handle->gpio_status), 1000 / portTICK_PERIOD_MS);
    ZH_ERROR_CHECK(err == AVR_OK, err);
    *reg = handle->gpio_status;
    return AVR_OK;
}

static avr_err_t _zh_avr_pcf8574_write_register(zh_avr_pcf8574_handle_t *handle, uint8_t reg)
{
    ZH_ERROR_CHECK(handle != NULL, AVR_ERR_INVALID_ARG);
    ZH_ERROR_CHECK(handle->is_initialized == true, AVR_ERR_NOT_FOUND);
    avr_err_t err = zh_avr_i2c_master_transmit(handle->i2c_address, &reg, sizeof(reg), 1000 / portTICK_PERIOD_MS);
    ZH_ERROR_CHECK(err == AVR_OK, err);
    handle->gpio_status = reg;
    return AVR_OK;
}