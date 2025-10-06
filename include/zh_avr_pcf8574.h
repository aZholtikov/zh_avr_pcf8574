#pragma once

#include "FreeRTOS.h"
#include "semphr.h"
#include "avr/pgmspace.h"
#include "zh_avr_i2c.h"
#include "zh_avr_vector.h"
#include "avr_err.h"
#include "avr_port.h"
#include "avr_bit_defs.h"

#define ZH_AVR_PCF8574_INIT_CONFIG_DEFAULT()   \
    {                                          \
        .task_priority = configMAX_PRIORITIES, \
        .stack_size = 124,                     \
        .i2c_address = 0xFF,                   \
        .p0_gpio_work_mode = 0,                \
        .p1_gpio_work_mode = 0,                \
        .p2_gpio_work_mode = 0,                \
        .p3_gpio_work_mode = 0,                \
        .p4_gpio_work_mode = 0,                \
        .p5_gpio_work_mode = 0,                \
        .p6_gpio_work_mode = 0,                \
        .p7_gpio_work_mode = 0,                \
        .interrupt_gpio = 0xFF,                \
        .interrupt_port = 0}

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct // Structure for initial initialization of PCF8574 expander.
    {
        uint8_t task_priority;  // Task priority for the PCF8574 expander isr processing. @note It is not recommended to set a value less than configMAX_PRIORITIES.
        uint8_t stack_size;     // Stack size for task for the PCF8574 expander isr processing processing.
        uint8_t i2c_address;    // Expander I2C address.
        bool p0_gpio_work_mode; // Expander GPIO PO work mode. True for input, false for output.
        bool p1_gpio_work_mode; // Expander GPIO P1 work mode. True for input, false for output.
        bool p2_gpio_work_mode; // Expander GPIO P2 work mode. True for input, false for output.
        bool p3_gpio_work_mode; // Expander GPIO P3 work mode. True for input, false for output.
        bool p4_gpio_work_mode; // Expander GPIO P4 work mode. True for input, false for output.
        bool p5_gpio_work_mode; // Expander GPIO P5 work mode. True for input, false for output.
        bool p6_gpio_work_mode; // Expander GPIO P6 work mode. True for input, false for output.
        bool p7_gpio_work_mode; // Expander GPIO P7 work mode. True for input, false for output.
        uint8_t interrupt_gpio; // Interrupt GPIO. @attention Must be same for all PCF8574 expanders.
        uint8_t interrupt_port; // Interrupt port.
    } zh_avr_pcf8574_init_config_t;

    typedef struct // PCF8574 expander handle.
    {
        uint8_t i2c_address;    // Expander I2C address.
        uint8_t gpio_work_mode; // Expander GPIO's work mode.
        uint8_t gpio_status;    // Expander GPIO's status.
        bool is_initialized;    // Expander initialization flag.
        void *system;           // System pointer for use in another components.
    } zh_avr_pcf8574_handle_t;

    typedef struct // Structure for sending data to the event handler when cause an interrupt.
    {
        uint8_t i2c_address; // The i2c address of PCF8574 expander that caused the interrupt.
        uint8_t gpio_number; // The GPIO that caused the interrupt.
        bool gpio_level;     // The GPIO level that caused the interrupt.
    } zh_avr_pcf8574_event_on_isr_t;

    /**
     * @brief Initialize PCF8574 expander.
     *
     * @param[in] config Pointer to PCF8574 initialized configuration structure. Can point to a temporary variable.
     * @param[out] handle Pointer to unique PCF8574 handle.
     *
     * @attention I2C driver must be initialized first.
     *
     * @note Before initialize the expander recommend initialize zh_avr_pcf8574_init_config_t structure with default values.
     *
     * @code zh_avr_pcf8574_init_config_t config = ZH_AVR_PCF8574_INIT_CONFIG_DEFAULT() @endcode
     *
     * @return AVR_OK if success or an error code otherwise.
     */
    avr_err_t zh_avr_pcf8574_init(const zh_avr_pcf8574_init_config_t *config, zh_avr_pcf8574_handle_t *handle);

    /**
     * @brief Read PCF8574 all GPIO's status.
     *
     * @param[in] handle Pointer to unique PCF8574 handle.
     * @param[out] reg Pointer to GPIO's status.
     *
     * @return AVR_OK if success or an error code otherwise.
     */
    avr_err_t zh_avr_pcf8574_read(zh_avr_pcf8574_handle_t *handle, uint8_t *reg);

    /**
     * @brief Set PCF8574 all GPIO's status.
     *
     * @param[in] handle Pointer to unique PCF8574 handle.
     * @param[in] reg GPIO's status.
     *
     * @attention Only the GPIO outputs are affected.
     *
     * @return AVR_OK if success or an error code otherwise.
     */
    avr_err_t zh_avr_pcf8574_write(zh_avr_pcf8574_handle_t *handle, uint8_t reg);

    /**
     * @brief Reset (set to initial) PCF8574 all GPIO's.
     *
     * @param[in] handle Pointer to unique PCF8574 handle.
     *
     * @return AVR_OK if success or an error code otherwise.
     */
    avr_err_t zh_avr_pcf8574_reset(zh_avr_pcf8574_handle_t *handle);

    /**
     * @brief Read PCF8574 GPIO status.
     *
     * @param[in] handle Pointer to unique PCF8574 handle.
     * @param[in] gpio GPIO number.
     * @param[out] status Pointer to GPIO status (true - HIGH, false - LOW).
     *
     * @return AVR_OK if success or an error code otherwise.
     */
    avr_err_t zh_avr_pcf8574_read_gpio(zh_avr_pcf8574_handle_t *handle, uint8_t gpio, bool *status);

    /**
     * @brief Set PCF8574 GPIO status.
     *
     * @param[in] handle Pointer to unique PCF8574 handle.
     * @param[in] gpio GPIO number.
     * @param[in] status GPIO status (true - HIGH, false - LOW).
     *
     * @attention Only the GPIO output is affected.
     *
     * @return AVR_OK if success or an error code otherwise.
     */
    avr_err_t zh_avr_pcf8574_write_gpio(zh_avr_pcf8574_handle_t *handle, uint8_t gpio, bool status);

    /**
     * @brief PCF8574 ISR handler.
     */
    BaseType_t zh_avr_pcf8574_isr_handler(void);

#ifdef __cplusplus
}
#endif