#include "soc/reg_base.h" // DR_REG_GPIO_BASE, DR_REG_IO_MUX_BASE
#include "driver/rtc_io.h" // rtc_gpio_*
#include "pin.h"

#define REGISTER_SIZE 32
#define NUM_PINS 40

#define ERR_INVALID_PIN (-1)

// GPIO Matrix Registers
#define GPIO_OUT_REG (DR_REG_GPIO_BASE + 0x04)
#define GPIO_OUT_W1TS_REG (DR_REG_GPIO_BASE + 0x08)
#define GPIO_OUT_W1TC_REG (DR_REG_GPIO_BASE + 0x0C)

#define GPIO_OUT1_REG (DR_REG_GPIO_BASE + 0x10)
#define GPIO_OUT1_W1TS_REG (DR_REG_GPIO_BASE + 0x14)
#define GPIO_OUT1_W1TC_REG (DR_REG_GPIO_BASE + 0x18)

#define GPIO_ENABLE_REG (DR_REG_GPIO_BASE + 0x20)
#define GPIO_ENABLE_W1TS_REG (DR_REG_GPIO_BASE + 0x24)
#define GPIO_ENABLE_W1TC_REG (DR_REG_GPIO_BASE + 0x28)

#define GPIO_ENABLE1_REG (DR_REG_GPIO_BASE + 0x2C)
#define GPIO_ENABLE1_W1TS_REG (DR_REG_GPIO_BASE + 0x30)
#define GPIO_ENABLE1_W1TC_REG (DR_REG_GPIO_BASE + 0x34)

#define GPIO_IN_REG (DR_REG_GPIO_BASE + 0x3C)
#define GPIO_IN1_REG (DR_REG_GPIO_BASE + 0x40)

#define GPIO_PIN_REG(n) (DR_REG_GPIO_BASE+ 0x88 + (0x4 * (n)))
#define GPIO_FUNC_OUT_SEL_CFG_REG(n) (DR_REG_GPIO_BASE + 0x0530 + (0x04 * (n)))
#define GPIO_FUNC_OUT_SEL_FIELD 0

// IO MUX Registers
#define IO_MUX_REG(n) (DR_REG_IO_MUX_BASE + (PIN_MUX_REG_OFFSET[n]))

// IO MUX Register Fields
#define FUN_WPD_BIT 7
#define FUN_WPU_BIT 8
#define FUN_IE_BIT 9
#define FUN_DRV_FIELD 10
#define MCU_SEL_FIELD 12

#define PAD_DRIVER_BIT 2

#define GPIO_FUNC_OUT_SEL_FIELD_DEFAULT 0x100
#define MCU_SEL_DEFAULT 2
#define FUN_DRV_DEFAULT 2

#define REG(r) (*(volatile uint32_t*)(r))
#define REG_SET_BIT(r,b) (REG(r) |= (0x01 << (b)))
#define REG_CLR_BIT(r,b) (REG(r) &= ~(0x01 << (b)))
#define REG_GET_BIT(r,b) ((REG(r) >> (b)) & 0x01)
#define REG_SET_FIELD(r,b,v) (REG(r) |= (v) << (b))
#define REG_CLR_ALL(r) (REG(r) = 0)

// Gives byte offset of IO_MUX Configuration Register
// from base address DR_REG_IO_MUX_BASE
static const uint8_t PIN_MUX_REG_OFFSET[] = {
    0x44, 0x88, 0x40, 0x84, 0x48, 0x6c, 0x60, 0x64, // pin  0- 7
    0x68, 0x54, 0x58, 0x5c, 0x34, 0x38, 0x30, 0x3c, // pin  8-15
    0x4c, 0x50, 0x70, 0x74, 0x78, 0x7c, 0x80, 0x8c, // pin 16-23
    0x90, 0x24, 0x28, 0x2c, 0xFF, 0xFF, 0xFF, 0xFF, // pin 24-31
    0x1c, 0x20, 0x14, 0x18, 0x04, 0x08, 0x0c, 0x10, // pin 32-39
};


// Reset the configuration of a pin to not be an input or an output.
// Pull-up is enabled so the pin does not float.
int32_t pin_reset(pin_num_t pin)
{
    if (rtc_gpio_is_valid_gpio(pin)) { // hand-off work to RTC subsystem
        rtc_gpio_deinit(pin);
        rtc_gpio_pullup_en(pin);
        rtc_gpio_pulldown_dis(pin);
    }
    // Reset GPIO_PINn_REG: All fields zero
    REG_CLR_ALL(GPIO_PIN_REG(pin));

    // Reset GPIO_FUNCn_OUT_SEL_CFG_REG: GPIO_FUNCn_OUT_SEL=0x100
    REG_CLR_ALL(GPIO_FUNC_OUT_SEL_CFG_REG(pin));
    REG_SET_FIELD(GPIO_FUNC_OUT_SEL_CFG_REG(pin), GPIO_FUNC_OUT_SEL_FIELD, GPIO_FUNC_OUT_SEL_FIELD_DEFAULT);

    //Reset IO_MUX_x_REG: MCU_SEL=2, FUN_DRV=2, FUN_WPU_BIT=1
    REG_CLR_ALL(IO_MUX_REG(pin));
    REG_SET_FIELD(IO_MUX_REG(pin), MCU_SEL_FIELD, MCU_SEL_DEFAULT);
    REG_SET_FIELD(IO_MUX_REG(pin), FUN_DRV_FIELD, FUN_DRV_DEFAULT);
    REG_SET_BIT(IO_MUX_REG(pin), FUN_WPU_BIT);

    // Now that the pin is reset, set the output level to zero
    return pin_set_level(pin, 0);
}

// Enable or disable a pull-up on the pin.
int32_t pin_pullup(pin_num_t pin, bool enable)
{
    if (rtc_gpio_is_valid_gpio(pin)) { // hand-off work to RTC subsystem
        if (enable) return rtc_gpio_pullup_en(pin);
        else return rtc_gpio_pullup_dis(pin);
    }
    if (enable) {
        REG_SET_BIT(IO_MUX_REG(pin), FUN_WPU_BIT);
    }
    else {
        REG_CLR_BIT(IO_MUX_REG(pin), FUN_WPU_BIT);
    }
    return 0;
}

// Enable or disable a pull-down on the pin.
int32_t pin_pulldown(pin_num_t pin, bool enable)
{
    if (rtc_gpio_is_valid_gpio(pin)) { // hand-off work to RTC subsystem
        if (enable) return rtc_gpio_pulldown_en(pin);
        else return rtc_gpio_pulldown_dis(pin);
    }
    if (enable) {
        REG_SET_BIT(IO_MUX_REG(pin), FUN_WPD_BIT);
    }
    else {
        REG_CLR_BIT(IO_MUX_REG(pin), FUN_WPD_BIT);
    }
    return 0;
}

// Enable or disable the pin as an input signal.
int32_t pin_input(pin_num_t pin, bool enable)
{
    if (pin < 0 || pin >= NUM_PINS) {
        return ERR_INVALID_PIN;
    }
    if (enable) {
        REG_SET_BIT(IO_MUX_REG(pin), FUN_IE_BIT);
    }
    else {
        REG_CLR_BIT(IO_MUX_REG(pin), FUN_IE_BIT);
    }
    return 0;
}

// Enable or disable the pin as an output signal.
int32_t pin_output(pin_num_t pin, bool enable)
{
    if (pin < 0 || pin >= NUM_PINS) {
        return ERR_INVALID_PIN;
    }
    // check if pin number is valid. If pin > 31, set the bit in the second GPIO register rather than the first
    if (pin < REGISTER_SIZE) {
        if (enable) {
            REG_SET_BIT(GPIO_ENABLE_W1TS_REG, pin);
        }
        else {
            REG_SET_BIT(GPIO_ENABLE_W1TC_REG, pin);
        }
    }
    else {
        pin -= REGISTER_SIZE;
        if (enable) {
            REG_SET_BIT(GPIO_ENABLE1_W1TS_REG, pin);
        }
        else {
            REG_SET_BIT(GPIO_ENABLE1_W1TC_REG, pin);
        }
    }
    return 0;
}

// Enable or disable the pin as an open-drain signal.
int32_t pin_odrain(pin_num_t pin, bool enable)
{
    if (pin < 0 || pin >= NUM_PINS) {
        return ERR_INVALID_PIN;
    }
    if (enable) {
        REG_SET_BIT(GPIO_PIN_REG(pin), PAD_DRIVER_BIT);
    }
    else {
        REG_CLR_BIT(GPIO_PIN_REG(pin), PAD_DRIVER_BIT);
    }
    return 0;
}

// Sets the output signal level if the pin is configured as an output.
int32_t pin_set_level(pin_num_t pin, int32_t level)
{
    if (pin < 0 || pin >= NUM_PINS) {
        return ERR_INVALID_PIN;
    }
    // check if pin number is valid. If pin > 31, set the bit in the second GPIO register rather than the first
    if (pin < REGISTER_SIZE) {
        if (level) {
            REG_SET_BIT(GPIO_OUT_W1TS_REG, pin);
        }
        else {
            REG_SET_BIT(GPIO_OUT_W1TC_REG, pin);
        }
    }
    else {
        pin -= REGISTER_SIZE;
        if (level) {
            REG_SET_BIT(GPIO_OUT1_W1TS_REG, pin);
        }
        else {
            REG_SET_BIT(GPIO_OUT1_W1TC_REG, pin);
        }
    }
    return 0;
}

// Gets the input signal level if the pin is configured as an input.
int32_t pin_get_level(pin_num_t pin)
{
    if (pin < 0 || pin >= NUM_PINS) {
        return ERR_INVALID_PIN;
    }
    if (pin < REGISTER_SIZE) {
        return REG_GET_BIT(GPIO_IN_REG, pin);
    }
    else {
        pin -= REGISTER_SIZE;
        return REG_GET_BIT(GPIO_IN1_REG, pin);
    }
}

// Get the value of the input registers, one pin per bit.
// The two 32-bit input registers are concatenated into a uint64_t.
uint64_t pin_get_in_reg(void)
{
    return (((uint64_t)REG(GPIO_IN1_REG)) << REGISTER_SIZE) | REG(GPIO_IN_REG);
}

// Get the value of the output registers, one pin per bit.
// The two 32-bit output registers are concatenated into a uint64_t.
uint64_t pin_get_out_reg(void)
{
    return (((uint64_t)REG(GPIO_OUT1_REG)) << REGISTER_SIZE) | REG(GPIO_OUT_REG);
}
