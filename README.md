# LP50XX

## Summary

This is an Kaluma library for interaction with the [LP5009](https://www.ti.com/product/LP5009) and [LP5012](https://www.ti.com/product/LP5012) constant current RGB driver from [Texas Instrument](https://www.ti.com/). 

## Supported platforms

This library should be compatible with all Kaluma-compatible board packages where an I2C bus is available


### Tested boards/platorms:
- Waveshare RP2040 Zero


## Getting started
### Hardware
You can get TSSOP from digikey or mouser and handsolder it to a breakout or PCB.
Currently there are no cheap eval boards available.

### Software
This library is made to work with the [Kaluma](https://kalumajs.org/) platform.


# Wiring

## I2C

Here is a wiring example for `I2C0`.

| Waveshare RP2040 Zero | LP5012    |
|-----------------------|-----------|
| 3V3                   | VCC (VDD) |
| GND                   | GND       |
| GP5 (I2C0 SDA)        | SDA       |
| GP4 (I2C0 SCL)        | SCL (SCK) |

# Usage

## I2C

You can initialize SSD1306 driver using I2C interface as below:

```js
const {LP50XX} = require('LP50XX');
const lp50xx = new LP50XX();
lp50xx.setup(board.i2c(0));

lp50xx.setLEDColor(0, 255,255,255);
```

# API

## I2C

### Class: LP50XX

A class for LP5012 and LP5009 driver communicating with I2C interface.

#### new LP50XX()

Create an instance of LP50XX driver.

#### lp50xx.setup(i2c[, options])

- **`i2c`** `<I2C>` An instance of `I2C` to communicate.
- **`options`** `<object>` Options for initialization.
    - **`address`** `<number>` I2C slave address. Default: `0x14`.
    - **`broadcast`** `<number>` I2C broadcast address. Default: `0x0C`..
    - **`enablePin`** `<number>` LP50XX enable pin Default: `15`.
    - **`ledConf`** `<string>` LP50XX led configuration Default: `LED_Configuration.RGB`.


Setup LP50XX driver for a given I2C bus and options.

#### lp50xx.reset()

Resets the LP50XX driver

#### lp50xx.resetRegisters(mode)

- **`mode`** `<string>`  the I2C address type to write to.

    This translates to the stored addresses in the class Default: `EAddressType.Normal`.

Resets all registers

#### lp50xx.configure(configuration, mode)

- **`configuration`** `<string>` The configuration of the device, this can be made by bitwise OR ('|') the enum 

- **`mode`** `<EAddressType>`  the I2C address type to write to.

  This translates to the stored addresses in the class Default: `EAddressType.Normal`.

A configuration can be `Configure(LED_GLOBAL_ON | MAX_CURRENT_25mA | PWM_DITHERING_ON | AUTO_INC_ON | POWER_SAVE_ON | LOG_SCALE_ON);`

Set LP50XX configuration;

#### lp50xx.setScaling(scaling, mode)

- **`scaling`** `<number>` The scaling of the device. @ref LOG_SCALE_OFF @ref LOG_SCALE_ON

- **`mode`** `<EAddressType>`  the I2C address type to write to.

  This translates to the stored addresses in the class Default: `EAddressType.Normal`.

Sets the PWM scaling used by the device

#### lp50xx.setPowerSaving(powerSaving, mode)

- **`powerSaving`** `<number>` The power saving mode. @ref POWER_SAVE_OFF @ref POWER_SAVE_ON

- **`mode`** `<EAddressType>`  the I2C address type to write to.

  This translates to the stored addresses in the class Default: `EAddressType.Normal`.

Sets the power saving mode of the device

#### lp50xx.setAutoIncrement(autoInc, mode)

- **`powerSaving`** `<number>` The auto increment mode. @ref AUTO_INC_OFF @ref AUTO_INC_ON

- **`mode`** `<EAddressType>`  the I2C address type to write to.

  This translates to the stored addresses in the class Default: `EAddressType.Normal`.

Sets the auto increment mode of the device

#### lp50xx.setPWMDithering(dithering, mode)

- **`dithering`** `<number>` The dithering mode. @ref PWM_DITHERING_OFF @ref PWM_DITHERING_ON

- **`mode`** `<EAddressType>`  the I2C address type to write to.

  This translates to the stored addresses in the class Default: `EAddressType.Normal`.

Sets the PWM dithering of the device

#### lp50xx.setMaxCurrentOption(option, mode)

- **`option`** `<number>` The max current option. @ref MAX_CURRENT_25mA @ref MAX_CURRENT_35mA

- **`mode`** `<EAddressType>`  the I2C address type to write to.

  This translates to the stored addresses in the class Default: `EAddressType.Normal`.

Sets the max current option of the device

#### lp50xx.setLEDGlobalOff(state, mode)

- **`state`** `<number>` The desired setting. @ref LED_GLOBAL_OFF @ref LED_GLOBAL_ON

- **`mode`** `<EAddressType>`  the I2C address type to write to.

  This translates to the stored addresses in the class Default: `EAddressType.Normal`.

Turns all LED outputs ON or OFF

#### lp50xx.setEnabelPin(pin)

- **`pin`** `<number>` Enabel pin

Sets the enable pin of the device. This pin is used to enable the device in Sets the enable pin of the device. This pin is used to enable the device in Begin

#### lp50xx.setLedConfiguration(ledConfiguration)

- **`ledConfiguration`** `<string>` LED_Configuration

Sets the LED configuration acording the LED_Configuration enum

#### lp50xx.setI2CAddress(address)

- **`address`** `<number>` 

Sets the I2C address

#### lp50xx.setBankControl(leds, mode)

- **`leds`** `<number>` The LEDs to include in BANK control. See @ref LP50XX_LEDS

- **`mode`** `<EAddressType>`  the I2C address type to write to.

  This translates to the stored addresses in the class Default: `EAddressType.Normal`.

Enables or Disables BANK control for specific LEDs

#### lp50xx.setBankBrightness(brightness, mode)

- **`brightness`** `<number>` The brightness level from 0 to 0xFF

- **`mode`** `<EAddressType>`  the I2C address type to write to.

  This translates to the stored addresses in the class Default: `EAddressType.Normal`.

Sets the brightness level of the whole BANK

#### lp50xx.setBankColorA(a, mode)

- **`a`** `<number>` The color value from 0 to 0xFF

- **`mode`** `<EAddressType>`  the I2C address type to write to.

  This translates to the stored addresses in the class Default: `EAddressType.Normal`.

Sets BANK color A related to Output 0,3,6,9


#### lp50xx.setBankColorB(b, mode)

- **`b`** `<number>` The color value from 0 to 0xFF

- **`mode`** `<EAddressType>`  the I2C address type to write to.

  This translates to the stored addresses in the class Default: `EAddressType.Normal`.

Sets BANK color B related to Output 1,4,7,10

#### lp50xx.setBankColorC(c, mode)

- **`c`** `<number>` The color value from 0 to 0xFF

- **`mode`** `<EAddressType>`  the I2C address type to write to.

  This translates to the stored addresses in the class Default: `EAddressType.Normal`.

Sets BANK color C related to Output 2,5,8,11

#### lp50xx.setBankColor(red, green, blue, mode)

- **`red`** `<number>` The color value from 0 to 0xFF
- **`green`** `<number>` The color value from 0 to 0xFF
- **`blue`** `<number>` The color value from 0 to 0xFF

- **`mode`** `<EAddressType>`  the I2C address type to write to.

  This translates to the stored addresses in the class Default: `EAddressType.Normal`.

Sets the BANK color according to the set LED configuration @ref SetLEDConfiguration

#### lp50xx.setLEDBrightness(led, brightness, mode)
- **`led`** `<number>` The led to set. 0..3
- **`brightness`** `<number>` The brightness level from 0 to 0xFF
- **`mode`** `<EAddressType>`  the I2C address type to write to.

  This translates to the stored addresses in the class Default: `EAddressType.Normal`.

Sets the brightness level of a single LED (3 outputs)

#### lp50xx.setOutputValue(output, value, mode)
- **`output`** `<number>` The Output to set. 0..11
- **`value`** `<number>` The brightness level from 0 to 0xFF
- **`mode`** `<EAddressType>`  the I2C address type to write to.

  This translates to the stored addresses in the class Default: `EAddressType.Normal`.

Sets the color level of a single output

#### lp50xx.setLEDColor(led, red, green, blue, mode)
- **`led`** `<number>` The led to set. 0..3
- **`red`** `<number>` The color value from 0 to 0xFF
- **`green`** `<number>` The color value from 0 to 0xFF
- **`blue`** `<number>` The color value from 0 to 0xFF

- **`mode`** `<EAddressType>`  the I2C address type to write to.

  This translates to the stored addresses in the class Default: `EAddressType.Normal`.


Sets the LED color according to the set LED configuration @ref SetLEDConfiguration

#### lp50xx.writeRegister(reg, value, mode)
- **`reg`** `<number>` The register to write to
- **`value`** `<number>` The value to write to the register
- **`mode`** `<EAddressType>`  the I2C address type to write to.

  This translates to the stored addresses in the class Default: `EAddressType.Normal`.
  
Writes a value to a specified register. @warning only use if you know what you're doing

#### lp50xx.readRegister(reg, amount)
- **`reg`** `<number>` The register to read from
- **`amount`** `<number>` of bytes to read
- **`mode`** `<EAddressType>`  the I2C address type to write to.

    This translates to the stored addresses in the class Default: `EAddressType.Normal`.

- **Returns**: `<Uint8ArraY>` bytes read from register.

Reads a value from a specified register.


