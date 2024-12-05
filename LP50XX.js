/**
 * @file LP50XX.js
 * @author AlexK99 (alexander.kettner1999@gmail.com)
 * @brief
 * @version 1.0
 * @date 2024-12-03
 *
 * @copyright Copyright (c) 2024
 *
 */

const DEFAULT_ADDRESS = 0x14;
const BROADCAST_ADDRESS = 0x0C;
const DEFAULT_ENABLEPIN = 15;


// R = RED
// G = GREEN
// B = BLUE
const LED_Configuration =  {
    RGB: "RGB",
    GRB: "GRB",
    BGR: "BGR",
    RBG: "RBG",
    GBR: "GBR",
    BRG: "BRG"
};

const LP50XX_LEDS = {
    LED_0: 1,
    LED_1: 2,
    LED_2: 4,
    LED_3: 8
};

// The I2C address type to write to
const EAddressType = {
    Normal: "Normal",
    Broadcast: "Broadcast"
};

const LP50XX_Configuration = {
    LED_GLOBAL_ON: 0 << 0,
    LED_GLOBAL_OFF: 1 << 0,
    MAX_CURRENT_25mA: 0 << 1,
    MAX_CURRENT_35mA: 1 << 1,
    PWM_DITHERING_OFF: 0 << 2,
    PWM_DITHERING_ON: 1 << 2,
    AUTO_INC_OFF: 0 << 3,
    AUTO_INC_ON: 1 << 3,
    POWER_SAVE_OFF: 0 << 4,
    POWER_SAVE_ON: 1 << 4,
    LOG_SCALE_OFF: 0 << 5,
    LOG_SCALE_ON: 1 << 5
};

// Register definitions
const DEVICE_CONFIG0 = 0x00     // Chip_EN
const DEVICE_CONFIG1 = 0x01     // Configurations for Log_scale, Power_save, Auto_inc, PWM_dithering, Max_current_option and LED_Global_off
const LED_CONFIG0 = 0x02        // Contains the BANK configurations
const BANK_BRIGHTNESS = 0x03    // Contains the BANK brightness level
const BANK_A_COLOR = 0x04       // Contains the BANK A color value
const BANK_B_COLOR = 0x05       // Contains the BANK B color value
const BANK_C_COLOR = 0x06       // Contains the BANK C color value
const LED0_BRIGHTNESS = 0x07    // Contains the brightness level for LED 0
const LED1_BRIGHTNESS = 0x08    // Contains the brightness level for LED 1
const LED2_BRIGHTNESS = 0x09    // Contains the brightness level for LED 2
const LED3_BRIGHTNESS = 0x0A    // Contains the brightness level for LED 3 (only on the LP5012)
const OUT0_COLOR = 0x0B         // Contains the color value for output 0
const OUT1_COLOR = 0x0C         // Contains the color value for output 1
const OUT2_COLOR = 0x0D         // Contains the color value for output 2
const OUT3_COLOR = 0x0E         // Contains the color value for output 3
const OUT4_COLOR = 0x0F         // Contains the color value for output 4
const OUT5_COLOR = 0x10         // Contains the color value for output 5
const OUT6_COLOR = 0x11         // Contains the color value for output 6
const OUT7_COLOR = 0x12         // Contains the color value for output 7
const OUT8_COLOR = 0x13         // Contains the color value for output 8
const OUT9_COLOR = 0x14         // Contains the color value for output 9 (only on the LP5012)
const OUT10_COLOR = 0x15        // Contains the color value for output 10 (only on the LP5012)
const OUT11_COLOR = 0x16        // Contains the color value for output 11 (only on the LP5012)
const RESET_REGISTERS = 0x17    // Resets all the registers to their default values


class LP50XX {

    /*----------------------- Initialisation functions --------------------------*/

    /**
     * @brief Setup i2c and configure the device
     *
     * @param i2c I2C Instance board.i2c(0)
     * @param options LP50XX configuration;
     */
    setup(i2c, options) {
        this.i2c = i2c;
        options = Object.assign(
            {
                address: DEFAULT_ADDRESS,
                broadcast: BROADCAST_ADDRESS,
                enablePin: DEFAULT_ENABLEPIN,
                ledConf: LED_Configuration.GRB
            },
            options
        );
        this.address = options.address;
        this.broadcast = options.broadcast;
        this.enablePin = options.enablePin;
        this.ledConf = options.ledConf;
        pinMode(this.enablePin, OUTPUT);
        digitalWrite(this.enablePin, HIGH);
        delayMicroseconds(500);
        this.i2c.write(new Uint8Array([0x00,0x40]), this.getAddress(EAddressType.Normal))
    }

    /**
     *
     * @brief Resets the state of the device
     *
     */
    reset(){
        digitalWrite(this.enablePin, LOW);
        delay(10);
        digitalWrite(this.enablePin, HIGH);
        delayMicroseconds(500);

        this.resetRegisters();

        this.i2c.write(new Uint8Array([DEVICE_CONFIG0,1<<6]),this.getAddress(EAddressType.Normal));
    }

    /**
     * @brief Resets the state of the Device registers#
     *
     * @param mode the I2C address type to write to
     */
    resetRegisters(mode = EAddressType.Normal) {
        this.i2c.write(new Uint8Array([RESET_REGISTERS,0xff]),this.getAddress(mode))
    }

    /*----------------------- Configuration functions ---------------------------*/

    /**
     * @brief Configures the device according to the configuration param
     *
     * @note A configuration can be `Configure(LED_GLOBAL_ON | MAX_CURRENT_25mA | PWM_DITHERING_ON | AUTO_INC_ON | POWER_SAVE_ON | LOG_SCALE_ON);`
     *
     * @param configuration The configuration of the device, this can be made by bitwise OR ('|') the enum @ref LP50XX_Configuration
     * @param mode the I2C address type to write to
     *
     */
    configure(configuration, mode = EAddressType.Normal) {
        this.i2c.write(new Uint8Array([DEVICE_CONFIG1, configuration & 0x3F]),this.getAddress(mode))
    }

    /**
     * @brief Sets the PWM scaling used by the device
     *
     * @param scaling The scaling of the device. @ref LOG_SCALE_OFF @ref LOG_SCALE_ON
     * @param mode the I2C address type to write to
     */
    setScaling(scaling, mode = EAddressType.Normal) {
        let data = this.i2c.memRead(1, this.getAddress(EAddressType.Normal), DEVICE_CONFIG1);

        if(!data) {
            return;
        }

        scaling = scaling & 1 << 5;

        if (scaling >> 5 & 1) {
            data |= 1 << 5;
        } else {
            data &= ~(1 << 5);
        }

        this.i2c.write(new Uint8Array([DEVICE_CONFIG1,data]),this.getAddress(mode))
    }

    /**
     * @brief Sets the power saving mode of the device
     *
     * @param powerSave The power saving mode. @ref POWER_SAVE_OFF @ref POWER_SAVE_ON
     * @param mode the I2C address type to write to
     */
    setPowerSaving(powerSaving, mode = EAddressType.Normal) {
        let data = this.i2c.memRead(1, this.getAddress(EAddressType.Normal), DEVICE_CONFIG1);

        if(!data) {
            return;
        }

        powerSaving = powerSaving & 1 << 4;

        if (powerSaving >> 4 & 1) {
            data |= 1 << 4;
        } else {
            data &= ~(1 << 4);
        }

        this.i2c.write(new Uint8Array([DEVICE_CONFIG1,data]),this.getAddress(mode));
    }

    /**
     * @brief Sets the auto increment mode of the device
     *
     * @param autoInc The auto increment mode. @ref AUTO_INC_OFF @ref AUTO_INC_ON
     * @param mode the I2C address type to write to
     */
    setAutoIncrement(autoInc, mode = EAddressType.Normal) {
        let data = this.i2c.memRead(1, this.getAddress(EAddressType.Normal), DEVICE_CONFIG1);

        if(!data) {
            return;
        }

        autoInc = autoInc & 1 << 3;

        if (autoInc >> 3 & 1) {
            data |= 1 << 3;
        } else {
            data &= ~(1 << 3);
        }

        this.i2c.write(new Uint8Array([ DEVICE_CONFIG1,data]),this.getAddress(mode));
    }

    /**
     * @brief Sets the PWM dithering of the device
     *
     * @param dithering The dithering mode. @ref PWM_DITHERING_OFF @ref PWM_DITHERING_ON
     * @param mode the I2C address type to write to
     */
    setPWMDithering(dithering, mode = EAddressType.Normal) {
        let data = this.i2c.memRead(1, this.getAddress(mode), DEVICE_CONFIG1);

        if(!data) {
            return;
        }

        dithering = dithering & 1 << 2;

        if (dithering >> 2 & 1) {
            data |= 1 << 2;
        } else {
            data &= ~(1 << 2);
        }

        this.i2c.write(new Uint8Array([DEVICE_CONFIG1,data]),this.getAddress(mode));
    }

    /**
     * @brief Sets the max current option of the device
     *
     * @param option The max current option. @ref MAX_CURRENT_25mA @ref MAX_CURRENT_35mA
     * @param mode the I2C address type to write to
     */
    setMaxCurrentOption(option, mode = EAddressType.Normal) {
        let data = this.i2c.memRead(1, this.getAddress(EAddressType.Normal), DEVICE_CONFIG1);

        if(!data) {
            return;
        }

        option = option & 1 << 1;

        if (option >> 1 & 1) {
            data |= 1 << 1;
        } else {
            data &= ~(1 << 1);
        }

        this.i2c.write(new Uint8Array([DEVICE_CONFIG1,data]),this.getAddress(mode));
    }

    /**
     * @brief Turns all LED outputs ON or OFF
     *
     * @param state The desired setting. @ref LED_GLOBAL_OFF @ref LED_GLOBAL_ON
     * @param mode the I2C address type to write to
     */
    setLEDGlobalOff(state, mode = EAddressType.Normal) {
        let data = this.i2c.memRead(1, this.getAddress(EAddressType.Normal), DEVICE_CONFIG1);

        if(!data) {
            return;
        }

        state = state & 1 << 0;
        if (state >> 0 & 1) {
            data |= 1 << 0;
        } else {
            data &= ~(1 << 0);
        }

        this.i2c.write(new Uint8Array([DEVICE_CONFIG1,data]),this.getAddress(mode));
    }

    /**
     * @brief Sets the enable pin of the device. This pin is used to enable the device in @ref Begin
     *
     * @param pin
     */
    setEnabelPin(pin) {
        pinMode(pin, OUTPUT);
        this.enablePin = pin;
    }

    /**
     * @brief Sets the LED configuration acording the @ref LED_Configuration enum
     *
     * @param ledConfiguration
     */
    setLedConfiguration(ledConfiguration) {
        this.ledConf = ledConfiguration;
    }

    /**
     * @brief Sets the I2C address
     *
     * @param address
     */
    setI2CAddress(address) {
        this.address = address;
    }

    /*----------------------- Bank control functions ----------------------------*/

    /**
     * @brief Enables or Disables BANK control for specific LEDs
     *
     * @param leds The LEDs to include in BANK control. See @ref LP50XX_LEDS
     * @param mode the I2C address type to write to
     *
     * @note Code example could be `SetBankControl(LED_0 | LED_1 | LED_2 | LED_3);`
     */
    setBankControl(leds, mode = EAddressType.Normal ) {
        this.i2c.write(new Uint8Array([LED_CONFIG0, leds]),this.getAddress(mode));
    }

    /**
     * @brief Sets the brightness level of the whole BANK
     *
     * @param brightness The brightness level from 0 to 0xFF
     * @param mode the I2C address type to write to
     */
    setBankBrightness(brightness, mode = EAddressType.Normal) {
        this.i2c.write(new Uint8Array([BANK_BRIGHTNESS, brightness]),this.getAddress(mode));
    }

    /**
     * @brief Sets BANK color A related to Output 0,3,6,9
     *
     * @param a The color value from 0 to 0xFF
     * @param mode the I2C address type to write to
     */
    setBankColorA(a, mode = EAddressType.Normal) {
        this.i2c.write(new Uint8Array([BANK_A_COLOR, a]), this.getAddress(mode))
    }

    /**
     * @brief Sets BANK color B related to Output 1,4,7,10
     *
     * @param b The color value from 0 to 0xFF
     * @param mode the I2C address type to write to
     */
    setBankColorB(b, mode) {
        this.i2c.write(new Uint8Array([BANK_B_COLOR, b]), this.getAddress(mode))
    }

    /**
     * @brief Sets BANK color C related to Output 2,5,8,11
     *
     * @param c The color value from 0 to 0xFF
     * @param mode the I2C address type to write to
     */
    setBankColorC(c, mode) {
        this.i2c.write(new Uint8Array([BANK_C_COLOR, c]), this.getAddress(mode))
    }

    /**
     * @brief Sets the BANK color according to the set LED configuration @ref SetLEDConfiguration
     *
     * @param red The red color value from 0 to 0xFF
     * @param green The green color value from 0 to 0xFF
     * @param blue The blue color value from 0 to 0xFF
     * @param mode the I2C address type to write to
     */
    setBankColor( red, green, blue, mode) {

        let data = [0,0,0];

        switch (this.ledConf) {
            case LED_Configuration.RGB:
                data[0] = red;
                data[1] = green;
                data[2] = blue;
                break;
            case LED_Configuration.GRB:
                data[0] = green;
                data[1] = red;
                data[2] = blue;
                break;
            case LED_Configuration.BGR:
                data[0] = blue;
                data[1] = green;
                data[2] = red;
                break;
            case LED_Configuration.RBG:
                data[0] = red;
                data[1] = blue;
                data[2] = green;
                break;
            case LED_Configuration.GBR:
                data[0] = green;
                data[1] = blue;
                data[2] = red;
                break;
            case LED_Configuration.BRG:
                data[0] = blue;
                data[1] = red;
                data[2] = green;
                break;
        }

        this.i2c.write(new Uint8Array([BANK_A_COLOR,data[0]]),this.getAddress(mode));
        this.i2c.write(new Uint8Array([BANK_B_COLOR,data[1]]),this.getAddress(mode));
        this.i2c.write(new Uint8Array([BANK_C_COLOR,data[2]]),this.getAddress(mode));
    }

    /*----------------------- Output control functions --------------------------*/

    /**
     * @brief Sets the brightness level of a single LED (3 outputs)
     *
     * @param led The led to set. 0..3
     * @param brighness The brightness level from 0 to 0xFF
     * @param mode the I2C address type to write to
     */
    setLEDBrightness(led, brighness, mode) {
        this.i2c.write(new Uint8Array([LED0_BRIGHTNESS + led,brighness]),this.getAddress(mode));
    }

    /**
     * @brief Sets the color level of a single output
     *
     * @param output The output to set. 0..11
     * @param value The color value from 0 to 0xFF
     * @param mode the I2C address type to write to
     */
    setOutputValue(output, value, mode) {
        this.i2c.write(new Uint8Array([OUT0_COLOR + output, value]), this.getAddress(mode));
    }

    /**
     * @brief Sets the LED color according to the set LED configuration @ref SetLEDConfiguration
     *
     * @param led The led to set. 0..3
     * @param red The red color value from 0 to 0xFF
     * @param green The green color value from 0 to 0xFF
     * @param blue The blue color value from 0 to 0xFF
     * @param mode the I2C address type to write to
     */
    setLEDColor(led, red, green, blue, mode) {

        let data = [0,0,0];
        switch (this.ledConf)
        {
        case LED_Configuration.RGB:
            data[0] = red;
            data[1] = green;
            data[2] = blue;
            break;
        case LED_Configuration.GRB:
            data[0] = green;
            data[1] = red;
            data[2] = blue;
            break;
        case LED_Configuration.BGR:
            data[0] = blue;
            data[1] = green;
            data[2] = red;
            break;
        case LED_Configuration.RBG:
            data[0] = red;
            data[1] = blue;
            data[2] = green;
            break;
        case LED_Configuration.GBR:
            data[0] = green;
            data[1] = blue;
            data[2] = red;
            break;
        case LED_Configuration.BRG:
            data[0] = blue;
            data[1] = red;
            data[2] = green;
            break;
        }
        this.i2c.write(new Uint8Array([OUT0_COLOR + (led * 3),data[0]]),this.getAddress(mode));
        this.i2c.write(new Uint8Array([OUT0_COLOR + (led * 3+1),data[1]]),this.getAddress(mode));
        this.i2c.write(new Uint8Array([OUT0_COLOR + (led * 3+2),data[2]]),this.getAddress(mode));

    }

    /*----------------------- Low level functions -------------------------------*/

    /**
     * @brief Writes a value to a specified register. @warning only use if you know what you're doing
     *
     * @param reg The register to write to
     * @param value The value to write to the register
     * @param mode the I2C address type to write to
     */
    writeRegister(reg, value, mode) {
        this.i2c.write(new Uint8Array([reg, value]), this.getAddress(mode));
    }

    /**
     * @brief Reads a value from a specified register.
     *
     * @param reg The register to read from
     * @param amount of bytes to read
     * @return Uint8Array data value
     */
    readRegister(reg, amount) {
        let data = this.i2c.memRead(amount, this.getAddress(EAddressType.Normal), reg);
        return data;
    }

    /*------------------------- Helper functions --------------------------------*/

    /*
     *  PRIVATE
     */

    /**
     * @brief Resolves the EAddressType into an address
     *
     * @param mode the I2C address type to write to. This translates to the stored addresses in the class
     * @return uint8_t the I2C address translated from the addressType
     */
    getAddress(mode) {
        let address;
        switch (mode)
        {
            case EAddressType.Broadcast:
            address = this.broadcast;
            break;
        case EAddressType.Normal:
        default:
            address = this.address;
            break;
        }
        return address;
    }

}

module.exports = Object.freeze({
    LP50XX,
    LED_Configuration,
    LP50XX_LEDS,
    EAddressType,
    LP50XX_Configuration,
    DEFAULT_ADDRESS,
    BROADCAST_ADDRESS,
    DEFAULT_ENABLEPIN,
    DEVICE_CONFIG0,
    DEVICE_CONFIG1,
    LED_CONFIG0,
    BANK_BRIGHTNESS,
    BANK_A_COLOR,
    BANK_B_COLOR,
    BANK_C_COLOR,
    LED0_BRIGHTNESS,
    LED1_BRIGHTNESS,
    LED2_BRIGHTNESS,
    LED3_BRIGHTNESS,
    OUT0_COLOR,
    OUT1_COLOR,
    OUT2_COLOR,
    OUT3_COLOR,
    OUT4_COLOR,
    OUT5_COLOR,
    OUT6_COLOR,
    OUT7_COLOR,
    OUT8_COLOR,
    OUT9_COLOR,
    OUT10_COLOR,
    OUT11_COLOR,
})
