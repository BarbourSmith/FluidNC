// Copyright (c) 2024 Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file with
// following exception: it may not be used for any reason by MakerMade or anyone with a business or personal connection to MakerMade

/***************************************************
 *   This is a library to interact with A DC Motor
 *
 *  By Bar Smith for Maslow CNC
 ****************************************************/

#include "DCMotor.h"

#include "esp_adc/adc_oneshot.h"

extern void bootTrace(const char*);  // TEMPORARY bring-up diagnostics

// ADC access via the IDF oneshot driver.  Arduino core 3's analogRead() can
// wedge on ADC2 pins (the TL current sense is GPIO18 = ADC2_CH7) while WiFi
// holds the ADC2 arbitration lock.  Reading through adc_oneshot with an
// explicit error path degrades to "no reading" instead of blocking.
#ifndef ADC_ATTEN_DB_12
#    define ADC_ATTEN_DB_12 ADC_ATTEN_DB_11
#endif
static adc_oneshot_unit_handle_t adcUnits[2] = { nullptr, nullptr };

static adc_oneshot_unit_handle_t adcUnitHandle(adc_unit_t unit) {
    if (adcUnits[unit] == nullptr) {
        adc_oneshot_unit_init_cfg_t cfg = {};
        cfg.unit_id                     = unit;
        cfg.ulp_mode                    = ADC_ULP_MODE_DISABLE;
        if (adc_oneshot_new_unit(&cfg, &adcUnits[unit]) != ESP_OK) {
            adcUnits[unit] = nullptr;
        }
    }
    return adcUnits[unit];
}


#define motorPWMFreq 16000
#define motorPWMRes 10

/*!
 *  @brief  Instantiates a new DCMotor class for generic two-wire control
 *  @param  forwardPin Output pin number for the motr. If this pin is at max
 *          output and the other pin is at 0 the motor turns forward
 *  @param  backwardPin Output pin number for the motor. If this pin is at
 *          max output and the other pin is at 0 the motor turns backward
 *  @param  readbackPin ESP32 adc_channel_t pin number for current read-back
 */
DCMotor::DCMotor() {}

void DCMotor::begin(uint8_t forwardPin, uint8_t backwardPin, int readbackPin, int channel1, int channel2) {
    _forward  = forwardPin;
    _back     = backwardPin;
    _readback = readbackPin;
    _channel1 = channel1;
    _channel2 = channel2;

    //Setup the motor controllers.  Arduino core 3 auto-assigns LEDC channels
    //per pin; ledcWrite now takes the pin rather than the channel number.
    ledcAttach(_forward, motorPWMFreq, motorPWMRes);
    ledcWrite(_forward, 0);  //Turn the motor off

    ledcAttach(_back, motorPWMFreq, motorPWMRes);
    ledcWrite(_back, 0);

    bootTrace("DC adc map");
    //Set up the current-sense ADC channel through the oneshot driver
    _adcValid = false;
    if (adc_oneshot_io_to_channel(_readback, &_adcUnit, &_adcChannel) == ESP_OK) {
        bootTrace("DC adc unit");
        if (auto handle = adcUnitHandle(_adcUnit)) {
            bootTrace("DC adc cfg");
            adc_oneshot_chan_cfg_t ch_cfg = {};
            ch_cfg.atten                  = ADC_ATTEN_DB_12;
            ch_cfg.bitwidth               = ADC_BITWIDTH_12;
            if (adc_oneshot_config_channel(handle, _adcChannel, &ch_cfg) == ESP_OK) {
                _adcValid = true;
            }
        }
    }
}

/*!
 *  @brief  Run the motors forward at the given speed
 *  @param speed The speed the motor should spin (0-1023)
 */
void DCMotor::forward(uint16_t speed) {
    runAtSpeed(FORWARD, speed);
}

/*!
 *  @brief  Run the motors forward at max speed
 */
void DCMotor::fullOut() {
    runAtSpeed(FORWARD, _maxSpeed);
}

/*!
 *  @brief  Run the motors backward at the given speed
 *  @param speed The speed the motor should spin (0-1023)
 */
void DCMotor::backward(uint16_t speed) {
    runAtSpeed(BACKWARD, speed);
}

/*!
 *  @brief  Run the motors backward at max speed
 */
void DCMotor::fullIn() {
    runAtSpeed(BACKWARD, _maxSpeed);
}

/*!
 *  @brief  Run the motors backward at half max speed
 */
void DCMotor::halfIn() {
    runAtPWM(_maxSpeed / -2);
}

/*!
 *  @brief  Run the motors at the given speed. Interpret sign as backward for
 *  negative and forward for positive
 *  @param speed The speed the motor should spin (-1023 to 1023)
 */
void DCMotor::runAtPWM(long signed_speed) {
    //Motor driver accepts -maxPWMvalue to maxPWMvalue but doesn't begin moving until motorStartsToMovePWM so we scale

    int  motorStartsToMovePWM = 75;
    int  maxPWMvalue          = 1023;
    long scaledSpeed          = map(abs(signed_speed), 0, maxPWMvalue, motorStartsToMovePWM, _maxSpeed);

    if (signed_speed < 0) {
        runAtSpeed(BACKWARD, scaledSpeed);
    } else {
        runAtSpeed(FORWARD, scaledSpeed);
    }
}

/*!
 *  @brief  Run the motors in the given direction at the given speed. All other
 *  speed setting functions use this to actually write to the outputs
 *  @param  direction Direction backward (0) or forward (1, or ~0)
 *  @param speed The pwm frequency sent to the motor (0-1023)
 */
void DCMotor::runAtSpeed(uint8_t direction, uint16_t speed) {
    if (direction == 0) {
        ledcWrite(_forward, _maxSpeed);
        ledcWrite(_back, _maxSpeed - speed);

    } else {
        ledcWrite(_back, _maxSpeed);
        ledcWrite(_forward, _maxSpeed - speed);
    }
}

/*!
 *  @brief  Stop the motors in a braking state
 */
void DCMotor::stop() {
    //These could be set to 1023 to allow coasting
    ledcWrite(_forward, 0);  //Stop
    ledcWrite(_back, 0);
}

/*!
 *  @brief  Stop the motors in a high-z state
 */
void DCMotor::highZ() {
    ledcWrite(_forward, 0);  //Stop
    ledcWrite(_back, 0);
}

/*!
 *  @brief  Read the value from an ADC and calculate the current. Allows
 *  multisampling to smooth signal
 *  NOTE: ESP32 adcs are non-linear and have deadzones at top and bottom.
 *        This value bottoms out above 0mA!
 *  @return Reading of current is in arbitrary units. 0 is no current, 4095 is max. TODO: Compute max in mA based on resistor choices.
 *
 */
double DCMotor::readCurrent() {
    if (!_adcValid) {
        return 0;
    }
    // On the ESP32-S3 with IDF 5 / Arduino core 3, ADC2 cannot be read while
    // WiFi is active (the radio owns the SAR ADC2 arbitration); attempting it
    // can wedge the system.  The TL current sense is on GPIO18 = ADC2_CH7, so
    // that channel reports its last pre-WiFi reading (0 on a fresh boot).
    // TODO: revisit if a safe ADC2-with-WiFi read path becomes available.
    if (_adcUnit == ADC_UNIT_2) {
        return _lastCurrentReading;
    }
    int raw = 0;
    if (adc_oneshot_read(adcUnits[_adcUnit], _adcChannel, &raw) == ESP_OK) {
        _lastCurrentReading = raw;
    }
    return _lastCurrentReading;
}
