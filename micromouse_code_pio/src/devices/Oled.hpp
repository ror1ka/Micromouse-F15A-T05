#pragma once

// Generative-AI assistance notice: marking-time serial diagnostics and the
// checked display-presence probe marked below were written with OpenAI Codex
// assistance and reviewed by the team.

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

class Oled {
public:
    Oled() {
        u8g2_Setup_ssd1306_i2c_128x64_noname_1(
            &u8g2, U8G2_R0, u8x8_byte_arduino_hw_i2c,
            u8x8_gpio_and_delay_arduino);
    }

    bool setup() {
        u8g2_InitDisplay(&u8g2);
        u8g2_ClearDisplay(&u8g2);
        u8g2_SetPowerSave(&u8g2, 0);
        return healthy();
    }

    // AI-assisted spec guard: U8g2 intentionally ignores I2C ACK status. Probe
    // the same address explicitly so Task 4.3 cannot begin with a blank required
    // map/percentage display after a cable fault or OLED brownout.
    bool healthy() {
        Wire.clearWireTimeoutFlag();
        Wire.beginTransmission(OLED_ADDRESS);
        const uint8_t status = Wire.endTransmission();
        if (status != 0 || Wire.getWireTimeoutFlag()) {
            Wire.clearWireTimeoutFlag();
            return false;
        }
        return true;
    }

    void clear() {
        u8g2_FirstPage(&u8g2);
        do {
        } while (u8g2_NextPage(&u8g2));
    }

    // void printMessage(int x, int y, const char* text) {
    //     u8g2.clearBuffer();
    //     u8g2.setFont(u8g2_font_ncenB10_tr); // Slightly smaller font for Nano
    //     u8g2.drawStr(x, y, text);
    //     u8g2.sendBuffer();
    // }

    u8g2_t& getDisplay() {
      return u8g2;
    }
private:
    static constexpr uint8_t OLED_ADDRESS = 0x3C;
    u8g2_t u8g2;

};
