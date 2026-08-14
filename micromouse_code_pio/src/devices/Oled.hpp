#pragma once

#include <Arduino.h>
#include "Wire.h"

#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

class Oled {
public:
    Oled() : u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE) {}

    void setup() {
        u8g2.begin();
    }

    void clear() {
        u8g2.firstPage();
        do {
        } while (u8g2.nextPage());
    }

    // void printMessage(int x, int y, const char* text) {
    //     u8g2.clearBuffer();
    //     u8g2.setFont(u8g2_font_ncenB10_tr); // Slightly smaller font for Nano
    //     u8g2.drawStr(x, y, text);
    //     u8g2.sendBuffer();
    // }

    void printMessage(int x, int y, const char* text) {
        u8g2.setFont(u8g2_font_ncenB10_tr);
        u8g2.firstPage();
        do {
            u8g2.drawStr(x, y, text);
        } while (u8g2.nextPage());
    }

    void printLidar(int f, int l, int r) {
        u8g2.setFont(u8g2_font_ncenB10_tr);
        u8g2.firstPage();
        char front[64];
        char left[64];
        char right[64];

        snprintf(front, sizeof(front), "Front Lidar: %d", f);
        snprintf(left, sizeof(left), "Left Lidar: %d", l);
        snprintf(right, sizeof(right), "Rront Lidar: %d", r);

        do {
            u8g2.drawStr(0, 10, front);
            u8g2.drawStr(0, 30, left);
            u8g2.drawStr(0, 50, right);
        } while (u8g2.nextPage());
    }

    U8G2_SSD1306_128X64_NONAME_1_HW_I2C& getDisplay() {
      return u8g2;
    }
private:
    U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2;

};