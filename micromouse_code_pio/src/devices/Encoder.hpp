#pragma once

#include <Arduino.h>

// The encoder class is a simple interface which counts and stores an encoders count.
// Encoder pin 1 is attached to the interupt on the arduino and used to trigger the count.
// Encoder pin 2 is attached to any digital pin and used to derrive rotation direction.
// The count is stored as a volatile variable due to the high frequency updates.
class Encoder {
public:
    Encoder(uint8_t enc1, uint8_t enc2, int n) : encoder1_pin(enc1), encoder2_pin(enc2) {
        pinMode(encoder1_pin, INPUT_PULLUP);
        pinMode(encoder2_pin, INPUT_PULLUP);

        if (n == 1) {
            instance1 = this;
            attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderISR1, RISING);
        } else if (n == 2) {
            instance2 = this;
            attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderISR2, RISING);
        }
    }

    void begin(void (*isr)()) {
        attachInterrupt(digitalPinToInterrupt(encoder1_pin), isr, RISING);
    }


    // Encoder function used to update the encoder
    void readEncoder() {
        noInterrupts();

        // NOTE: DO NOT PLACE SERIAL PRINT STATEMENTS IN THIS FUNCTION
        // NOTE: DO NOT CALL THIS FUNCTION MANUALLY IT WILL ONLY WORK IF CALLED BY THE INTERRUPT
        // TODO: Increase or Decrease the count by one based on the reading on encoder pin 2'
        if (digitalRead(encoder2_pin) == HIGH) {
            count--;
        } else if (digitalRead(encoder2_pin) == LOW) {
            count++;
        }

        interrupts();
    }

    // Helper function which to convert encouder count to radians
    float getRotation() {

        // TODO: Convert encoder count to radians
        // Task 3 - Find amount radians turned
        return static_cast<float>(count) / counts_per_revolution * 2 * PI;
        // return count;

        // Task 2 - Find counts_per_revolution
    }

    void resetCount() {
        count = 0;
    }

private:
    static void readEncoderISR1() {
        if (instance1 != nullptr) {
            instance1->readEncoder();
        }
    }

    static void readEncoderISR2() {
        if (instance2 != nullptr) {
            instance2->readEncoder();
        }
    }

public:
    const uint8_t encoder1_pin;
    const uint8_t encoder2_pin;
    volatile int8_t direction;
    float position = 0;
    uint16_t counts_per_revolution = 700;
    volatile long count = 0;
    uint32_t prev_time;
    bool read = false;

private:
    static Encoder* instance1;
    static Encoder* instance2;
};

Encoder* Encoder::instance1 = nullptr;
Encoder* Encoder::instance2 = nullptr;