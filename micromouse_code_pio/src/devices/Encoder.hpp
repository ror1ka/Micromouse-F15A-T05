#pragma once

#include <Arduino.h>

// The encoder class is a simple interface which counts and stores an encoders count.
// Encoder pin A is attached to the interupt on the arduino and used to trigger the count.
// Encoder pin B is attached to any digital pin and used to derrive rotation direction.
// The count is stored as a volatile variable due to the high frequency updates.
//
// Only two encoders can exist at once, because the Nano only has two external
// interrupt pins. Which of the two static ISR slots an instance claims is chosen
// by the Side passed to the constructor.
class Encoder {
public:
    enum class Side : uint8_t { Left, Right };

    Encoder(uint8_t encA, uint8_t encB, Side side, uint16_t counts_per_revolution)
        : encoderA_pin(encA), encoderB_pin(encB), counts_per_revolution(counts_per_revolution) {
        pinMode(encoderA_pin, INPUT_PULLUP);
        pinMode(encoderB_pin, INPUT_PULLUP);

        if (side == Side::Left) {
            instanceLeft = this;
            attachInterrupt(digitalPinToInterrupt(encoderA_pin), readEncoderISRLeft, RISING);
        } else {
            instanceRight = this;
            attachInterrupt(digitalPinToInterrupt(encoderA_pin), readEncoderISRRight, RISING);
        }
    }

    // Encoder function used to update the encoder
    void readEncoder() {
        // NOTE: DO NOT PLACE SERIAL PRINT STATEMENTS IN THIS FUNCTION
        // NOTE: DO NOT CALL THIS FUNCTION MANUALLY IT WILL ONLY WORK IF CALLED BY THE INTERRUPT
        // Interrupts are already disabled by the hardware for the duration of the
        // ISR, so this must not re-enable them - doing so would let interrupts nest.
        if (digitalRead(encoderB_pin) == HIGH) {
            count--;
        } else {
            count++;
        }
    }

    // Helper function which converts encoder count to radians
    float getRotation() const {
        return static_cast<float>(getCount()) / counts_per_revolution * 2 * PI;
    }

    void resetCount() {
        // count is multi-byte and written from an ISR, so the write has to be
        // atomic from the main loop's point of view.
        noInterrupts();
        count = 0;
        interrupts();
    }

    long getCount() const {
        noInterrupts();
        long snapshot = count;
        interrupts();
        return snapshot;
    }

private:
    static void readEncoderISRLeft() {
        if (instanceLeft != nullptr) {
            instanceLeft->readEncoder();
        }
    }

    static void readEncoderISRRight() {
        if (instanceRight != nullptr) {
            instanceRight->readEncoder();
        }
    }

    const uint8_t encoderA_pin;
    const uint8_t encoderB_pin;
    const uint16_t counts_per_revolution;
    volatile long count = 0;

    static Encoder* instanceLeft;
    static Encoder* instanceRight;
};

Encoder* Encoder::instanceLeft = nullptr;
Encoder* Encoder::instanceRight = nullptr;
