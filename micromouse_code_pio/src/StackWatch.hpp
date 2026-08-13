#pragma once

#include <Arduino.h>

// Measures how close the stack has ever come to the top of the globals.
//
// On a 2KB ATmega328P this is not a theoretical worry. The stack grows down
// from RAMEND into whatever the linker put at the top of .bss - which is
// `mouse`, whose last member is the OLED, whose u8g2 struct ends with
// `draw_color`. Overflow it and every draw call renders in black: the display
// keeps taking valid I2C data and paints blank pages down the screen, then
// stays blank. Nothing else visibly breaks, so it does not look like a memory
// fault at all.
//
// The measurement is the standard AVR canary trick. Before main() runs, every
// byte between the end of the globals and the top of RAM is painted with a
// known value. Anything the stack has since touched is no longer that value, so
// counting the surviving run from the bottom gives the high-water mark of the
// whole run so far - not just this instant, which is what makes it able to
// catch a peak that happens deep inside a control loop.
//
// Set STACK_WATCH to false to compile all of it away.
constexpr bool STACK_WATCH = true;

#if 1  // guarded this way so the naked function is not emitted when unused

namespace stackwatch {

constexpr uint8_t CANARY = 0xC5;

// Provided by the linker: end of .bss, and the top of RAM.
extern "C" uint8_t _end;
extern "C" uint8_t __stack;

// Runs from .init1, before main() and before any constructor, so it is painting
// memory nothing owns yet. Naked because there is no stack frame to be had at
// that point - it is writing over where the frame would live.
extern "C" void stackWatchPaint(void)
    __attribute__((naked, used, section(".init1")));

extern "C" void stackWatchPaint(void) {
    __asm volatile(
        "    ldi r30, lo8(_end)\n"
        "    ldi r31, hi8(_end)\n"
        "    ldi r24, lo8(0xc5)\n"
        "    ldi r25, hi8(__stack)\n"
        "    rjmp .stackwatch_cmp\n"
        ".stackwatch_loop:\n"
        "    st  Z+, r24\n"
        ".stackwatch_cmp:\n"
        "    cpi r30, lo8(__stack)\n"
        "    cpc r31, r25\n"
        "    brlo .stackwatch_loop\n"
        "    breq .stackwatch_loop\n" ::);
}

// Bytes of stack that have never been touched. This is the headroom that was
// left over at the worst moment of the run; if it reaches zero the stack has
// been writing into the globals and the numbers below are already suspect.
inline uint16_t bytesNeverUsed() {
    const uint8_t* p = &_end;
    uint16_t count = 0;

    while (p <= &__stack && *p == CANARY) {
        p++;
        count++;
    }

    return count;
}

// Total space the stack has to grow into, i.e. RAM above the globals.
inline uint16_t bytesAvailable() {
    return (uint16_t)(&__stack - &_end) + 1;
}

}  // namespace stackwatch

// Prints the worst-case stack depth reached so far, tagged with `label` so a
// long run can be read back and the deepest phase picked out.
inline void reportStack(const __FlashStringHelper* label) {
    if (!STACK_WATCH) {
        return;
    }

    const uint16_t free = stackwatch::bytesNeverUsed();

    Serial.print(F("stack "));
    Serial.print(label);
    Serial.print(F(": peak used "));
    Serial.print(stackwatch::bytesAvailable() - free);
    Serial.print(F(" of "));
    Serial.print(stackwatch::bytesAvailable());
    Serial.print(F(", "));
    Serial.print(free);
    Serial.println(F(" never touched"));
}

#endif
