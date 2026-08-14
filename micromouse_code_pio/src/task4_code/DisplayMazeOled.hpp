#pragma once

// Generative-AI assistance notice: the compact numeric completion renderer
// marked "AI-assisted" was written with OpenAI Codex and reviewed by the team.

#include <Arduino.h>

#include "Micromouse.hpp"
#include "task4_code/MazeMap.hpp"

// AI-assisted seven-segment digits avoid linking a complete font and Arduino's
// generic Print number formatter into the flash-constrained marking firmware.
// Scale 2 gives a clear 5x9-pixel digit while retaining an explicit percentage.
inline uint8_t completionDigitSegments(uint8_t digit) {
    switch (digit) {
        case 0: return 0x3F;
        case 1: return 0x06;
        case 2: return 0x5B;
        case 3: return 0x4F;
        case 4: return 0x66;
        case 5: return 0x6D;
        case 6: return 0x7D;
        case 7: return 0x07;
        case 8: return 0x7F;
        default: return 0x6F;
    }
}

inline void drawCompletionDigit(u8g2_t& display, uint8_t x, uint8_t y,
                                uint8_t digit) {
    const uint8_t segments = completionDigitSegments(digit);
    if (segments & 0x01) u8g2_DrawLine(&display, x, y, x + 4, y);
    if (segments & 0x02) u8g2_DrawLine(&display, x + 4, y, x + 4, y + 4);
    if (segments & 0x04) u8g2_DrawLine(&display, x + 4, y + 4, x + 4, y + 8);
    if (segments & 0x08) u8g2_DrawLine(&display, x, y + 8, x + 4, y + 8);
    if (segments & 0x10) u8g2_DrawLine(&display, x, y + 4, x, y + 8);
    if (segments & 0x20) u8g2_DrawLine(&display, x, y, x, y + 4);
    if (segments & 0x40) u8g2_DrawLine(&display, x, y + 4, x + 4, y + 4);
}

inline void drawCompletionPercent(u8g2_t& display, uint8_t percent) {
    uint8_t x = 60;
    const uint8_t y = 53;
    if (percent >= 100) {
        drawCompletionDigit(display, x, y, 1);
        x += 7;
    }
    if (percent >= 10) {
        drawCompletionDigit(display, x, y, (percent / 10) % 10);
        x += 7;
    }
    drawCompletionDigit(display, x, y, percent % 10);
    x += 7;
    u8g2_DrawPixel(&display, x, y);
    u8g2_DrawLine(&display, x, y + 8, x + 4, y);
    u8g2_DrawPixel(&display, x + 4, y + 8);
}

inline bool drawMazeOled(Micromouse& mouse, MazeMap& maze, Pose& pose){
    ///// TEST
    // digitalWrite(LED_BUILTIN, HIGH);
    /////
    // mouse.setupOled();
    // Reinitialise while stopped before every frame. An SSD1306 that browned out
    // can still ACK its address while having forgotten all display configuration.
    if (!mouse.setupOled()) return false;
    auto& oledDisplay = mouse.oled().getDisplay();

    // oledDisplay.setDrawColor(1); //// ADDED FOR TEST

    int cellPixelWidth = 6;
    int offsetFromCornerX = 1;
    int offsetFromCornerY = 1;

    u8g2_FirstPage(&oledDisplay);

    while (true) {
        ///// TEST
        // u8g2_DrawBox(&oledDisplay, 115, 0, 10, 10);
        /////
        for (int row = 0; row < MAZE_HEIGHT; row++) {
            for (int col = 0; col < MAZE_WIDTH; col++) {
                if (!maze.inMaze(row, col)) {
                    continue;
                }
                // Finds top left pixel coord for current row & col
                int pixelX = offsetFromCornerX + col * cellPixelWidth;
                int pixelY = offsetFromCornerY + row * cellPixelWidth;

                // Draws walls if they're there w/ a line of length 6 pixels. If unknown it's a dot in middle
                if (maze.getWallState(row, col, NORTH) == WALL) {
                    u8g2_DrawLine(&oledDisplay, pixelX, pixelY, pixelX + cellPixelWidth, pixelY);
                } else if (maze.getWallState(row, col, NORTH) == UNKNOWN) {
                    u8g2_DrawPixel(&oledDisplay, pixelX + cellPixelWidth/2, pixelY);
                }
                if (maze.getWallState(row, col, EAST) == WALL) {
                    u8g2_DrawLine(&oledDisplay, pixelX + cellPixelWidth, pixelY, pixelX + cellPixelWidth, pixelY + cellPixelWidth);
                } else if (maze.getWallState(row, col, EAST) == UNKNOWN) {
                    u8g2_DrawPixel(&oledDisplay, pixelX + cellPixelWidth, pixelY + cellPixelWidth/2);
                }
                if (maze.getWallState(row, col, SOUTH) == WALL) {
                    u8g2_DrawLine(&oledDisplay, pixelX, pixelY + cellPixelWidth, pixelX + cellPixelWidth, pixelY + cellPixelWidth);
                } else if (maze.getWallState(row, col, SOUTH) == UNKNOWN) {
                    u8g2_DrawPixel(&oledDisplay, pixelX + cellPixelWidth/2, pixelY + cellPixelWidth);
                }
                if (maze.getWallState(row, col, WEST) == WALL) {
                    u8g2_DrawLine(&oledDisplay, pixelX, pixelY, pixelX, pixelY + cellPixelWidth);
                } else if (maze.getWallState(row, col, WEST) == UNKNOWN) {
                    u8g2_DrawPixel(&oledDisplay, pixelX, pixelY + cellPixelWidth/2);
                }
                // Draws a pixel in cells which have been visited
                if (maze.hasBeenVisited(row, col)) {
                    u8g2_DrawPixel(&oledDisplay, pixelX + cellPixelWidth/2, pixelY + cellPixelWidth/2);
                }                                                
            }
        }
        // Micromouse curr position on display in middle of current grid
        int mousePixelX = offsetFromCornerX + pose.col * cellPixelWidth + cellPixelWidth/2;
        int mousePixelY = offsetFromCornerY + pose.row * cellPixelWidth + cellPixelWidth/2;

        // Draws mouse at current pose as a little box
        u8g2_DrawBox(&oledDisplay, mousePixelX - 1, mousePixelY - 1, 3, 3);

        drawCompletionPercent(oledDisplay, maze.getCompletionPercent());

        // I'm pretty sure this will return false once the entire image is generated 
        // tho need to fully confirm
        if (!u8g2_NextPage(&oledDisplay)) {
            break;
        }
    }
    ///// TEST
    // delay(200);
    // digitalWrite(LED_BUILTIN, LOW);
    return mouse.oledHealthy();
}
