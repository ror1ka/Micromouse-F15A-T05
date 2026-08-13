#pragma once

#include <Arduino.h>

#include "Micromouse.hpp"
#include "task4_code/MazeMap.hpp"

inline void drawMazeOled(Micromouse& mouse, MazeMap& maze, Pose& pose){
    ///// TEST
    // digitalWrite(LED_BUILTIN, HIGH);
    /////
    // mouse.setupOled();
    auto& oledDisplay = mouse.oled().getDisplay();

    // oledDisplay.setDrawColor(1); //// ADDED FOR TEST

    int cellPixelWidth = 6;
    int offsetFromCornerX = 1;
    int offsetFromCornerY = 1;

    oledDisplay.setFont(u8g2_font_5x7_tr);
    oledDisplay.firstPage();

    while (true) {
        ///// TEST
        // oledDisplay.drawBox(115, 0, 10, 10);
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
                    oledDisplay.drawLine(pixelX, pixelY, pixelX + cellPixelWidth, pixelY);
                } else if (maze.getWallState(row, col, NORTH) == UNKNOWN) {
                    oledDisplay.drawPixel(pixelX + cellPixelWidth/2, pixelY);
                }
                if (maze.getWallState(row, col, EAST) == WALL) {
                    oledDisplay.drawLine(pixelX + cellPixelWidth, pixelY, pixelX + cellPixelWidth, pixelY + cellPixelWidth);
                } else if (maze.getWallState(row, col, EAST) == UNKNOWN) {
                    oledDisplay.drawPixel(pixelX + cellPixelWidth, pixelY + cellPixelWidth/2);
                }
                if (maze.getWallState(row, col, SOUTH) == WALL) {
                    oledDisplay.drawLine(pixelX, pixelY + cellPixelWidth, pixelX + cellPixelWidth, pixelY + cellPixelWidth);
                } else if (maze.getWallState(row, col, SOUTH) == UNKNOWN) {
                    oledDisplay.drawPixel(pixelX + cellPixelWidth/2, pixelY + cellPixelWidth);
                }
                if (maze.getWallState(row, col, WEST) == WALL) {
                    oledDisplay.drawLine(pixelX, pixelY, pixelX, pixelY + cellPixelWidth);
                } else if (maze.getWallState(row, col, WEST) == UNKNOWN) {
                    oledDisplay.drawPixel(pixelX, pixelY + cellPixelWidth/2);
                }
                // Draws a pixel in cells which have been visited
                if (maze.hasBeenVisited(row, col)) {
                    oledDisplay.drawPixel(pixelX + cellPixelWidth/2, pixelY + cellPixelWidth/2);
                }                                                
            }
        }
        // Micromouse curr position on display in middle of current grid
        int mousePixelX = offsetFromCornerX + pose.col * cellPixelWidth + cellPixelWidth/2;
        int mousePixelY = offsetFromCornerY + pose.row * cellPixelWidth + cellPixelWidth/2;

        // Draws mouse at current pose as a little box
        oledDisplay.drawBox(mousePixelX - 1, mousePixelY - 1, 3, 3);

        oledDisplay.setCursor(58, 63);
        oledDisplay.print(maze.getCompletionPercent());
        oledDisplay.print("%");

        // I'm pretty sure this will return false once the entire image is generated 
        // tho need to fully confirm
        if (!oledDisplay.nextPage()) {
            break;
        }
    }
    ///// TEST
    // delay(200);
    // digitalWrite(LED_BUILTIN, LOW);
}