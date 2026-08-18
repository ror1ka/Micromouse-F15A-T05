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

    // The digits-only subset of the same 5x7 typeface the display used before.
    // Identical metrics (5x7 cell, cap height 6), so the numbers come out
    // pixel-for-pixel as they did - it just carries " *+,-./0123456789:"
    // instead of all of ASCII, which is 174 bytes of flash against 804.
    //
    // The one glyph it does not carry is '%' itself, ASCII 37: every _tn and
    // _mn subset in u8g2 stops at ':'. It is drawn by hand below.
    oledDisplay.setFont(u8g2_font_5x7_tn);
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

        // Completion score, as a percentage of the cells visited.
        const int pctX = 58;
        const int pctBaseline = 63;

        oledDisplay.setCursor(pctX, pctBaseline);
        oledDisplay.print(maze.getCompletionPercent());

        // The '%' sign, drawn rather than typed - see the font note above.
        // Cap height is 6 and the baseline is 63, so the digits occupy rows
        // 58..63 and this sits in the same band: dot top-left, dot
        // bottom-right, diagonal between them.
        //
        // drawPixel and drawLine are both already linked for the maze grid, so
        // the whole glyph costs a handful of call sites rather than a font.
        const int pctSignX = oledDisplay.getCursorX() + 1;
        const int pctTop = pctBaseline - 5;

        oledDisplay.drawPixel(pctSignX, pctTop);
        oledDisplay.drawPixel(pctSignX + 4, pctTop + 5);
        oledDisplay.drawLine(pctSignX + 4, pctTop, pctSignX, pctTop + 5);

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