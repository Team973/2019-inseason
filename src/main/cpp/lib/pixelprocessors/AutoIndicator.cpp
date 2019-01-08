/*
 * AutoIndicator.cpp
 *
 *  Created on: March 11, 2018
 *      Author: Cole Brinsfield
 */

#include "AutoIndicator.h"
#include <iostream>
namespace LightPattern {

AutoIndicator::AutoIndicator() {
    m_gameData = "";
}
void AutoIndicator::SetData(std::string gameDataStr) {
    m_gameData = gameDataStr;
}

void AutoIndicator::Tick(PixelState& state) {
    int column_width = state.numLEDs / 3;
    if (m_gameData.size() == 3) {
        switch (m_gameData[0]) {
            case 'L':
                std::fill(state.pixels.begin(),
                          state.pixels.end() - column_width * 2, LEFT_GREEN);
                break;
            case 'R':
                std::fill(state.pixels.begin(),
                          state.pixels.end() - column_width * 2, RIGHT_WHITE);
                break;
            default:
                std::fill(state.pixels.begin(),
                          state.pixels.end() - column_width * 2,
                          DEFAULT_FILL_RED);
        }
        switch (m_gameData[1]) {
            case 'L':
                std::fill(state.pixels.begin() + column_width,
                          state.pixels.end() - column_width, LEFT_GREEN);
                break;
            case 'R':
                std::fill(state.pixels.begin() + column_width,
                          state.pixels.end() - column_width, RIGHT_WHITE);
                break;
            default:
                std::fill(state.pixels.begin() + column_width,
                          state.pixels.end() - column_width, DEFAULT_FILL_RED);
        }
        switch (m_gameData[2]) {
            case 'L':
                std::fill(state.pixels.begin() + column_width * 2,
                          state.pixels.end(), LEFT_GREEN);
                break;
            case 'R':
                std::fill(state.pixels.begin() + column_width * 2,
                          state.pixels.end(), RIGHT_WHITE);
                break;
            default:
                std::fill(state.pixels.begin() + column_width * 2,
                          state.pixels.end(), DEFAULT_FILL_RED);
        }
    }
    else {
        std::fill(state.pixels.begin(), state.pixels.end(), NO_MESSAGE_BLUE);
    }
}
}
