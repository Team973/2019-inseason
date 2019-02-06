/*
 * GameMode.h
 *
 *  Created on: February 5, 2019
 *      Authors: John P, Luis, Dillan
 */
#pragma once

#include "frc/WPILib.h"

using namespace frc;

namespace frc973 {

/*
 * GameMode represents the robots scoring mode. You can't be doing Cargo and
 * Hatch at the same time, so it is in one or the other for example.
 */
enum class GameMode
{
    Cargo,
    Hatch,
    EndGame,
};
}
