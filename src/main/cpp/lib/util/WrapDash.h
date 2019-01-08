/*
 * WrapDash.h
 *
 * Quick wrapper of wpilib's debug string interface
 *
 *  Created on: Sep 3, 2015
 *      Author: Andrew
 */

#pragma once

#include "frc/WPILib.h"

using namespace frc;

namespace frc973 {

/**
 * Defines dashboard positions for data.
 */
enum DBStringPos
{
    DB_LINE0, /**< First column, first row. */
    DB_LINE1, /**< First column, second row. */
    DB_LINE2, /**< First column, third row. */
    DB_LINE3, /**< First column, fourth row. */
    DB_LINE4, /**< First column, fifth row. */
    DB_LINE5, /**< Second column, first row. */
    DB_LINE6, /**< Second column, second row. */
    DB_LINE7, /**< Second column, third row. */
    DB_LINE8, /**< Second column, fourth row. */
    DB_LINE9  /**< Second column, fifth row. */
};

/**
 * Use printf-like syntax to print to the smart dash debug string place.
 * @param position The position to output to.
 * @param formatstring The printf syntax to output.
 * @param ... Inputs to the formatsrting.
 */
void DBStringPrintf(DBStringPos position, const char *formatstring, ...);

/**
 * Converts the match type to a string.
 * @return The converted string.
 */
const char *MatchTypeToString(frc::DriverStation::MatchType matchType);
}
