/*
 * AsynchLogCell.h
 *
 * Defines an asynchronous extension to the LogCell class.
 * When AsynchLogCell.GetContent is called, the AsynchLogCell
 * calls a callback to generate that content.
 *
 * This is in contrast with the standard LogCell which returns
 * whatever was last written to it.
 *
 *  Created on: Jan 1, 2015
 *      Author: Andrew
 */

#pragma once

#include "lib/logging/LogSpreadsheet.h"

namespace frc973 {

class AsynchLogCellListener;

/**
 * AsynchLogCell works like a LogCell except that it takes a reference to a
 * listener whom it will notify when it's content is requested.
 */
class AsynchLogCell : public LogCell {
public:
    /**
     * Construct an AsynchLogCell. When GetContent is called,
     * listener->NofityAsynchLogCellListener(this) gets called to generate the
     * content to return.
     * @param name The name of the column to log.
     * @param listener The listener to notify when content is requested.
     * @param size The cell size.
     */
    AsynchLogCell(char *name, AsynchLogCellListener *listener,
                  unsigned int size = DEFAULT_MAX_LOG_CELL_SIZE);

    /**
     * Call listener->NotifyAsynchLogCellListener(this) to get the contents to
     * be logged and return those contents.
     * @return The contents to be logged.
     */
    virtual const char *GetContent();

private:
    AsynchLogCellListener *m_listener;
};

/**
 * AsynchLogCellListener is an interface describing a class that is capable of
 * responding to AsynchLogCell notifications.  The listener should, given a
 * reference to the cell creating the notification, call cell->LogText (or
 * something like) that to fill the given cell with whatever content is needed.
 */
class AsynchLogCellListener {
public:
    virtual ~AsynchLogCellListener() {
    }

    /**
     * Handle an AsynchLogCell's request for data
     * @param cell The cell requesting data.
     */
    virtual void NotifyAsynchLogCellListener(AsynchLogCell *cell) = 0;
};
}
