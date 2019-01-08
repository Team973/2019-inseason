/*
 * LogSpreadsheetBase.h
 *
 * Base class for an object that writes a spreadsheet.
 *
 * Define a subclass to use this.  Creates a row every time TaskPostPeriodic
 * is called, so if you wanted you could create a TaskManager and log at
 * whatever frequency you want.
 *
 *  Created on: Nov 24, 2015
 *      Author: Andrew
 */

#pragma once

#include <iostream>
#include <fstream>
#include <vector>

#include "lib/managers/TaskMgr.h"
#include "lib/managers/CoopTask.h"
#include <pthread.h>

namespace frc973 {

constexpr uint32_t DEFAULT_MAX_LOG_CELL_SIZE =
    32; /**< The default maximum cell size. */

constexpr uint32_t LOG_CELL_FLAG_CLEAR_ON_READ =
    1; /**< Clear the cell on read or not. */

/**
 * Represents a column in the spreadsheet.  For the column to be printed, you
 * must register the instance of a LogCell in the LogSpreadsheet using
 * LogSpreadsheet.RegisterCell.
 */
class LogCell {
public:
    /**
     * Instantiate a LogCell passing in the name of the column and the maximum
     * length of the field (default 32 chars).
     * @param name The name of the column.
     * @param size The maximum length of the cell.
     * @param flags The flags to pass.
     */
    explicit LogCell(const char *name,
                     uint32_t size = DEFAULT_MAX_LOG_CELL_SIZE,
                     uint32_t flags = 0);
    virtual ~LogCell();

    /**
     * Log a String (C-string).
     * @param text The text to be put in the cell.
     */
    void LogText(const char *text);

    /**
     * Log an integer.
     * @param val The integer to be logged in the cell.
     */
    void LogInt(int val);

    /**
     * Log a double.
     * @param val The double to be logged in the cell.
     */
    void LogDouble(double val);

    /**
     * Log a String using printf-style syntax.
     * @param formatstr containing text and printf-style %directives.
     * @param ... The list of arguments to be printed.
     */
    void LogPrintf(const char *formatstr, ...);

    /**
     * Return the name of this cell.
     * @return The name of the cell.
     */
    const char *GetName();

    /**
     * Get the contents of the string.
     * @return The content.
     */
    virtual const char *GetContent();

    /**
     * Clear the cell so its contents are empty.
     */
    void ClearCell();

    /**
     * Check whether the cell should be cleared after being read.
     * @return Whether the cell should be cleared after being logged
     */
    bool ClearOnRead() {
        return m_flags & LOG_CELL_FLAG_CLEAR_ON_READ;
    }

    /**
     * Aquire a lock on the spreadsheet.
     */
    void AcquireLock() {
        pthread_mutex_lock(&m_mutex);
    }

    /**
     * Release the lock.
     */
    void ReleaseLock() {
        pthread_mutex_unlock(&m_mutex);
    }

protected:
    char *m_buffer; /**< Buffer to write to. */

private:
    const char *m_name;
    const int m_buffSize;
    const uint32_t m_flags;
    pthread_mutex_t m_mutex;
};

/**
 * A LogSpreadsheet writes to a file the contents of each of its registered
 * cells.
 */
class LogSpreadsheet : public CoopTask {
public:
    /**
     * Create a new LogSpreadsheetBase... we need the scheduler that will be
     * calling us.
     * @param scheduler The Task Manager to register this with.
     */
    explicit LogSpreadsheet(TaskMgr *scheduler);
    virtual ~LogSpreadsheet();

    /**
     * Initialize the table... open the file, write the column headers, etc.
     */
    void Start();

    /**
     * Called regularly. If we are initialized, write a row of the table.
     * @param mode The current operating mode of the robot.
     */
    void TaskPostPeriodic(RobotMode mode) override;

    /**
     * Register a new column to be logged.  You will figure this value out
     * in GetValue.  You cannot register more columns after the header of
     * the file has already been written.
     * @param cell The cell to register.
     */
    void RegisterCell(LogCell *cell);

private:
    void WriteRow();

    std::vector<LogCell *> m_cells;
    std::ofstream *m_oFile;
    TaskMgr *m_scheduler;
    bool m_initialized;
    RobotMode m_mode;
};
}
