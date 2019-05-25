#pragma once

#include <unistd.h>

#include "frc/WPILib.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace frc;

namespace frc973 {

/**
 * Interface that continually checks the gyro and serves that data out
 */
class SPIGyro {
public:
    /**
     * Constructor initializes gyroscope, starts a thread to continually
     * update values, and then returns.
     */
    SPIGyro();

    /**
     * Returns the latest angle reading from the gyro.
     * @return The latest angle reading.
     */
    double GetDegrees();

    /**
     * Returns the latest angular momentum reading from the gyro.
     * @return The latest angular momentum reading.
     */
    double GetDegreesPerSec();

    /**
     * Sets the current gyro heading to zero
     */
    void Reset();

    /**
     * Zero the gyroscope by averaging all readings over 2 seconds. If a lot of
     * those ended up in error, at least make sure we have ~.5 seconds worth of
     * data before continuing.
     */
    void ZeroAngle();

    /**
     * Notify gyro to start shutdown sequence soon.
     */
    void Quit();

private:
    /**
     * Initializes and zeros the gyro, then starts collecting angular data
     * to be served out by this object.  Should be run in its own thread
     * (started by constructor).  Doesn't stop until someone calls Quit.
     */
    static void *Run(void *p);

    /**
     * Runs the recommended gyro startup procedure including checking all
     * of the self-test bits.
     * Returns true on success.
     */
    bool InitializeGyro();

    /**
     * Collects all the anglular momentum readings from the last 6 seconds
     * so that when the user calls ZeroAngle (on autoInit) it has the data
     * to average.
     */
    void CollectZeroData();

    /**
     * Pulls the angular rate from the gyro, checks for errors, and then
     * updates this object to serve out that data.
     */
    void UpdateReading();

    /*
     * Gets a reading from the gyro.
     * Returns a value to be passed to the Extract* methods or 0 for error.
     */
    uint32_t GetReading();

    /**
     * Reads from the gyro's internal memory and returns the value.
     * Retries until it succeeds.
     */
    uint16_t DoRead(uint8_t address);

    /**
     * Returns all of the non-data bits in the "header" except the parity from
     * value.
     */
    uint8_t ExtractStatus(uint32_t value) {
        return (value >> 26) & ~4;
    }

    /**
     * Checks for erros in the passed int.
     * Returns true if there was an error in the reading, false otherwise.
     * Prints messages to explain possible errors.
     * |res| should be the result of calling gyro.GetReading()
     */
    bool CheckErrors(uint32_t res);

    /**
     * Returns all of the error bits in the "footer" from value.
     */
    uint8_t ExtractErrors(uint32_t value) {
        return (value >> 1) & 0x7F;
    }

    /**
     * Returns the anglular rate contained in value.
     */
    double ExtractAngle(uint32_t value);

    /**
     * Performs a transaction with the gyro.
     * to_write is the value to write. This function handles setting the
     * checksum bit. result is where to stick the result. This function verifies
     * the parity bit. Returns true for success.
     */
    bool DoTransaction(uint32_t to_write, uint32_t *result);

    /**
     * Returns the part ID from the gyro.
     * Retries until it succeeds.
     */
    uint32_t ReadPartID();

    // Readings per second.
    static const int kReadingRate = 200;

    pthread_mutex_t mutex;
    SPI *gyro;
    Timer timer;
    double angle;
    double angularMomentum;
    long timeLastUpdate;
    bool run_;
    double zero_offset;  // used for zeroing the gyro

    unsigned int zeroing_points_collected;
    static const unsigned int zero_data_buffer_size = 6 * kReadingRate;
    double zeroing_data[zero_data_buffer_size];
};
}
