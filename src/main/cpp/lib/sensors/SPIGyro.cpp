#ifndef GYRO_CPP
#define GYRO_CPP

#include "lib/sensors/SPIGyro.h"

namespace frc973 {

/**
 * Some nifty byte swapping.
 */
uint32_t swapTheBytes(uint32_t dword) {
    return ((dword >> 24) & 0x000000FF) | ((dword >> 8) & 0x0000FF00) |
           ((dword << 8) & 0x00FF0000) | ((dword << 24) & 0xFF000000);
}

/*
 * Constructor initializes gyroscope, starts a thread to continually
 * update values, and then returns.
 */
SPIGyro::SPIGyro()
        : mutex(PTHREAD_MUTEX_INITIALIZER)
        , gyro(new SPI(SPI::kOnboardCS0))
        , timer() {
    run_ = true;

    // The gyro goes up to 8.08MHz.
    // The myRIO goes up to 4MHz, so the roboRIO probably does too.
    gyro->SetClockRate(4e6);
    gyro->SetChipSelectActiveLow();
    gyro->SetClockActiveHigh();
    gyro->SetSampleDataOnLeadingEdge();
    gyro->SetMSBFirst();

    zeroing_points_collected = 0;
    zero_offset = 0;

    fprintf(stderr, "Starting gyro thread\n");
    pthread_t updateThread;
    pthread_create(&updateThread, NULL, Run, this);
    fprintf(stderr, "Started gyro thread\n");
}

/*
 * Returns the latest angle reading from the gyro.
 */
double SPIGyro::GetDegrees() {
    pthread_mutex_lock(&mutex);
    double ret = angle;
    pthread_mutex_unlock(&mutex);
    return ret;
}

/*
 * Returns the the latest angular momentum reading from the gyro.
 */
double SPIGyro::GetDegreesPerSec() {
    pthread_mutex_lock(&mutex);
    double ret = angularMomentum;
    pthread_mutex_unlock(&mutex);
    return ret;
}

/*
 * Sets the current gyro heading to zero
 */
void SPIGyro::Reset() {
    pthread_mutex_lock(&mutex);
    angle = 0;
    pthread_mutex_unlock(&mutex);
}

/*
 * Zero the gyroscope by averaging all readings over 2 seconds.
 * If a lot of those ended up in error, at least make sure we have ~.5
 * seconds worth of data before continuing.
 */
void SPIGyro::ZeroAngle() {
    unsigned int num_samples = zeroing_points_collected;
    if (zeroing_points_collected > zero_data_buffer_size)
        num_samples = zero_data_buffer_size;

    zero_offset = 0;
    for (unsigned int i = 0; i < num_samples; i++) {
        zero_offset -= zeroing_data[i];
    }
    zero_offset /= static_cast<double>(num_samples);

    pthread_mutex_lock(&mutex);
    angle = 0;
    pthread_mutex_unlock(&mutex);

    printf("Total zero offset: %f\n", zero_offset);
}

/*
 * Notify gyro to start shutdown sequence soon.
 */
void SPIGyro::Quit() {
    run_ = false;
}

/*
 * Runs closed loop; grabs angle values from gyro and saves them so
 * they can be returned by GetDegrees.  Should be run in its own thread
 * (started by constructor).  Doesn't stop until someone calls Quit.
 */
void *SPIGyro::Run(void *p) {
    SPIGyro *inst = (SPIGyro *)p;

    inst->timer.Reset();
    inst->timer.Start();

    usleep(100000);

    while (inst->run_ && !inst->InitializeGyro()) {
        usleep(500000);
        printf("retrying initialization\n");
    }
    printf("The Gyro has been successfully initialized\n");

    printf("Gyro ID: %u\n", inst->ReadPartID());

    int cyc = 0;

    while (inst->run_) {
        inst->timer.Reset();
        inst->timer.Start();

        inst->CollectZeroData();

        if (inst->zeroing_points_collected == inst->zero_data_buffer_size)
            inst->ZeroAngle();

        inst->UpdateReading();

        if (cyc++ == 50) {
            cyc = 0;
            // SmartDashboard::PutNumber("Gyro angle: ", inst->GetDegrees());
            // printf("angle is %f, momentum is %f\n", inst->GetDegrees(),
            // inst->GetDegreesPerSec());
        }

        // Wait till the next 1/kReadingRate us period to make next reading
        usleep((unsigned int)(((static_cast<double>(1.0) /
                                static_cast<double>(inst->kReadingRate)) -
                               inst->timer.Get()) *
                              static_cast<double>(1e6)));
    }
    return NULL;
}

/*
 * Runs the recommended gyro startup procedure including checking all
 * of the self-test bits.
 * Returns true on success.
 */
bool SPIGyro::InitializeGyro() {
    uint32_t result;
    if (!DoTransaction(0x20000003, &result)) {
        printf("Failed to communicate with gyro (pre-self-check)\n");
        return false;
    }
    if (result != 1) {
        // We might have hit a parity error or something and are now retrying,
        // so this isn't a very big deal.
        printf("gyro unexpected initial response 0x%X\n", result);
    }

    // Wait 50ms for it to assert the fault conditions before reading them.
    usleep(50000);

    if (!DoTransaction(0x20000000, &result)) {
        printf("failed to clear latched non-fault data\n");
        return false;
    }
    printf("gyro dummy response is 0x%X\n", result);

    if (!DoTransaction(0x20000000, &result)) {
        printf("failed to read self-test data\n");
        return false;
    }
    if (ExtractStatus(result) != 2) {
        printf("gyro first value 0x%X not self-test data\n", result);
        return false;
    }
    if (ExtractErrors(result) != 0x7F) {
        printf("gyro first value 0x%X does not have all errors\n", result);
        return false;
    }
    printf("Value was test data yay!\n");

    if (!DoTransaction(0x20000000, &result)) {
        printf("failed to clear latched self-test data\n");
        return false;
    }
    if (ExtractStatus(result) != 2) {
        printf("gyro second value 0x%X not self-test data\n", result);
        return false;
    }
    printf("Second value was test data again yay!\n");

    return true;
}

/*
 * Collects all the anglular momentum readings from the last 6 seconds
 * so that when the user calls ZeroAngle (on autoInit) it has the data
 * to average.
 */
void SPIGyro::CollectZeroData() {
    static unsigned int zeroing_index = 0;

    const uint32_t result = GetReading();
    if (CheckErrors(result) == false) {
        const double new_angle =
            ExtractAngle(result) / static_cast<double>(kReadingRate);
        zeroing_data[zeroing_index] = new_angle;
        zeroing_points_collected++;
    }

    if (++zeroing_index > zero_data_buffer_size)
        zeroing_index = 0;
}

/*
 * Pulls the angular rate from the gyro, checks for errors, and then
 * updates this object to serve out that data.
 */
void SPIGyro::UpdateReading() {
    static int startup_cycles_left = 2 * kReadingRate;
    // static uint64_t lastCall;

    const uint32_t result = GetReading();
    if (startup_cycles_left > 0) {
        startup_cycles_left--;
        // lastCall = GetUsecTime();
        return;
    }
    if (CheckErrors(result) == false) {
        // uint64_t now = GetUsecTime();
        // uint64_t diff = now - lastCall;
        // double diffSec = ((double) (diff / 1000)) / 1000.0;

        double new_angle = ExtractAngle(result) / (double)kReadingRate;
        new_angle += zero_offset;

        pthread_mutex_lock(&mutex);
        angle += new_angle;
        angularMomentum = new_angle;

        // lastCall = now;
        pthread_mutex_unlock(&mutex);
    }
}

/*
 * Gets a reading from the gyro.
 * Returns a value to be passed to the Extract* methods or 0 for error.
 */
uint32_t SPIGyro::GetReading() {
    uint32_t result;
    if (!DoTransaction(0x20000000, &result)) {
        printf("Lost comm while trying to get reading\n");
        return 0;
    }
    return result;
}

/*
 * Reads from the gyro's internal memory and returns the value.
 * Retries until it succeeds.
 */
uint16_t SPIGyro::DoRead(uint8_t address) {
    const uint32_t command = (0x8 << 28) | (address << 17);
    uint32_t response;
    while (true) {
        if (!DoTransaction(command, &response)) {
            printf("reading 0x%X failed\n", address);
            continue;
        }
        if ((response & 0xEFE00000) != 0x4E000000) {
            printf("gyro read from 0x%X gave unexpected response 0x%X\n",
                   address, response);
            continue;
        }
        return (response >> 5) & 0xFFFF;
    }
}

/*
 * Checks for erros in the passed int.
 * Returns true if there was an error in the reading, false otherwise.
 * Prints messages to explain possible errors.
 * |res| should be the result of calling gyro.GetReading()
 */
bool SPIGyro::CheckErrors(uint32_t result) {
    if (result == 0) {
        printf("normal gyro read failed\n");
        return true;
    }
    switch (ExtractStatus(result)) {
        case 0:
            printf("gyro says data is bad: 0x%X -> 0x%X from %f\n", result,
                   ExtractStatus(result), ExtractAngle(result));
            return true;
        case 1:
            break;
        default:
            printf("gyro gave weird status 0x%X\n", ExtractStatus(result));
            return true;
    }
    if (ExtractErrors(result) != 0) {
        const uint8_t errors = ExtractErrors(result);
        if (errors & (1 << 6)) {
            printf("gyro gave PLL error\n");
        }
        if (errors & (1 << 5)) {
            printf("gyro gave quadrature error\n");
        }
        if (errors & (1 << 4)) {
            printf("gyro gave non-volatile memory error\n");
        }
        if (errors & (1 << 3)) {
            printf("gyro gave volatile memory error\n");
        }
        if (errors & (1 << 2)) {
            printf("gyro gave power error\n");
        }
        if (errors & (1 << 1)) {
            printf("gyro gave continuous self-test error\n");
        }
        if (errors & 1) {
            printf("gyro gave unexpected self-test mode\n");
        }
        return true;
    }
    return false;
}

/*
 * Returns the anglular rate contained in value.
 * unit degrees per sec
 */
double SPIGyro::ExtractAngle(uint32_t value) {
    const int16_t reading = -(int16_t)(value >> 10 & 0xFFFF);
    return static_cast<double>(reading) / 80.0;
}

/*
 * Performs a transaction with the gyro.
 * to_write is the value to write. This function handles setting the checksum
 * bit.
 * result is where to stick the result. This function verifies the parity bit.
 * Returns true for success.
 */
bool SPIGyro::DoTransaction(uint32_t to_write, uint32_t *result) {
    static const uint8_t kBytes = 4;
    static_assert(kBytes == sizeof(to_write),
                  "need the same number of bytes as sizeof(the data)");

    if (__builtin_parity(to_write & ~1) == 0)
        to_write |= 1;

    uint8_t to_send[kBytes], to_receive[kBytes];
    const uint32_t to_write_flipped = swapTheBytes(to_write);
    memcpy(to_send, &to_write_flipped, kBytes);

    switch (gyro->Transaction(to_send, to_receive, kBytes)) {
        case -1:
            printf("SPI::Transaction failed\n");
            return false;
        case kBytes:
            break;
        default:
            printf("SPI::Transaction returned something weird\n");
            break;
    }

    memcpy(result, to_receive, kBytes);
    if (__builtin_parity(*result & 0xFFFF) != 1) {
        printf("high byte parity failure\n");
        return false;
    }
    if (__builtin_parity(*result) != 1) {
        printf("whole value parity failure\n");
        return false;
    }

    *result = swapTheBytes(*result);
    return true;
}

/*
 * Returns the part ID from the gyro.
 * Retries until it succeeds.
 */
uint32_t SPIGyro::ReadPartID() {
    return (DoRead(0x0E) << 16) | DoRead(0x10);
}
}

#endif
