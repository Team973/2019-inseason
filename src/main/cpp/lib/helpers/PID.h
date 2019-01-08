/*
 * RobotInfo.h
 *
 * Author: Oliver Curry
 *
 * Almost identical to the pid module Oliver wrote... style updated
 * to match the new style standard.
 *
 * For velocity control, pass the PID_SPEED_CTRL flag in the |flags|
 * param of the constructor. This object will automatically integrate
 * output as long as you make sure to integrate your input.
 */

#pragma once

#include <math.h>
#include <stdint.h>

namespace frc973 {

constexpr uint32_t PID_SPEED_CTRL =
    0x00000001; /**< Integrate the output of this PID */

/**
 * Interface for using PID.
 */
class PID {
public:
    /**
     * Initialize a PID object by passing P, I, and D constants
     * @param Kp The P constant.
     * @param Ki The I constant.
     * @param Kd The D constant.
     * @param flags 32-bitstring representing any flags (defined above).
     */
    PID(double Kp = 0.0, double Ki = 0.0, double Kd = 0.0, uint32_t flags = 0);

    /**
     * Initialize a PID object by passing an array of 3 elements; the first
     * element holds Kp, the second element holds Ki, and the third element
     * holds Kd.
     * @param gains Array of 3 doubles where each element represents Kp, Ki, and
     * Kd.
     * @param flags 32-bitstring representing any flags (defined above).
     */
    PID(double gains[3], uint32_t flags = 0);

    /**
     * Update the PID gains by passing P, I, and D constants.
     * @param Kp The P constant.
     * @param Ki The I constant.
     * @param Kd The D constant.
     */
    void SetGains(double Kp = 0.0, double Ki = 0.0, double Kd = 0.0);

    /**
     * Update PID constants by passing an array of 3 elements; the first element
     * holds Kp, the second element holds Ki, and the third element holds Kd.
     * @param gains Array of 3 doubles where each element represents Kp, Ki, and
     * Kd.
     */
    void SetGains(double gains[3]);

    /**
     * Set the target for the PID controller.
     * @param target The target the PID controler will try to achieve.
     */
    void SetTarget(double target);

    /**
     * Get the current target of the PID controller.
     * @return The current target that the pid controller is trying to achieve.
     */
    double GetTarget();

    /**
     * Reset the PID controller (forget any stateful information). This could be
     * helpful when setting a new, unrelated setpoint.
     * @param currPosition The setpoint to reset to.
     */
    void Reset(double currPosition = NAN);

    /**
     * Set a cap on the integral term of PID. This cap will be in the units of
     * error*sec, whatever that means to you.  This cap works in absolute terms;
     * so the integral of error will be bound between positive |icap| and
     * negative |icap|.
     * @param icap The maximum integral of error that may occur in the units of
     * error*sec.
     */
    void SetICap(double icap);

    /**
     * Set output bound. By default, these bounds are -1.0 and 1.0.
     * @param min The minimum bound.
     * @param max The maximum bound.
     */
    void SetBounds(double min, double max);

    /**
     * Calculate the output of the PID controller. This form uses the FPGA to
     * determine current time and compares it to the time the controller was
     * last called at.
     * @param actual The current position/speed/state of the system being
     * controlled.
     * @return Calculated ideal value to send to output device to achieve the
     * desired target.
     */
    double CalcOutput(double actual);

    /**
     * Calculate the output of the PID controller. This form uses the |time|
     * parameter (unit in milliseconds) to determine time and compares it to the
     * time the controller was last called with.
     * @param actual The current position/speed/state of the system being
     * controlled.
     * @param timeMs The current time in milliseconds.
     * @return Calculated ideal value to send to output device to achieve the
     * desired target.
     */
    double CalcOutput(double actual, uint32_t timeMs);

    /**
     * Calculate the output with error of the PID controller.
     * @param error The current error.
     * @return Calculated output with error.
     */
    double CalcOutputWithError(double error);

    /**
     * Calculate the output with error of the PID controller.
     * @param error The current error.
     * @param timeMs The current time in milliseconds.
     * @return Calculated output with error.
     */
    double CalcOutputWithError(double error, uint32_t timeMs);

    /**
     * Get the previous output of the PID controller.
     * @return Previously calculated PID output value.
     */
    double GetPrevOutput();

    /**
     * Set the previous output of the PID controller. This can be useful for
     * setting the start speed for speed control.
     * @param prev The output (feed forward) value.
     */
    void SetPrevOutput(double prev);

private:
    double m_Kp;
    double m_Ki;
    double m_Kd;

    double m_target;
    double m_min;
    double m_max;

    double m_timeLastUpdateSec;
    double m_prevPos;
    double m_prevErr;
    double m_integral;
    double m_icap;

    uint32_t m_flags;
    double m_lastOutput;
};
}
