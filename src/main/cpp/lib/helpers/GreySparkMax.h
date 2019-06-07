#pragma once

#include "rev/CANSparkMax.h"

using namespace rev;

namespace frc973 {

/**
 * Interface for the GreySparkMax wrapper to be used on CANSparkMax's.
 */
class GreySparkMax : public virtual CANSparkMax {
public:
    /**
     * Construct a GreySparkMax to wrap over a CANSparkMax.
     * @param canId The CANSparkMax's CAN ID.
     * @param motorType The MotorType connected.
     */
    GreySparkMax(int canId, CANSparkMax::MotorType motorType)
            : CANSparkMax(canId, motorType) {
        RestoreFactoryDefaults();
    }
    virtual ~GreySparkMax() {
    }

    /**
     * Configure P, I, D, and F for a specifc CANSparkMax.
     * @param kSlotIdx The slot ID.
     * @param kP The proportional value.
     * @param kI The integral value.
     * @param kD The derivative value.
     * @param kF The feed-forward value.
     * @return The configured motor.
     */

    GreySparkMax* Config_PID(int kSlotIdx, double kP, double kI, double kD,
                             double kF) {
        this->GetPIDController().SetP(kP, kSlotIdx);
        this->GetPIDController().SetI(kI, kSlotIdx);
        this->GetPIDController().SetD(kD, kSlotIdx);
        this->GetPIDController().SetFF(kF, kSlotIdx);

        return this;
    }
};
}
