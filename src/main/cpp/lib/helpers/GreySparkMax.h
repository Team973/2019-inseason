#pragma once

#include "frc/WPILib.h"
#include "rev/CANSparkMax.h"

using namespace rev;

namespace frc973 {

/**
 * Interface for the GreySparkMax wrapper to be used on SPARK MAX's.
 */
class GreySparkMax : public virtual CANSparkMax {
public:
    /**
     * Construct a GreySparkMax to wrap over a SPARK MAX.
     * @param canId The SPARK MAX's CAN id.
     */
    GreySparkMax(int canId, CANSparkMax::MotorType motorType)
            : CANSparkMax(canId, motorType) {
        RestoreFactoryDefaults();
    }
    virtual ~GreySparkMax() {
    }

    /**
     * Configurate F, P, I, and D for a specifc SPARK MAX
     * @param motor The SPARK MAX to configure
     * @param kSlotIdx Slot Id
     * @param kP Proportional value
     * @param kI Integral value
     * @param kD Derivative value
     * @param kF Feed-Forward value
     * @param kTimeoutMs Timeout value in milliseconds
     * @return The configured motor
     */

    GreySparkMax* Config_PID(int kSlotIdx, double kP, double kI, double kD,
                             double kFF) {
        this->GetPIDController().SetP(kP, kSlotIdx);
        this->GetPIDController().SetI(kI, kSlotIdx);
        this->GetPIDController().SetD(kD, kSlotIdx);
        this->GetPIDController().SetFF(kFF, kSlotIdx);

        return this;
    }
};
}
