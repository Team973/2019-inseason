#pragma once

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"

namespace frc973 {

/**
 * Interface for the GreyTalonSRX wrapper to be used on TalonSRX's.
 */
class GreyTalonSRX
        : public virtual BaseMotorController
        , public virtual TalonSRX {
public:
    /**
     * Construct a GreyTalonSRX to wrap over a Talon SRX.
     * @param canId The Talon SRX's CAN id.
     */
    GreyTalonSRX(int canId)
            : BaseMotorController(canId | 0x02040000), TalonSRX(canId) {
        FactoryReset(this);
    }
    virtual ~GreyTalonSRX() {
    }

    /**
     * Configurate F, P, I, and D for a specifc Talon SRX
     * @param motor The Talon SRX to configure
     * @param kSlotIdx Slot Id
     * @param kP Proportional value
     * @param kI Integral value
     * @param kD Derivative value
     * @param kF Feed-Forward value
     * @param kTimeoutMs Timeout value in milliseconds
     * @return The configured motor
     */

    GreyTalonSRX* Config_PID(int kSlotIdx, double kP, double kI, double kD,
                             double kF, int kTimeoutMs) {
        this->Config_kP(kSlotIdx, kP, kTimeoutMs);
        this->Config_kI(kSlotIdx, kI, kTimeoutMs);
        this->Config_kD(kSlotIdx, kD, kTimeoutMs);
        this->Config_kF(kSlotIdx, kF, kTimeoutMs);

        return this;
    }

    /**
     * Factory reset a specific Talon SRX.
     * @param motor The Talon SRX to reset.
     * @return The resetted Talon SRX.
     */
    TalonSRX* FactoryReset(TalonSRX* motor) {
        motor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,
                                            10);  // 0 = Not cascaded PID Loop;
                                                  // 10 = in constructor, not in
                                                  // a loop
        motor->SetSensorPhase(false);
        motor->SetInverted(false);
        motor->SetNeutralMode(NeutralMode::Coast);
        motor->GetSensorCollection().SetQuadraturePosition(0, 0);

        motor->ConfigNominalOutputForward(0.0, 10);
        motor->ConfigNominalOutputReverse(0.0, 10);
        motor->ConfigPeakOutputForward(1.0, 10);
        motor->ConfigPeakOutputReverse(-1.0, 10);
        motor->ConfigNeutralDeadband(0.04, 10);
        motor->ConfigOpenloopRamp(0, 10);
        motor->ConfigClosedloopRamp(0, 10);

        // Gains
        motor->Config_kP(0, 0.0, 10);
        motor->Config_kI(0, 0.0, 10);
        motor->Config_kD(0, 0.0, 10);
        motor->Config_kF(0, 0.0, 10);
        motor->ConfigMotionCruiseVelocity(0.0, 10);
        motor->ConfigMotionAcceleration(0.0, 10);
        motor->SelectProfileSlot(0, 0);

        // Limiting
        motor->EnableCurrentLimit(false);
        motor->ConfigPeakCurrentDuration(0, 10);
        motor->ConfigPeakCurrentLimit(0, 10);
        motor->ConfigContinuousCurrentLimit(0, 10);
        motor->EnableVoltageCompensation(false);
        motor->ConfigVoltageCompSaturation(12, 10);
        motor->ConfigForwardSoftLimitThreshold(10000, 10);
        motor->ConfigReverseSoftLimitThreshold(10000, 10);
        motor->ConfigForwardSoftLimitEnable(false, 10);
        motor->ConfigReverseSoftLimitEnable(false, 10);

        motor->Set(ControlMode::PercentOutput, 0.0);

        return motor;
    }
};
}
