#pragma once

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
     * Construct a GreyTalonSRX to wrap over a TalonSRX.
     * @param canId The TalonSRX's CAN ID.
     */
    GreyTalonSRX(int canId)
            : BaseMotorController(canId | 0x02040000), TalonSRX(canId) {
        FactoryReset(this);
    }
    virtual ~GreyTalonSRX() {
    }

    /**
     * Configure P, I, D, and F for a specifc GreyTalonSRX.
     * @param kSlotIdx The slot ID.
     * @param kP The proportional value.
     * @param kI The integral value.
     * @param kD The derivative value.
     * @param kF The feed-forward value.
     * @param kTimeoutMs The timeout value in milliseconds.
     * @return The configured GreyTalonSRX.
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
     * Factory reset a specific TalonSRX.
     * @param motor The TalonSRX to reset.
     * @return The resetted TalonSRX.
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

/**
 * Interface for the GreyVictorSPX wrapper to be used on VictorSPX's.
 */
class GreyVictorSPX
        : public virtual BaseMotorController
        , public virtual VictorSPX {
public:
    /**
     * Construct a GreyVictorSPX to wrap over a VictorSPX.
     * @param canId The VictorSPX's CAN ID.
     */
    GreyVictorSPX(int canId)
            : BaseMotorController(canId | 0x02040000), VictorSPX(canId) {
        FactoryReset(this);
    }
    virtual ~GreyVictorSPX() {
    }

    /**
     * Configure P, I, D, and F for a specifc GreyVictorSPX.
     * @param kSlotIdx The slot ID.
     * @param kP The proportional value.
     * @param kI The integral value.
     * @param kD The derivative value.
     * @param kF The feed-forward value.
     * @param kTimeoutMs The timeout value in milliseconds.
     * @return The configured GreyVictorSPX.
     */

    GreyVictorSPX* Config_PID(int kSlotIdx, double kP, double kI, double kD,
                              double kF, int kTimeoutMs) {
        this->Config_kP(kSlotIdx, kP, kTimeoutMs);
        this->Config_kI(kSlotIdx, kI, kTimeoutMs);
        this->Config_kD(kSlotIdx, kD, kTimeoutMs);
        this->Config_kF(kSlotIdx, kF, kTimeoutMs);

        return this;
    }

    /**
     * Factory reset a specific VictorSPX.
     * @param motor The VictorSPX to reset.
     * @return The resetted VictorSPX.
     */
    VictorSPX* FactoryReset(VictorSPX* motor) {
        motor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,
                                            10);  // 0 = Not cascaded PID Loop;
                                                  // 10 = in constructor, not in
                                                  // a loop
        motor->SetSensorPhase(false);
        motor->SetInverted(false);
        motor->SetNeutralMode(NeutralMode::Coast);

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
