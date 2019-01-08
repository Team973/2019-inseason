/**
 * Limelight.h
 *
 * Authors: Kyle, Chris M.
 */
#pragma once

#include "frc/WPILib.h"
#include "lib/util/WrapDash.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"

using namespace frc;

namespace frc973 {

class Limelight {
public:
    /**
     * Constuct a Limelight Camera.
     */
    Limelight();
    virtual ~Limelight();

    /**
     * Limelight LED modes.
     */
    enum class LightMode
    {
        on,   /**< The light mode for turned on LEDs. */
        off,  /**< The light mode for turned off LEDs. */
        blink /**< The light mode for blinking LEDs. */
    };

    /**
     * Limelight camera modes
     */
    enum class CameraMode
    {
        onVision, /**< The vision camera mode that uses tuned settings for
                     autonomous. */
        onDriver  /**< The driver camera mode that uses tuned settings for
                     teleop. */
    };

    /**
     * Sets the limelight's LEDs to on, off, or blink
     * @param mode the LED mode
     */
    void SetLightMode(LightMode mode);

    /**
     * Sets the limelight's camera mode to use vision or driver version where
     * camera feedback changes
     * @param mode the camera mode
     */
    void SetCameraMode(CameraMode mode);

    /**
     * Sets the limelight's LEDs to on
     */
    void SetLightOn();

    /**
     * Sets the limelight's LEDs to off
     */
    void SetLightOff();

    /**
     * Sets the limelight's LEDs to blinking
     */
    void SetLightBlink();

    /**
     * Sets the limelight's camera to use vision settings
     */
    void SetCameraVision();

    /**
     * Sets the limelight's cameras to use driver settings (just raw feedback)
     */
    void SetCameraDriver();

    /**
     * Checks if target is present or not
     * @return target is seen or not
     */
    bool isTargetValid();

    /**
     * Gets the target x-offset
     * @return target x-offset
     */
    double GetXOffset();

    /**
     * Gets the target y-offset
     * @return target y-offset
     */
    double GetYOffset();

    /**
     * Gets the target area
     * @return target area
     */
    double GetTargetArea();

    /**
     * Gets the target skew
     * @return target skew
     */
    double GetTargetSkew();

    /**
     * Gets the camera's latency
     * @return camera latency
     */
    double GetLatency();

private:
    std::shared_ptr<NetworkTable> m_limelight;  // constructs the limelight

    LightMode m_lightMode;    // enum for LED state
    CameraMode m_cameraMode;  // enum for camera state

    bool m_targetStatus;  // target is seen or not [0,1]

    double m_horizontalOffset;  // horizontal target offset in degrees [-27, 27]
    double m_verticalOffset;  // vertical target offset in degrees [-20.5, 20.5]
    double m_targetArea;      // target area in percentage of image [0, 100]
    double m_targetSkew;      // target skew or rotation [-90, 0]
    double m_latency;  // The pipeline’s latency contribution (ms) Add at least
                       // 11ms for image capture latency.
};
}
