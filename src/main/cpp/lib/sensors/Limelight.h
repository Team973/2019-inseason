/**
 * Limelight.h
 *
 * Authors: Kyle, Chris M.
 */
#pragma once

#include "frc/WPILib.h"
#include <iostream>
#include "math.h"
#include "lib/util/WrapDash.h"
#include "lib/filters/BullshitFilter.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "src/info/RobotInfo.h"

using namespace frc;

namespace frc973 {

class Limelight {
public:
    /**
     * Constuct a Limelight Camera.
     * @param name The name of the limelight being constructed
     */
    Limelight(const char *name);
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
     *  Limelight stream modes.
     */
    enum class StreamMode
    {
        standard,    /**< Side by side Limelight/USB camera. */
        pipMain,     /**< Picture in picture w/ USB camera in lower right. */
        pipSecondary /**< Picture in picture w/ Limelight in lower right. */
    };

    /**
     *  Limelight snapshot modes.
     */
    enum class SnapshotMode
    {
        stop, /**< Stop taking snapshots. */
        start /**< Take snapshots. */
    };

    /**
     *  Limelight pipeline modes.
     */
    enum class PipelineMode
    {
        drive,          /**< The limelight drive pipeline */
        default_vision, /**< The limelight default vision pipeline */
        target_vision,  /**< The limelight target vision pipeline */
        off,            /**< The limelight off pipeline */
        vision3d        /**< The limelight 3d pipeline */
    };

    void SetPiPMain();

    void SetPiPSecondary();

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
     * Sets the limelight's pipeline index.
     * @param index The pipeline index.
     */
    void SetPipelineIndex(int index);

    /**
     * Set the pipeline mode.
     * @param mode The pipeline mode.
     */
    void SetPipeline(PipelineMode mode);

    /**
     * Sets the limelight's stream mode.
     * @param mode The stream mode.
     */
    void SetStreamMode(StreamMode mode);

    /**
     * Sets the limelight's snapshot mode.
     * @param mode The snapshot mode.
     */
    void SetSnapshotMode(SnapshotMode mode);

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
     * Sets the limelight's camera to use target vision settings
     */
    void SetCameraVision();

    /**
     * Sets the limelight's cameras to use driver settings (just raw feedback)
     */
    void SetCameraDriver();

    /**
     * Sets the limelight's camera to use default vision settings
     */
    void SetCameraDefaultVision();

    /**
     * Sets the limelight's camera to use off settings (disables as much
     * feedback as possible)
     */
    void SetCameraOff();

    /**
     * Sets the limelight's camera to use 3d processing vision settings
     */
    void SetCamera3D();

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

    /**
     * Gets the camera's pipeline index
     * @return camera pipeline index
     */
    double GetPipeline();

    /**
     * Gets the target's horizontal length
     * @return target's horizontal length
     */
    double GetHorizontalLength();

    /**
     * Gets the target's vertical length
     * @return target's vertical length
     */
    double GetVerticalLength();

    /**
     * Finds the angle skew of the robot in reference to the target
     * @return targets skew
     */
    double FindTargetSkew();

    /**
     * Finds the horizontal distance of the robot in reference to the target
     * @return The targest distance in inches
     */
    double GetHorizontalDistance();

    static constexpr double TARGET_HEIGHT = 24.25;  // in inches from ground
    static constexpr double CAMERA_HEIGHT = 4.75;   // in inches from ground
    static constexpr double CAMERA_ANGLE =
        40.0 * (Constants::PI / 180.0);  // in degrees wrt ground

private:
    std::shared_ptr<NetworkTable> m_limelight;  // constructs the limelight
    const char *m_camName;

    LightMode m_lightMode;        // enum for LED state
    CameraMode m_cameraMode;      // enum for camera state
    StreamMode m_streamMode;      // enum for stream state
    SnapshotMode m_snapshotMode;  // enum for snapshot state
    PipelineMode m_pipelineMode;  // enum for pipeline state

    bool m_targetStatus;  // target is seen or not [0,1]

    double m_horizontalOffset;  // horizontal target offset in degrees [-27, 27]
    double m_verticalOffset;  // vertical target offset in degrees [-20.5, 20.5]
    double m_targetArea;      // target area in percentage of image [0, 100]
    double m_targetSkew;      // target skew or rotation [-90, 0]
    double m_latency;  // The pipeline’s latency contribution (ms) Add at least
                       // 11ms for image capture latency.

    static constexpr double TARGET_ASPECT_RATIO =
        6.0 / 15.0;  // Constant for the targets aspect ratio
};
}
