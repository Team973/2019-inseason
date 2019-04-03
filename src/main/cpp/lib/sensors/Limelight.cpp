#include "lib/sensors/Limelight.h"

using namespace frc;

namespace frc973 {
Limelight::Limelight(const char *name, bool inverted)
        : m_limelight(nt::NetworkTableInstance::GetDefault().GetTable(name))
        , m_camName(name)
        , m_isInverted(inverted)
        , m_lightMode(LightMode::off)
        , m_cameraMode(CameraMode::onDriver)
        , m_streamMode(StreamMode::standard)
        , m_snapshotMode(SnapshotMode::stop)
        , m_pipelineMode(PipelineMode::drive)
        , m_lowPassSkewFilter(new MovingAverageFilter(0.75))
        , m_targetStatus(false)
        , m_horizontalOffset(0.0)
        , m_verticalOffset(0.0)
        , m_targetArea(0.0)
        , m_targetSkew(0.0)
        , m_latency(0.0) {
}

Limelight::~Limelight() {
}

void Limelight::SetLightMode(LightMode mode) {
    switch (mode) {
        case LightMode::on:
            m_limelight->PutNumber("ledMode", 3);
            break;
        case LightMode::off:
            m_limelight->PutNumber("ledMode", 1);
            break;
        case LightMode::blink:
            m_limelight->PutNumber("ledMode", 2);
            break;
    }
}

void Limelight::SetCameraMode(CameraMode mode) {
    switch (mode) {
        case CameraMode::onVision:
            m_limelight->PutNumber("camMode", 0);
            break;
        case CameraMode::onDriver:
            m_limelight->PutNumber("camMode", 1);
            break;
    }
}

void Limelight::SetPipelineIndex(int index) {
    if (index >= 0 || index <= 9) {
        m_limelight->PutNumber("pipeline", index);
    }
    else {
        printf("Limelight pipeline must be between 0-9. Ignoring value: %d\n",
               index);
    }
}

void Limelight::SetPipeline(PipelineMode mode) {
    switch (mode) {
        case PipelineMode::drive:
            SetPipelineIndex(0);
            break;
        case PipelineMode::vision_center:
            SetPipelineIndex(1);
            break;
        case PipelineMode::vision_right:
            SetPipelineIndex(2);
            break;
        case PipelineMode::vision_left:
            SetPipelineIndex(3);
            break;
    }
}

void Limelight::SetStreamMode(StreamMode mode) {
    switch (mode) {
        case StreamMode::standard:
            m_limelight->PutNumber("stream", 0);
            break;
        case StreamMode::pipMain:
            m_limelight->PutNumber("stream", 1);
            break;
        case StreamMode::pipSecondary:
            m_limelight->PutNumber("stream", 2);
            break;
    }
}

void Limelight::SetSnapshotMode(SnapshotMode mode) {
    switch (mode) {
        case SnapshotMode::stop:
            m_limelight->PutNumber("snapshot", 0);
            break;
        case SnapshotMode::start:
            m_limelight->PutNumber("snapshot", 1);
            break;
    }
}

void Limelight::SetPiPMain() {
    SetStreamMode(StreamMode::pipMain);
}

void Limelight::SetPiPSecondary() {
    SetStreamMode(StreamMode::pipSecondary);
}

void Limelight::SetLightOn() {
    SetLightMode(LightMode::on);
}

void Limelight::SetLightOff() {
    SetLightMode(LightMode::off);
}

void Limelight::SetLightBlink() {
    SetLightMode(LightMode::blink);
}

void Limelight::SetCameraDriver() {
    SetPipeline(PipelineMode::drive);
    SetCameraMode(CameraMode::onDriver);
    SetLightOff();
}

void Limelight::SetCameraVisionCenter() {
    SetPipeline(PipelineMode::vision_center);
    SetCameraMode(CameraMode::onVision);
    SetLightOn();
}

void Limelight::SetCameraVisionRight() {
    SetPipeline(PipelineMode::vision_right);
    SetCameraMode(CameraMode::onVision);
    SetLightOn();
}

void Limelight::SetCameraVisionLeft() {
    SetPipeline(PipelineMode::vision_left);
    SetCameraMode(CameraMode::onVision);
    SetLightOn();
}

bool Limelight::isTargetValid() {
    return m_limelight->GetNumber("tv", 0.0);
}

double Limelight::GetXOffset() {
    if (m_isInverted) {
        return -m_limelight->GetNumber("tx", 0.0);
    }
    else {
        return m_limelight->GetNumber("tx", 0.0);
    }
}

double Limelight::GetYOffset() {
    if (m_isInverted) {
        return -m_limelight->GetNumber("ty", 0.0);
    }
    else {
        return m_limelight->GetNumber("ty", 0.0);
    }
}

double Limelight::GetTargetArea() {
    return m_limelight->GetNumber("ta", 0.0);
}

double Limelight::GetTargetSkew() {
    double skew = m_limelight->GetNumber("ts", 0.0);
    if (skew < -45.0) {
        return m_lowPassSkewFilter->Update(skew + 90.0);
    }
    else {
        return m_lowPassSkewFilter->Update(skew);
    }
}

double Limelight::GetLatency() {
    return m_limelight->GetNumber("tl", 0.0);
}

double Limelight::GetPipeline() {
    return m_limelight->GetNumber("pipeline", 0.0);
}

double Limelight::GetHorizontalLength() {
    return m_limelight->GetNumber("tlong", 0.0);
}

double Limelight::GetVerticalLength() {
    return m_limelight->GetNumber("tvert", 0.0);
}

double Limelight::FindTargetSkew() {
    return (180.0 / Constants::PI) *
           asin(Util::bound((GetHorizontalLength() / GetVerticalLength()) *
                                TARGET_ASPECT_RATIO,
                            0.0, 1.0));
}

double Limelight::GetHorizontalDistance() {
    if (SmartDashboard::GetBoolean("DB/Button 0", false) == true) {
        return ((m_DBTargetHeight - m_DBCameraHeight) /
                tan(m_DBCameraAngle +
                    this->GetYOffset() * (Constants::PI / 180.0))) -
               m_DBCameraBumperOffset;
    }
    else {
        return ((TARGET_HEIGHT - CAMERA_HEIGHT) /
                tan(CAMERA_ANGLE +
                    this->GetYOffset() * (Constants::PI / 180.0))) -
               CAMERA_BUMPER_OFFSET;
    }
}

void Limelight::UpdateLimelightDB() {
    m_DBTargetHeight =
        stod(SmartDashboard::GetString("DB/String 6", "0.0").substr(4, 8));
    m_DBCameraAngle =
        stod(SmartDashboard::GetString("DB/String 7", "0.0").substr(4, 9)) *
        (Constants::PI / 180.0);
    m_DBCameraBumperOffset =
        stod(SmartDashboard::GetString("DB/String 7", "0.0").substr(16, 20));
    m_DBCameraHeight =
        stod(SmartDashboard::GetString("DB/String 6", "0.0").substr(14, 18));
}

void Limelight::CreateLimelightDB() {
    DBStringPrintf(DB_LINE6, "TH: 29.00 CH: 47.00");
    DBStringPrintf(DB_LINE7, "CA: -27.30 CBO: 19.25");
}
}
