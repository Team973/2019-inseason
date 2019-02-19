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
        , m_pipelineMode(PipelineMode::off)
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
        case PipelineMode::default_vision:
            SetPipelineIndex(1);
            break;
        case PipelineMode::target_vision:
            SetPipelineIndex(2);
            break;
        case PipelineMode::off:
            SetPipelineIndex(3);
            break;
        case PipelineMode::vision3d:
            SetPipelineIndex(4);
            break;
    }
}

void Limelight::SetStreamMode(StreamMode mode) {
    switch (mode) {
        case StreamMode::standard:
            m_limelight->PutNumber("stream", 0);
            SmartDashboard::PutString("misc/limelight/currentCamera",
                                      "limelight");
            break;
        case StreamMode::pipMain:
            m_limelight->PutNumber("stream", 1);
            SmartDashboard::PutString("misc/limelight/currentCamera",
                                      "limelight");
            break;
        case StreamMode::pipSecondary:
            m_limelight->PutNumber("stream", 2);
            SmartDashboard::PutString("misc/limelight/currentCamera", "usb");
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
    DBStringPrintf(DBStringPos::DB_LINE1, "%f",
                   m_limelight->GetNumber("ledMode", 0));
    SetLightMode(LightMode::blink);
}

void Limelight::SetCameraVision() {
    SetPipeline(PipelineMode::target_vision);
    SetCameraMode(CameraMode::onVision);
    SetLightOn();
}

void Limelight::SetCameraDriver() {
    SetPipeline(PipelineMode::drive);
    SetCameraMode(CameraMode::onDriver);
    SetLightOn();
}

void Limelight::SetCameraDefaultVision() {
    SetPipeline(PipelineMode::default_vision);
    SetCameraMode(CameraMode::onVision);
    SetLightOn();
}

void Limelight::SetCameraOff() {
    SetPipeline(PipelineMode::off);
    SetCameraMode(CameraMode::onDriver);
    SetLightOff();
}

void Limelight::SetCamera3D() {
    SetPipeline(PipelineMode::vision3d);
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
    return m_limelight->GetNumber("ts", 0.0);
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
    return ((TARGET_HEIGHT - CAMERA_HEIGHT) /
            tan(CAMERA_ANGLE + this->GetYOffset() * (Constants::PI / 180.0))) -
           CAMERA_BUMPER_OFFSET;
}
}
