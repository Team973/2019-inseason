#include "lib/sensors/Limelight.h"

using namespace frc;

namespace frc973 {
Limelight::Limelight(const char *name)
        : m_limelight(nt::NetworkTableInstance::GetDefault().GetTable(name))
        , m_camName(name)
        , m_lightMode(LightMode::on)
        , m_cameraMode(CameraMode::onVision)
        , m_targetStatus(false)
        , m_horizontalOffset(0.0)
        , m_verticalOffset(0.0)
        , m_targetArea(0.0)
        , m_targetSkew(0.0)
        , m_latency(0.0) {
    SetCameraDriver();
}

Limelight::~Limelight() {
}

void Limelight::SetLightMode(LightMode mode) {
    switch (mode) {
        case LightMode::on:
            m_limelight->PutNumber("ledMode", 0);
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
    try {
        if (index >= 0 || index <= 9) {
            m_limelight->PutNumber("pipeline", 0);
        }
        else {
            throw std::out_of_range("Limelight pipeline must be between 0-9.");
        }
    } catch (const std::out_of_range &e) {
        std::cout << "Limelight pipeline index out of range." << std::endl;
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

void Limelight::SetLightOn() {
    SetLightMode(LightMode::on);
}

void Limelight::SetLightOff() {
    SetLightMode(LightMode::off);
}

void Limelight::SetLightBlink() {
    SetLightMode(LightMode::blink);
}

void Limelight::SetCameraVision() {
    SetLightOn();
    SetPipeline(PipelineMode::target_vision);
    SetCameraMode(CameraMode::onVision);
}

void Limelight::SetCameraDriver() {
    SetLightOff();
    SetPipeline(PipelineMode::drive);
    SetCameraMode(CameraMode::onDriver);
}

bool Limelight::isTargetValid() {
    return m_limelight->GetNumber("tv", 0.0);
}

double Limelight::GetXOffset() {
    return m_limelight->GetNumber("tx", 0.0);
}

double Limelight::GetYOffset() {
    return m_limelight->GetNumber("ty", 0.0);
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
}
