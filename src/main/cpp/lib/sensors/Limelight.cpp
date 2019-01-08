#include "lib/sensors/Limelight.h"

using namespace frc;

namespace frc973 {
Limelight::Limelight()
        : m_limelight(
              nt::NetworkTableInstance::GetDefault().GetTable("limelight"))
        , m_lightMode(LightMode::on)
        , m_cameraMode(CameraMode::onVision)
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
    SetCameraMode(CameraMode::onVision);
}

void Limelight::SetCameraDriver() {
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
