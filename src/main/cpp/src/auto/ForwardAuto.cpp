#include "src/auto/ForwardAuto.h"

using namespace frc;

namespace frc973 {
ForwardAuto::ForwardAuto(Drive *drive) : m_drive(drive) {
}

ForwardAuto::~ForwardAuto() {
}

void ForwardAuto::Execute() {
    switch (m_autoState) {
        case 0:
            m_drive->PIDDrive(150.0, 0.0, Drive::RelativeTo::Now, 0.8);
            m_autoState++;
            break;
    }
}

void ForwardAuto::Reset() {
}
}
