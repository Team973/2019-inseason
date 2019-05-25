#include "lib/bases/AutoRoutineBase.h"

namespace frc973 {
AutoRoutineBase::AutoRoutineBase() : m_autoState(0) {
}

AutoRoutineBase::~AutoRoutineBase() {
}

void AutoRoutineBase::Execute() {
    std::cout << "Routine Start" << std::endl;
}

void AutoRoutineBase::Reset() {
    m_autoState = 0;
}
}
