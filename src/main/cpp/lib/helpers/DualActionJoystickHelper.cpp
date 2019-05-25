#include "lib/helpers/DualActionJoystickHelper.h"

namespace frc973 {

ObservableDualActionJoystick::ObservableDualActionJoystick(
    uint16_t port, DualActionJoystickObserver *observer, TaskMgr *scheduler,
    DriverStation *ds)
        : Joystick(port)
        , m_port(port)
        , m_observer(observer)
        , m_ds(ds)
        , m_prevBtn(0)
        , m_scheduler(scheduler)
        , m_logCell(nullptr)
        , m_lastLXVal(false)
        , m_lastLYVal(false)
        , m_lastRXVal(false)
        , m_lastRYVal(false)
        , m_lastDXVal(false)
        , m_lastDYVal(false) {
    if (m_ds == nullptr) {
        m_ds = &DriverStation::GetInstance();
    }

    m_prevBtn = m_ds->GetStickButtons(port);

    if (scheduler != nullptr) {
        scheduler->RegisterTask("JoystickHelper", this, TASK_PRE_PERIODIC);
    }
}

ObservableDualActionJoystick::~ObservableDualActionJoystick() {
    if (m_scheduler != nullptr) {
        m_scheduler->UnregisterTask(this);
    }
}

ObservableDualActionJoystick *ObservableDualActionJoystick::RegisterLog(
    LogSpreadsheet *logger) {
    if (m_logCell == nullptr) {
        // TODO this memory is never freed
        char *cellTitleBuf = (char *)malloc(32 * sizeof(char));
        sprintf(cellTitleBuf, "Joystick Btn Port %d", m_port);

        m_logCell = new LogCell(cellTitleBuf, 64);
        logger->RegisterCell(m_logCell);
    }

    return this;
}

float ObservableDualActionJoystick::GetRawAxisWithDeadband(int axis,
                                                           bool fSquared,
                                                           double threshold) {
    float value = Util::deadband(GetRawAxis(axis), threshold);

    if (fSquared) {
        value = Util::signSquare(value);
    }

    return value;
}

bool ObservableDualActionJoystick::GetDPadUpVirtButton() {
    int pov = GetPOV();
    return pov == 0 || pov == 315 || pov == 45;
}

bool ObservableDualActionJoystick::GetDPadDownVirtButton() {
    int pov = GetPOV();
    return pov == 180 || pov == 225 || pov == 135;
}

bool ObservableDualActionJoystick::GetDPadLeftVirtButton() {
    int pov = GetPOV();
    return pov == 270 || pov == 315 || pov == 225;
}

bool ObservableDualActionJoystick::GetDPadRightVirtButton() {
    int pov = GetPOV();
    return pov == 90 || pov == 135 || pov == 45;
}

bool ObservableDualActionJoystick::GetLXVirtButton() {
    double pos = this->GetRawAxis(DualAction::LeftXAxis);

    if (pos > VIRTUAL_JOYSTICK_THRESHOLD) {
        m_lastLXVal = true;
    }
    else if (pos < -VIRTUAL_JOYSTICK_THRESHOLD) {
        m_lastLXVal = false;
    }

    return m_lastLXVal;
}

bool ObservableDualActionJoystick::GetLYVirtButton() {
    double pos = -this->GetRawAxis(DualAction::LeftYAxis);

    if (pos > VIRTUAL_JOYSTICK_THRESHOLD) {
        m_lastLYVal = true;
    }
    else if (pos < -VIRTUAL_JOYSTICK_THRESHOLD) {
        m_lastLYVal = false;
    }

    return m_lastLYVal;
}

bool ObservableDualActionJoystick::GetRXVirtButton() {
    double pos = this->GetRawAxis(DualAction::RightXAxis);

    if (pos > VIRTUAL_JOYSTICK_THRESHOLD) {
        m_lastRXVal = true;
    }
    else if (pos < -VIRTUAL_JOYSTICK_THRESHOLD) {
        m_lastRXVal = false;
    }

    return m_lastRXVal;
}

bool ObservableDualActionJoystick::GetRYVirtButton() {
    double pos = -this->GetRawAxis(DualAction::RightYAxis);

    if (pos > VIRTUAL_JOYSTICK_THRESHOLD) {
        m_lastRYVal = true;
    }
    else if (pos < -VIRTUAL_JOYSTICK_THRESHOLD) {
        m_lastRYVal = false;
    }

    return m_lastRYVal;
}

bool ObservableDualActionJoystick::GetDXVirtButton() {
    if (this->GetDPadRightVirtButton()) {
        m_lastDXVal = true;
    }
    else if (this->GetDPadLeftVirtButton()) {
        m_lastDXVal = false;
    }

    return m_lastDXVal;
}

bool ObservableDualActionJoystick::GetDYVirtButton() {
    if (this->GetDPadUpVirtButton()) {
        m_lastDYVal = true;
    }
    else if (this->GetDPadDownVirtButton()) {
        m_lastDYVal = false;
    }

    return m_lastDYVal;
}

uint32_t ObservableDualActionJoystick::GetAllButtons() {
    uint32_t btns = m_ds->GetStickButtons(m_port);

    btns |= GetLXVirtButton() << (DualAction::LXAxisVirtButton - 1);
    btns |= GetLYVirtButton() << (DualAction::LYAxisVirtButton - 1);
    btns |= GetRXVirtButton() << (DualAction::RXAxisVirtButton - 1);
    btns |= GetRYVirtButton() << (DualAction::RYAxisVirtButton - 1);
    btns |= GetDXVirtButton() << (DualAction::DXAxisVirtButton - 1);
    btns |= GetDYVirtButton() << (DualAction::DYAxisVirtButton - 1);
    btns |= GetDPadUpVirtButton() << (DualAction::DPadUpVirtBtn - 1);
    btns |= GetDPadDownVirtButton() << (DualAction::DPadDownVirtBtn - 1);
    btns |= GetDPadLeftVirtButton() << (DualAction::DPadLeftVirtBtn - 1);
    btns |= GetDPadRightVirtButton() << (DualAction::DPadRightVirtBtn - 1);

    return btns;
}

/**
 * Careful this code is dense and contains crazy bit-shifty logic.
 *    X&~(X^-X) extracts the least significant set bit from X in mask form
 *    __builtin_ffs(Y) gets the position of the least significant set bit
 */
void ObservableDualActionJoystick::TaskPrePeriodic(RobotMode mode) {
    uint32_t currBtn = GetAllButtons();

    if (m_observer != nullptr) {
        uint32_t changedBtn = m_prevBtn ^ currBtn;
        uint32_t btnMask, btn;

        while (changedBtn != 0) {
            /* btnMask contains the least significant set bit in changedBtn */
            btnMask = changedBtn & ~(changedBtn ^ -changedBtn);
            /* btn contains the index of the lssb in btnMask... aka the button
             * number */
            btn = __builtin_ffs(btnMask);
            if ((currBtn & btnMask) != 0) {
                /* Button is pressed */
                m_observer->ObserveDualActionJoystickStateChange(m_port, btn,
                                                                 true);
            }
            else {
                /* Button is released */
                m_observer->ObserveDualActionJoystickStateChange(m_port, btn,
                                                                 false);
            }
            /* clear |changedBtn| from the mask so we can get the next lsb */
            changedBtn &= ~btnMask;
        }
    }
    m_prevBtn = currBtn;

    if (m_logCell) {
        m_logCell->LogPrintf("%x", currBtn);
    }
}
}
