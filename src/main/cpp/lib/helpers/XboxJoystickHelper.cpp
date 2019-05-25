#include "lib/helpers/XboxJoystickHelper.h"

namespace frc973 {

ObservableXboxJoystick::ObservableXboxJoystick(uint16_t port,
                                               XboxJoystickObserver *observer,
                                               TaskMgr *scheduler,
                                               DriverStation *ds)
        : XboxController(port)
        , m_port(port)
        , m_observer(observer)
        , m_ds(ds)
        , m_prevBtn(0)
        , m_scheduler(scheduler)
        , m_logCell(nullptr) {
    if (m_ds == nullptr) {
        m_ds = &DriverStation::GetInstance();
    }

    m_prevBtn = m_ds->GetStickButtons(port);

    if (scheduler != nullptr) {
        scheduler->RegisterTask("JoystickHelper", this, TASK_PRE_PERIODIC);
    }
}

ObservableXboxJoystick::~ObservableXboxJoystick() {
    if (m_scheduler != nullptr) {
        m_scheduler->UnregisterTask(this);
    }
}

ObservableXboxJoystick *ObservableXboxJoystick::RegisterLog(
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

float ObservableXboxJoystick::GetRawAxisWithDeadband(int axis, bool fSquared,
                                                     double threshold) {
    float value = Util::deadband(GetRawAxis(axis), threshold);

    if (fSquared) {
        value = Util::signSquare(value);
    }

    return value;
}

bool ObservableXboxJoystick::GetDPadUpVirtButton() {
    int pov = GetPOV();
    return pov == 0 || pov == 315 || pov == 45;
}

bool ObservableXboxJoystick::GetDPadDownVirtButton() {
    int pov = GetPOV();
    return pov == 180 || pov == 225 || pov == 135;
}

bool ObservableXboxJoystick::GetDPadLeftVirtButton() {
    int pov = GetPOV();
    return pov == 270 || pov == 315 || pov == 225;
}

bool ObservableXboxJoystick::GetDPadRightVirtButton() {
    int pov = GetPOV();
    return pov == 90 || pov == 135 || pov == 45;
}

uint32_t ObservableXboxJoystick::GetAllButtons() {
    uint32_t btns = m_ds->GetStickButtons(m_port);

    btns |= GetDPadUpVirtButton() << (Xbox::DPadUpVirtBtn - 1);
    btns |= GetDPadDownVirtButton() << (Xbox::DPadDownVirtBtn - 1);
    btns |= GetDPadLeftVirtButton() << (Xbox::DPadLeftVirtBtn - 1);
    btns |= GetDPadRightVirtButton() << (Xbox::DPadRightVirtBtn - 1);

    return btns;
}

/**
 * Careful this code is dense and contains crazy bit-shifty logic.
 *    X&~(X^-X) extracts the least significant set bit from X in mask form
 *    __builtin_ffs(Y) gets the position of the least significant set bit
 */
void ObservableXboxJoystick::TaskPrePeriodic(RobotMode mode) {
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
                m_observer->ObserveXboxJoystickStateChange(m_port, btn, true);
            }
            else {
                /* Button is released */
                m_observer->ObserveXboxJoystickStateChange(m_port, btn, false);
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
