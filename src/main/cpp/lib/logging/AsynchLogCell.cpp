#include "lib/logging/AsynchLogCell.h"

namespace frc973 {

AsynchLogCell::AsynchLogCell(char *name, AsynchLogCellListener *listener,
                             unsigned int size)
        : LogCell(name, size), m_listener(listener) {
}

const char *AsynchLogCell::GetContent() {
    m_listener->NotifyAsynchLogCellListener(this);

    return m_buffer;
}
}
