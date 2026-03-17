#pragma once

namespace tank_drive {
    bool init();
    void shutdown();
    bool handle_key(char ch);
    void tick();
}

