#include <cmath>

#include "odyssey/exitcondition.hpp"
#include "pros/rtos.hpp"

namespace odyssey {

ExitCondition::ExitCondition(float range, int time)
    : range(range),
      time(time) {}

bool ExitCondition::getExit() const { return done; }

bool ExitCondition::update(float input) {
    const int currentTime = pros::millis();
    if (std::fabs(input) > range) {
        // outside the range: restart the timer
        startTime = -1;
    } else if (startTime == -1) {
        // just entered the range: start the timer
        startTime = currentTime;
    } else if (currentTime >= startTime + time) {
        // stayed in the range long enough
        done = true;
    }
    return done;
}

void ExitCondition::reset() {
    startTime = -1;
    done = false;
}

} // namespace odyssey
