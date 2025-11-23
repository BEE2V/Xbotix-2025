#include "AsyncTrigger.h"
AsyncTrigger::AsyncTrigger(long p) : period(p), lastTriggerTime(0) {}

bool AsyncTrigger::check()
{
    long currentTime = millis();
    if (currentTime - lastTriggerTime >= period)
    {
        lastTriggerTime = currentTime;
        return true;
    }
    return false;
}