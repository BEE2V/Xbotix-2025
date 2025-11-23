#pragma once
#include <Arduino.h>
class AsyncTrigger
{
public:
    long period;
    long lastTriggerTime;

    AsyncTrigger(long p);
    bool check();
};