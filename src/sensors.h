#ifndef _SENSORS_H_
#define _SENSORS_H_

#include "sensesp/sensors/sensor.h"

using namespace sensesp;

FloatProducer *ConnectTachoSender(int pin, String name);
FloatProducer *ConnectTankSender(int pin, String name);

#endif
