#ifndef _NMEA_H_
#define _NMEA_H_

void setupNMEA();
void loopNMEA();

void checkNmeaErrors(void *parameter);
void reciveNmeaMessage(void *parameter);

void SendN2kTankLevel(double level, double capacity); // Adjust max tank capacity.  Is it 200 ???
void SendN2kExhaustTemp(double temp);
void SendN2kInternalTemp(double temp);
void SendN2kEngineRPM(double RPM);
void SendN2kBattery(double BatteryVoltage);

#endif
