/*
	EPICS asyn driver for Brooks GF80 "S-protocol" device
	Dec 2017, M. W. Bruker
*/

#ifndef _BROOKS_GF80_DRIVER_H
#define _BROOKS_GF80_DRIVER_H

#include "asynPortDriver.h"
#include "stdint.h"

class asynUser;

#define P_DeviceAddressString "DEVICE_ADDRESS"
#define P_FlowUnitString "FLOW_UNIT"
#define P_GetFlowRateString "GET_FLOW_RATE"
#define P_FlowSetpointString "FLOW_SETPOINT"
#define P_GetValveControlString "GET_VALVE_CONTROL"
#define P_PID_KP_String "PID_KP"
#define P_PID_KI_String "PID_KI"
#define P_PID_KD_String "PID_KD"

class BrooksGF80Driver : public asynPortDriver {
public:
    BrooksGF80Driver(const char *portName, const char *serialPortName, const char *tagName);
    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    virtual asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
protected:
    float ieee754toFloat(unsigned char *number);
    void floatToIeee754(float number, unsigned char *result);
    unsigned char calculateChecksum(unsigned char *message, int len);
    asynStatus sendCommand(unsigned char commandId, unsigned char *dataToSend, size_t sendLen, unsigned char *dataRecvd, size_t *recvdLen);
    asynStatus writePIDValues();
    epicsFloat64 getFlowConversionFactor(unsigned char unitCode) const;
    
    unsigned char deviceAddress[5];
    static const int preambleBytes = 5;
    static const int numParams = 8;
    int P_DeviceAddress;
    int P_FlowUnit;
    int P_GetFlowRate;
    int P_FlowSetpoint;
    int P_GetValveControl;
    int P_PID_KP;
    int P_PID_KI;
    int P_PID_KD;

    asynUser *asynUserSerial;
};


#endif

