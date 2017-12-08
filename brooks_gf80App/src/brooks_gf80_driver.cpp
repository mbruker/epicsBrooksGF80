/*
    EPICS asyn driver for Brooks GF80 "S-protocol" device
    Dec 2017, M. W. Bruker
*/

#include "brooks_gf80_driver.h"
#include <asynDriver.h>
#include <iocsh.h>
#include <epicsExport.h>
#include <asynOctetSyncIO.h>
#include <stdint.h>
#include <string.h>
#include <cantProceed.h>

/*
    These two functions work on a Raspberry Pi but are architecture-dependent
    because of endianness. They should eventually be replaced by a better implementation.
*/
float BrooksGF80Driver::ieee754toFloat(unsigned char *number)
{
    union {
        float f;
        unsigned char c[4];
    } u;
    u.c[3] = number[0];
    u.c[2] = number[1];
    u.c[1] = number[2];
    u.c[0] = number[3];
    return u.f;
}

void BrooksGF80Driver::floatToIeee754(float number, unsigned char *result)
{
    union {
        float f;
        unsigned char c[4];
    } u;
    u.f = number;
    result[0] = u.c[3];
    result[1] = u.c[2];
    result[2] = u.c[1];
    result[3] = u.c[0];
}


BrooksGF80Driver::BrooksGF80Driver(const char *portName, const char *serialPortName, const char *tagName)
   : asynPortDriver(portName,
                    1, /* maxAddr */
                    numParams,
                    asynInt32Mask | asynFloat64Mask | asynOctetMask | asynDrvUserMask,
                    asynInt32Mask | asynFloat64Mask | asynOctetMask,
                    ASYN_CANBLOCK, /* asynFlags */
                    1, /* autoConnect */
                    0, /* default priority */
                    0) /* default stack size */
{
    asynStatus status;
    size_t replyBytes = 0;
    unsigned char replyData[24];
    asynInterface *pasynInterface;
    struct ioPvt *pioPvt;
    
    if (strlen(tagName) != 8) {
        printf("Error: tagName has to contain exactly 8 characters.\n");
        return;
    }
    
    pioPvt = (struct ioPvt *) callocMustSucceed(1, sizeof(struct ioPvt), "BrooksGF80");
    asynUserSerial = pasynManager->createAsynUser(0, 0);
    asynUserSerial->userPvt = pioPvt;
    status = pasynManager->connectDevice(asynUserSerial, serialPortName, 0);
    if (status != asynSuccess) {
        printf("Cannot connect to port %s: %s\n", serialPortName, asynUserSerial->errorMessage);
        return;
    }
    pasynInterface = pasynManager->findInterface(asynUserSerial, asynOctetType, 1);
    if (!pasynInterface) {
        printf("%s interface not supported\n", asynOctetType);
        return;
    }
    pioPvt->pasynOctet = (asynOctet *) pasynInterface->pinterface;
    pioPvt->octetPvt = pasynInterface->drvPvt;

    // get long address of device
    deviceAddress[0] = 0;
    deviceAddress[1] = 0;
    deviceAddress[2] = 0;
    deviceAddress[3] = 0;
    deviceAddress[4] = 0;

    unsigned char tagNameData[6];
    tagNameData[0] = (tagName[0] & 0x3f) << 2 | (tagName[1] & 0x3f) >> 4;
    tagNameData[1] = (tagName[1] & 0x3f) << 4 | (tagName[2] & 0x3f) >> 2;
    tagNameData[2] = (tagName[2] & 0x3f) << 6 | (tagName[3] & 0x3f);
    tagNameData[3] = (tagName[4] & 0x3f) << 2 | (tagName[5] & 0x3f) >> 4;
    tagNameData[4] = (tagName[5] & 0x3f) << 4 | (tagName[6] & 0x3f) >> 2;
    tagNameData[5] = (tagName[6] & 0x3f) << 6 | (tagName[7] & 0x3f);
    
    sendCommand(11, tagNameData, 6, replyData, &replyBytes);
    if (replyBytes != 12) {
        printf("Error: command 11: invalid reply from device\n");
        return;
    }
    deviceAddress[0] = replyData[1];
    deviceAddress[1] = replyData[2];
    deviceAddress[2] = replyData[9];
    deviceAddress[3] = replyData[10];
    deviceAddress[4] = replyData[11];
    printf("GF80: Device found: Mfr ID=%02x; Device type=%02x; ID=%02x %02x %02x\n", deviceAddress[0], deviceAddress[1], deviceAddress[2], deviceAddress[3], deviceAddress[4]);

    // Get PID controller values
    sendCommand(220, 0, 0, replyData, &replyBytes);
    if (replyBytes != 12) {
        printf("Error: command 220: invalid reply from device\n");
        return;
    }
    createParam(P_PID_KP_String, asynParamFloat64, &P_PID_KP);
    setDoubleParam(P_PID_KP, ieee754toFloat(&replyData[0]));
    createParam(P_PID_KI_String, asynParamFloat64, &P_PID_KI);
    setDoubleParam(P_PID_KI, ieee754toFloat(&replyData[4]));
    createParam(P_PID_KD_String, asynParamFloat64, &P_PID_KD);
    setDoubleParam(P_PID_KD, ieee754toFloat(&replyData[8]));

    char addressString[15];
    sprintf(addressString, "%02x %02x %02x %02x %02x", deviceAddress[0], deviceAddress[1], deviceAddress[2], deviceAddress[3], deviceAddress[4]);
    createParam(P_DeviceAddressString, asynParamOctet, &P_DeviceAddress);
    setStringParam(P_DeviceAddress, addressString);

    createParam(P_FlowUnitString, asynParamInt32, &P_FlowUnit);
    setIntegerParam(P_FlowUnit, 0);
    createParam(P_GetFlowRateString, asynParamFloat64, &P_GetFlowRate);
    setDoubleParam(P_GetFlowRate, 0.0);
    createParam(P_FlowSetpointString, asynParamFloat64, &P_FlowSetpoint);
    setDoubleParam(P_FlowSetpoint, 0.0);
    createParam(P_GetValveControlString, asynParamFloat64, &P_GetValveControl);
    setDoubleParam(P_GetValveControl, 0.0);
}

unsigned char BrooksGF80Driver::calculateChecksum(unsigned char *message, int len)
{
    unsigned char checksum = 0;
        while (--len >= 0)
    checksum ^= message[len];
    return checksum;
}


/*
    0 <= len <= 24
*/
asynStatus BrooksGF80Driver::sendCommand(unsigned char commandId, unsigned char *dataToSend, size_t sendLen, unsigned char *dataRecvd, size_t *recvdLen)
{
    asynStatus status = asynSuccess;
    size_t numBytes = 0;
    unsigned char message[50];
    unsigned char *pMessage = message;
    struct ioPvt *pioPvt = (struct ioPvt *) asynUserSerial->userPvt;

    // preamble
    for (unsigned char i = 0; i < preambleBytes; ++i)
        *pMessage++ = 0xff;
    
    // start character: long frame addressing, master to slave
    *pMessage++ = 0x82;

    // device address
    *pMessage++ = deviceAddress[0] | 0x80;
    *pMessage++ = deviceAddress[1];
    *pMessage++ = deviceAddress[2];
    *pMessage++ = deviceAddress[3];
    *pMessage++ = deviceAddress[4];

    // command ID
    *pMessage++ = commandId;

    // extra data
    *pMessage++ = (unsigned char) sendLen;
    for (unsigned char i = 0; i < sendLen; ++i)
        *pMessage++ = dataToSend[i];
    
    // checksum byte
    *pMessage++ = calculateChecksum(message + preambleBytes, sendLen + 8);

    // lock device until complete reply has been received
    pasynManager->lockPort(asynUserSerial);

    // send message to device and read reply
    asynUserSerial->timeout = 1.0;
    status = pioPvt->pasynOctet->write(pioPvt->octetPvt, asynUserSerial, (char *) message, sendLen + 9 + preambleBytes, &numBytes);

    do {
        status = pioPvt->pasynOctet->read(pioPvt->octetPvt, asynUserSerial, (char *) message, 1, &numBytes, 0);
    } while (message[0] == 0xff);
    status = pioPvt->pasynOctet->read(pioPvt->octetPvt, asynUserSerial, (char *) message + 1, 7, &numBytes, 0);
    if (numBytes < 9) {
        // ...
    }
    
    size_t replyBytes = message[7];
    if (replyBytes > 24) {
        replyBytes = 24;
        // ...
    }
    if (recvdLen)
        *recvdLen = replyBytes - 2;

    // read the rest: two status bytes (included in byte count), all data bytes, one checksum byte (not included in byte count)
    status = pioPvt->pasynOctet->read(pioPvt->octetPvt, asynUserSerial, (char *) message + 8, replyBytes + 1, &numBytes, 0);
    if (numBytes < replyBytes + 1) {
        // ...
    }
    unsigned int deviceStatus = message[8] << 8 | message[9];

    if (message[8 + replyBytes] != calculateChecksum(message, 8 + replyBytes)) {
        printf("GF80: invalid checksum in reply\n");
        status = asynError;
    }
    
    if (dataRecvd)
        for (unsigned int i = 0; i < *recvdLen; ++i)
            dataRecvd[i] = message[10 + i];
    
    pasynManager->unlockPort(asynUserSerial);

    return status;
}

/*
    Multiplying by this factor converts the internal unit to L/min.
    Returns 0 for unsupported unit codes, so check before dividing.
*/
epicsFloat64 BrooksGF80Driver::getFlowConversionFactor(unsigned char unitCode) const
{
    // see s-protocol manual p. 101
    switch (unitCode) {
        case 17: return 1.0;            // l / min
        case 19: return 1000.0 / 60.0;  // m3 / h
        case 24: return 60.0;           // l / s
        case 28: return 60000.0;        // m3 / s
        case 57: return 0;              // % XXX depends on calibration, how do we determine the range?
        case 131: return 1000.0;        // m3 / min
        case 138: return 1.0 / 60.0;    // l / h
        case 170: return 0.06;          // ml / s
        case 171: return 0.001;         // ml / min
        case 172: return 1.0 / 60000.0; // ml / h
    }
    return 0;
}

asynStatus BrooksGF80Driver::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
    return asynPortDriver::readInt32(pasynUser, value);
}

asynStatus BrooksGF80Driver::readFloat64(asynUser *pasynUser, epicsFloat64 *value)
{
    asynStatus status = asynSuccess;
    int function = pasynUser->reason;
    unsigned char reply[24];
    size_t replyLength = 0;

    if (function == P_GetFlowRate) {
        status = sendCommand(1, 0, 0, reply, &replyLength);
        if ((status != asynSuccess) || (replyLength < 5)) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, "%s: GetFlowRate: invalid reply", "Brooks_GF80");
            return asynError;
        }
        // normalize result to L/min by dividing by the unit used by the device
        *value = getFlowConversionFactor(reply[0]) * ieee754toFloat(&reply[1]);
        return asynSuccess;
    } else if (function == P_GetValveControl) {
        status = sendCommand(237, 0, 0, reply, &replyLength);
        if ((status != asynSuccess) || (replyLength < 3)) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, "%s: GetValveControl: invalid reply", "Brooks_GF80");
            return asynError;
        }
        // Full scale = 62500. Return value in %
        *value = (((uint32_t) reply[0]) << 16 | ((uint32_t) reply[1]) << 8 | (uint32_t) reply[2]) / 625.0;
        return asynSuccess;
    } else if (function == P_FlowSetpoint) {
        status = sendCommand(235, 0, 0, reply, &replyLength);
        if ((status != asynSuccess) || (replyLength < 10)) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, "%s: ReadSetpoint: invalid reply", "Brooks_GF80");
            return asynError;
        }
        setIntegerParam(P_FlowUnit, reply[5]);
        *value = getFlowConversionFactor(reply[5]) * ieee754toFloat(&reply[6]);
        return asynSuccess;
    } else
        return asynPortDriver::readFloat64(pasynUser, value);
}

asynStatus BrooksGF80Driver::writePIDValues()
{
    unsigned char commandData[12];
    double kp = 0, ki = 0, kd = 0;
    getDoubleParam(P_PID_KP, &kp);
    getDoubleParam(P_PID_KI, &ki);
    getDoubleParam(P_PID_KD, &kd);
    floatToIeee754(kp, &commandData[0]);
    floatToIeee754(ki, &commandData[4]);
    floatToIeee754(kd, &commandData[8]);
    return sendCommand(221, commandData, 12, 0, 0);
}

asynStatus BrooksGF80Driver::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;

    if (function == P_PID_KP) {
        setDoubleParam(P_PID_KP, value);
        return writePIDValues();
    } else if (function == P_PID_KI) {
        setDoubleParam(P_PID_KI, value);
        return writePIDValues();
    } else if (function == P_PID_KD) {
        setDoubleParam(P_PID_KD, value);
        return writePIDValues();
    } else if (function == P_FlowSetpoint) {
        int flowUnit = 0;
        getIntegerParam(P_FlowUnit, &flowUnit);
        epicsFloat64 factor = getFlowConversionFactor(flowUnit);
        if (factor == 0)
            return asynError;
        unsigned char commandData[5];
        commandData[0] = 250;
        floatToIeee754(value / factor, &commandData[1]);
        return sendCommand(236, commandData, 5, 0, 0);
    } else
        return asynPortDriver::writeFloat64(pasynUser, value);
}


extern "C" {

int BrooksGF80Configure(const char *portName, const char *serialPortName, char *tagName)
{
    new BrooksGF80Driver(portName, serialPortName, tagName); // scary but apparently the usual way
    return asynSuccess;
}

static const iocshArg initArg0 = { "portName", iocshArgString };
static const iocshArg initArg1 = { "serialPortName", iocshArgString };
static const iocshArg initArg2 = { "tagName", iocshArgString };
static const iocshArg * const initArgs[] = {&initArg0, &initArg1, &initArg2};
static const iocshFuncDef initFuncDef = { "BrooksGF80Configure", 3, initArgs };
static void initCallFunc(const iocshArgBuf *args)
{
    BrooksGF80Configure(args[0].sval, args[1].sval, args[2].sval);
}

void BrooksGF80DriverRegister()
{
    iocshRegister(&initFuncDef, initCallFunc);
}

epicsExportRegistrar(BrooksGF80DriverRegister);

}

