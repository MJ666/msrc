#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include "device.h"
#include "formatData.h"

class Sensor : public FormatData
{
protected:
    uint16_t timestamp_ = 0, dataId_, frameId_ = 0x10;
    float *valueLP_;
    uint32_t value_;
    uint8_t refresh_;

public:
    AbstractDevice *deviceP_;
    Sensor(uint16_t dataId, float *valueLP, uint8_t refresh, AbstractDevice *deviceP);
    virtual ~Sensor();
    Sensor *nextP = NULL;
    uint16_t timestamp();
    void setTimestamp(uint16_t dataId);
    uint16_t dataId();
    uint16_t frameId();
    uint8_t refresh();
    void update();
    virtual uint32_t valueFormatted();
};

class SensorDouble : public Sensor
{
protected:
    float *valueMP_;

public:
    SensorDouble(uint16_t dataId, float *valueLP, float *valueMP, uint8_t refresh, AbstractDevice *deviceP);
    virtual uint32_t valueFormatted();
};

class SensorLatLon : public SensorDouble
{
protected:
    uint8_t type_ = TYPE_LAT;

public:
    SensorLatLon(uint16_t dataId, float *lonP, float *latP, uint8_t refresh, AbstractDevice *deviceP);
    uint32_t valueFormatted();
};

class SensorDateTime : public SensorDouble
{
protected:
    uint8_t type_ = TYPE_DATE;

public:
    SensorDateTime(uint16_t dataId, float *timeP, float *dateP, uint8_t refresh, AbstractDevice *deviceP);
    uint32_t valueFormatted();
};

class SensorCell : public SensorDouble
{
protected:
    uint8_t cellIndex_ = 0;

public:
    SensorCell(uint16_t dataId, float *indexM, float *indexL, uint8_t cellIndex, uint8_t refresh, AbstractDevice *deviceP);
    uint32_t valueFormatted();
};

class Sensord : public FormatData
{
protected:
    uint16_t timestamp_ = 0, value_;
    uint8_t dataId_;
    float *valueP_;
    uint8_t refresh_;

public:
    AbstractDevice *deviceP_;
    Sensord(uint8_t dataId, float *value, uint8_t refresh, AbstractDevice *deviceP);
    ~Sensord();
    Sensord *nextP = NULL;
    uint16_t timestamp();
    void setTimestamp(uint16_t dataId);
    uint8_t dataId();
    uint8_t refresh();
    void update();
    uint16_t valueFormatted();
};

class SensorIbus : public FormatData
{
protected:
    uint8_t dataId_;
    uint8_t type_;

public:
    AbstractDevice *deviceP_;
    SensorIbus(uint8_t dataId, uint8_t type, AbstractDevice *deviceP);
    virtual ~SensorIbus();
    uint8_t dataId();
    uint8_t type();
    void update();
    virtual uint8_t *valueFormatted() = 0;
};

class SensorIbusS16 : public SensorIbus
{
protected:
    float *valueP_;
    int16_t valueFormatted_;

public:
    SensorIbusS16(uint8_t dataId, uint8_t type, float *valueP, AbstractDevice *deviceP);
    ~SensorIbusS16();
    uint8_t *valueFormatted();
};

class SensorIbusU16 : public SensorIbus
{
protected:
    float *valueP_;
    uint16_t valueFormatted_;

public:
    SensorIbusU16(uint8_t dataId, uint8_t type, float *valueP, AbstractDevice *deviceP);
    ~SensorIbusU16();
    uint8_t *valueFormatted();
};

class SensorIbusS32 : public SensorIbus
{
protected:
    float *valueP_;
    int32_t valueFormatted_;

public:
    SensorIbusS32(uint8_t dataId, uint8_t type, float *valueP, AbstractDevice *deviceP);
    ~SensorIbusS32();
    uint8_t *valueFormatted();
};

class SensorIbusGps : public SensorIbus
{
protected:
    uint8_t *satP_;
    float *latP_;
    float *lonP_;
    float *altP_;
    uint8_t buffer[15] = {0}; // fix, 2x sat, 4x lat, 4x lon, 4x alt

public:
    SensorIbusGps(uint8_t dataId, uint8_t type, uint8_t *satP, float *latP, float *lonP, float *altP, AbstractDevice *deviceP);
    ~SensorIbusGps();
    uint8_t *valueFormatted();
};

class SensorSbus : public FormatData
{
protected:
    uint8_t dataId_;
    float *valueP_;

public:
    AbstractDevice *deviceP_;
    SensorSbus(uint8_t dataId, float *value, AbstractDevice *deviceP);
    ~SensorSbus();
    uint8_t dataId();
    float *valueP();
    void update();
    uint16_t valueFormatted();
};

class SensorMultiplex : public FormatData
{
protected:
    uint8_t dataId_;
    float *valueP_;

public:
    AbstractDevice *deviceP_;
    SensorMultiplex(uint8_t dataId, float *value, AbstractDevice *deviceP);
    ~SensorMultiplex();
    uint8_t dataId();
    float *valueP();
    void update();
    uint16_t valueFormatted();
};

class SensorJetiEx
{
protected:
    uint8_t sensorId_;
    uint8_t type_;
    uint8_t format_;
    float *valueP_;
    char text_[32] = "SENSOR";
    char unit_[8] = "";

public:
    AbstractDevice *deviceP_;
    SensorJetiEx(uint8_t type, uint8_t format, float *value, AbstractDevice *deviceP);
    ~SensorJetiEx();
    void setText(const char *textP);
    void setUnit(const char *textP);
    char *textP();
    char *unitP();
    void setSensorId(uint8_t sensorId);
    uint8_t type();
    uint8_t format();
    float *valueP();
    void update();
};

#endif