#ifndef ESP32_SDI12_H
#define ESP32_SDI12_H

#include "SoftwareSerial.h"

#define SDI12_SERIAL_DEBUG

class ESP32_SDI12 {
private:
    // Software Serial instance (initialised in `begin()`)
    SoftwareSerial uart;

    static constexpr int8_t MAX_N_DEVICES       =       9;
    static constexpr int8_t LEN_VENDOR          =       8;
    static constexpr int8_t LEN_MODEL           =       6;
    static constexpr int8_t LEN_SENSOR_VERSION  =       3;
    static constexpr int8_t LEN_INFO            =       13;
    // Baud rate of SDI-12 TX/RX
    static constexpr int16_t SDI12_BAUD         =       1200;
    // Max size of response (in bytes)
    static constexpr int8_t MAX_RES_SIZE        =       75;
    static constexpr int8_t MAX_CMD_SIZE        =       8;
    // Message buffer for response
    char msg_buf[MAX_RES_SIZE] = {0};
    char sensor_cmd[MAX_CMD_SIZE] = {0};

    enum Commands {
        SDI12_Measure           =       0x00,
        SDI12_CRC_Measure       =       0x01,
        SDI12_AD_Measure        =       0x02,
        SDI12_AD_CRC_Measure    =       0x03,
        SDI12_Data              =       0x04,
        SDI12_Attention         =       0x05,
        SDI12_Information       =       0x06,
        SDI12_CNC_Measure       =       0x07,
        SDI12_CND_CRC_Measure   =       0x08,
        SDI12_Verify            =       0x09,
        SDI12_Change_Address    =       0x0A
    };

    struct Measure {
        uint8_t address;
        uint16_t delay_time;
        uint8_t n_values;
    };

public:
    enum Status {
        SDI12_OK                =       0x00,
        SDI12_ERR               =       0x01,
        SDI12_SENSOR_NOT_FOUND  =       0x02,
        SDI12_INVALID_ADDR      =       0x03,
        SDI12_ADDR_IN_USE       =       0x04,
        SDI12_TIMEOUT           =       0x05,
        SDI12_BUF_OVERFLOW      =       0x06
    };

    struct Sensor {
        uint8_t address;
        uint8_t sdi_version_major;
        uint8_t sdi_version_minor;
        uint8_t vendor[LEN_VENDOR];
        uint8_t model[LEN_MODEL];
        uint8_t sensor_version[LEN_SENSOR_VERSION];
        uint8_t aux_information[LEN_INFO];
    };

    struct Sensors {
        uint8_t count;
        Sensor sensor[MAX_N_DEVICES];
    };

    explicit ESP32_SDI12(int8_t pin);
    ~ESP32_SDI12();
    void begin();
    Status ackActive(uint8_t address);
    Status sensorInfo(uint8_t address, Sensor* sensor);
    Status sensorInfo(uint8_t address);
    Status sensorsOnBus(Sensors* sensors);
    Status changeAddress(uint8_t address,
                         uint8_t newAddress);
    Status measure(uint8_t address, float* values, uint16_t n_values);

    int8_t sdi12_pin = -1;

private:
    Status querySensor(uint8_t address, Commands cmd, uint8_t position = 0,
                       uint8_t newAddress = 0);
    static Status validAddress(uint8_t address);
    Status requestMeasure(uint8_t address, Measure* measure);
    Status requestData(uint8_t address, uint8_t position);

#ifdef SDI12_SERIAL_DEBUG
    // Debug message that outputs sensor information over a serial connection
    void sensor_debug(Sensor* sensor);
#endif // SDI12_SERIAL_DEBUG

};

#endif //ESP32_SDI12_H
