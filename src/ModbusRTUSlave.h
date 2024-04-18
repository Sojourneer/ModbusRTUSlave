#ifndef ModbusRTUSlave_h
#define ModbusRTUSlave_h

#define MODBUS_RTU_SLAVE_BUF_SIZE 256
#define NO_DE_PIN 255
#define NO_ID 0

#include "Arduino.h"
#ifdef __AVR__
#include <SoftwareSerial.h>
#endif

typedef struct {
  int16_t start;
  int16_t count;
  int8_t  handler_index;
} register_block_t;

typedef bool (*ByteHandler)(int16_t base_register, int16_t offset, int16_t store_index);
typedef bool (*WordHandler)(int16_t base_register, int16_t offset, int16_t store_index);

class ModbusRTUSlave {
  public:
    ModbusRTUSlave(HardwareSerial& serial, uint8_t dePin = NO_DE_PIN);
    #ifdef __AVR__
    ModbusRTUSlave(SoftwareSerial& serial, uint8_t dePin = NO_DE_PIN);
    #endif
    #ifdef HAVE_CDCSERIAL
    ModbusRTUSlave(Serial_& serial, uint8_t dePin = NO_DE_PIN);
    #endif


    void registerByteHandlers(ByteHandler[]);
    void registerWordHandlers(WordHandler[]);

    void configureCoils(register_block_t specs[], uint8_t spec_count, bool coils[]);
    void configureDiscreteInputs(register_block_t specs[], uint8_t spec_count, uint8_t discreteInputs[]);
    void configureHoldingRegisters(register_block_t specs[], uint8_t spec_count, uint16_t holdingRegisters[]);
    void configureInputRegisters(register_block_t specs[], uint8_t spec_count, uint16_t inputRegisters[]);

    #ifdef ESP32
    void begin(uint8_t id, unsigned long baud, uint32_t config = SERIAL_8N1, int8_t rxPin = -1, int8_t txPin = -1, bool invert = false);
    #else
    void begin(uint8_t id, unsigned long baud, uint32_t config = SERIAL_8N1);
    #endif
    void poll();
    
  private:
    HardwareSerial *_hardwareSerial;
    #ifdef __AVR__
    SoftwareSerial *_softwareSerial;
    #endif
    #ifdef HAVE_CDCSERIAL
    Serial_ *_usbSerial;
    #endif
    Stream *_serial;
    uint8_t _dePin;
    uint8_t _buf[MODBUS_RTU_SLAVE_BUF_SIZE];

    bool *_coils;
    uint8_t *_discreteInputs;
    uint16_t *_holdingRegisters;
    uint16_t *_inputRegisters;

    register_block_t* _coil_specs;
    register_block_t* _DI_specs;
    register_block_t* _HR_specs;
    register_block_t* _IR_specs;
    uint8_t           _coil_spec_count;
    uint8_t           _DI_spec_count;
    uint8_t           _HR_spec_count;
    uint8_t           _IR_spec_count;
    
    ByteHandler*  _byteHandlers;
    WordHandler*  _wordHandlers;

    uint8_t _id;
    unsigned long _charTimeout;
    unsigned long _frameTimeout;

    uint8_t findSpec(int16_t reg, register_block_t *specs, uint8_t spec_count, int16_t &block_start, int16_t &offset, uint8_t &handler_index, int16_t &store_index);
    int16_t reg_to_storeIndex(register_block_t* spec, uint8_t spec_count, int16_t reg);

    int16_t coil_to_storeIndex(int16_t reg) { return reg_to_storeIndex(_coil_specs, _coil_spec_count, reg); }
    int16_t DI_to_storeIndex(int16_t reg) { return reg_to_storeIndex(_DI_specs, _DI_spec_count, reg); }
    int16_t Hreg_to_storeIndex(int16_t reg) { return reg_to_storeIndex(_HR_specs, _HR_spec_count, reg); }
    int16_t Ireg_to_storeIndex(int16_t reg) { return reg_to_storeIndex(_IR_specs, _IR_spec_count, reg); }

    void _processReadCoils();
    void _processReadDiscreteInputs();
    void _processReadHoldingRegisters();
    void _processReadInputRegisters();
    void _processWriteSingleCoil();
    void _processWriteSingleHoldingRegister();
    void _processWriteMultipleCoils();
    void _processWriteMultipleHoldingRegisters();

    bool _readRequest();
    void _writeResponse(uint8_t len);
    void _exceptionResponse(uint8_t code);
    void _clearRxBuffer();

    void _calculateTimeouts(unsigned long baud, uint32_t config);
    uint16_t _crc(uint8_t len);
    uint16_t _div8RndUp(uint16_t value);
    uint16_t _bytesToWord(uint8_t high, uint8_t low);
};

#endif
