#ifndef CRSF_H
#define CRSF_H

#include <Arduino.h>
#include <stdint.h>
#include "config.h" 

// — Basic Defines
#define CRSF_MAX_CHANNEL        16
#define CRSF_FRAME_SIZE_MAX     64 

// — Addresses & Types
#define CRSF_ADDRESS_MODULE     0xEE 
#define CRSF_ADDRESS_RADIO      0xEA 
#define CRSF_ADDRESS_FLIGHT     0xC8
#define CRSF_TYPE_CHANNELS      0x16

// — Frame Structure Constants
#define CRSF_PACKET_SIZE        26   
#define CRSF_PAYLOAD_OFFSET     3    

// — Classes

// 1. Parser Class
class CRSFParser {
public:
    CRSFParser();
    void reset();
    bool feed(uint8_t b); // Returns true if a full valid frame is ready in Last buffer

    // Accessors for Last Valid Frame
    uint8_t getFrameType() const;
    const uint8_t* getPayload() const; 
    uint8_t getPayloadLen() const;     
    const uint8_t* getFrame() const;   
    uint8_t getFrameLenTotal() const;  

    // Stats
    uint32_t statsFramesOk;
    uint32_t statsCrcFails;
    uint32_t statsLenRejects;
    uint32_t statsOverflows;

private:
    // Working Buffer (Being assembled)
    uint8_t _state;
    uint8_t _bufWorking[CRSF_FRAME_SIZE_MAX + 4]; 
    uint8_t _idx;
    uint8_t _targetLen; 

    // Last Valid Buffer (Ready for consumption)
    uint8_t _bufLast[CRSF_FRAME_SIZE_MAX + 4];
    uint8_t _lastLenTotal;
};

// 2. Output Helper Class
class CRSF {
public:
    void begin(void);
    void crsfPrepareDataPacket(uint8_t packet[], int16_t channels[]); 
    void CrsfWritePacket(uint8_t packet[], uint8_t packetLength);
    void crsfPrepareCmdPacket(uint8_t packetCmd[], uint8_t command, uint8_t value);
};

// — Helper Prototypes
void crsfDecodeChannels(const uint8_t *payload, uint16_t out[16]);
// Alias for requirement satisfaction:
inline bool crsfDecodeChannelsPacked(const uint8_t payload22[22], uint16_t out16[16]) {
    crsfDecodeChannels(payload22, out16);
    return true; 
}
void crsfBuildChannelsFrame(uint8_t outFrame[26], const uint16_t ch[16]);
uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len);

#endif // CRSF_H
