#include "crsf.h"

// — CRC8 Table —
static const uint8_t crsf_crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
    };

uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) {
    uint8_t crc = 0;
    while (len--) {
        crc = crsf_crc8tab[crc ^ *ptr++];
    }
    return crc;
}

// — CRSFParser (Double Buffered) Implementation —
CRSFParser::CRSFParser() {
    statsFramesOk = 0;
    statsCrcFails = 0;
    statsLenRejects = 0;
    statsOverflows = 0;
    _lastLenTotal = 0;
    reset(); // Sets working state to 0
}

void CRSFParser::reset() {
    // Reset only the working assembler state
    _state = 0; // 0=Addr, 1=Len, 2=Data
    _idx = 0;
    _targetLen = 0;
}

bool CRSFParser::feed(uint8_t b) {
    if (_idx >= sizeof(_bufWorking)) {
        statsOverflows++;
        reset(); 
    }

    switch (_state) {
        case 0: // Expecting Address
            // Buffer fill starts
            _bufWorking[0] = b;
            _state = 1;
            break;

        case 1: // Expecting Len
            if (b >= 2 && b <= CRSF_FRAME_SIZE_MAX) {
                _targetLen = b;
                _bufWorking[1] = b;
                _idx = 2; // Type starts at index 2
                _state = 2;
            } else {
                statsLenRejects++;
                reset(); 
            }
            break;

        case 2: // Reading Payload + CRC
            _bufWorking[_idx++] = b;
            
            // Check if done
            // Total = Addr(1) + Len(1) + [Len bytes]
            // _idx points to next, so count is _idx
            if (_idx == (_targetLen + 2)) {
                // Done. CRC Check
                // CRC covers Type..End. (From index 2 to End-1)
                // Length to check: _targetLen - 1 (exclude CRC byte)
                
                uint8_t expectedCrc = _bufWorking[_idx - 1]; // Last byte
                uint8_t calculatedCrc = crsf_crc8(&_bufWorking[2], _targetLen - 1);
                
                if (expectedCrc == calculatedCrc) {
                    statsFramesOk++;
                    // VALID FRAME -> Copy to Last Buffer
                    memcpy(_bufLast, _bufWorking, _idx);
                    _lastLenTotal = _idx;
                    
                    reset(); // Prepare working buffer for next, but...
                    return true; // Notify caller that _bufLast has fresh data
                } else {
                    statsCrcFails++;
                    reset();
                }
            }
            break;
    }
    return false;
}

uint8_t CRSFParser::getFrameType() const { return _bufLast[2]; }
const uint8_t* CRSFParser::getPayload() const { return &_bufLast[3]; }
uint8_t CRSFParser::getPayloadLen() const { return (_lastLenTotal > 2) ? (_lastLenTotal - 4) : 0; } // Total - Addr(1)-Len(1)-Type(1)-CRC(1) ? No. Len=Type+Pay+CRC.
// Len byte value = Type(1) + Payload(N) + CRC(1).
// Payload Len = Len - 2.
// _lastLenTotal = 2 + Len.
const uint8_t* CRSFParser::getFrame() const { return _bufLast; }
uint8_t CRSFParser::getFrameLenTotal() const { return _lastLenTotal; }


// — Helper Function —

void crsfDecodeChannels(const uint8_t *payload, uint16_t out[16]) {
    // 11 bits per channel. Packed.
    out[0]  = (payload[0]       | payload[1] << 8)                        & 0x07FF;
    out[1]  = (payload[1] >> 3  | payload[2] << 5)                        & 0x07FF;
    out[2]  = (payload[2] >> 6  | payload[3] << 2 | payload[4] << 10)     & 0x07FF;
    out[3]  = (payload[4] >> 1  | payload[5] << 7)                        & 0x07FF;
    out[4]  = (payload[5] >> 4  | payload[6] << 4)                        & 0x07FF;
    out[5]  = (payload[6] >> 7  | payload[7] << 1 | payload[8] << 9)      & 0x07FF;
    out[6]  = (payload[8] >> 2  | payload[9] << 6)                        & 0x07FF;
    out[7]  = (payload[9] >> 5  | payload[10] << 3)                       & 0x07FF;
    out[8]  = (payload[11]      | payload[12] << 8)                       & 0x07FF;
    out[9]  = (payload[12] >> 3 | payload[13] << 5)                       & 0x07FF;
    out[10] = (payload[13] >> 6 | payload[14] << 2 | payload[15] << 10)   & 0x07FF;
    out[11] = (payload[15] >> 1 | payload[16] << 7)                       & 0x07FF;
    out[12] = (payload[16] >> 4 | payload[17] << 4)                       & 0x07FF;
    out[13] = (payload[17] >> 7 | payload[18] << 1 | payload[19] << 9)    & 0x07FF;
    out[14] = (payload[19] >> 2 | payload[20] << 6)                       & 0x07FF;
    out[15] = (payload[20] >> 5 | payload[21] << 3)                       & 0x07FF;
}

void crsfBuildChannelsFrame(uint8_t outFrame[26], const uint16_t ch[16]) {
    outFrame[0] = CRSF_ADDRESS_MODULE;
    outFrame[1] = 24; 
    outFrame[2] = CRSF_TYPE_CHANNELS;
    
    uint8_t* p = &outFrame[CRSF_PAYLOAD_OFFSET]; // Points to byte index 3
    
    // Copy to temp buffer AND MASK 11-BIT
    unsigned int buf[16];
    for(int i=0;i<16;i++) buf[i] = ch[i] & 0x07FF; // Safety Mask
    
    p[0]  = (uint8_t)(buf[0]);
    p[1]  = (uint8_t)(buf[0] >> 8 | buf[1] << 3);
    p[2]  = (uint8_t)(buf[1] >> 5 | buf[2] << 6);
    p[3]  = (uint8_t)(buf[2] >> 2);
    p[4]  = (uint8_t)(buf[2] >> 10 | buf[3] << 1);
    p[5]  = (uint8_t)(buf[3] >> 7 | buf[4] << 4);
    p[6]  = (uint8_t)(buf[4] >> 4 | buf[5] << 7);
    p[7]  = (uint8_t)(buf[5] >> 1);
    p[8]  = (uint8_t)(buf[5] >> 9 | buf[6] << 2);
    p[9]  = (uint8_t)(buf[6] >> 6 | buf[7] << 5);
    p[10] = (uint8_t)(buf[7] >> 3);
    p[11] = (uint8_t)(buf[8]);
    p[12] = (uint8_t)(buf[8] >> 8 | buf[9] << 3);
    p[13] = (uint8_t)(buf[9] >> 5 | buf[10] << 6);
    p[14] = (uint8_t)(buf[10] >> 2);
    p[15] = (uint8_t)(buf[10] >> 10 | buf[11] << 1);
    p[16] = (uint8_t)(buf[11] >> 7 | buf[12] << 4);
    p[17] = (uint8_t)(buf[12] >> 4 | buf[13] << 7);
    p[18] = (uint8_t)(buf[13] >> 1);
    p[19] = (uint8_t)(buf[13] >> 9 | buf[14] << 2);
    p[20] = (uint8_t)(buf[14] >> 6 | buf[15] << 5);
    p[21] = (uint8_t)(buf[15] >> 3);
    
    outFrame[25] = crsf_crc8(&outFrame[2], 23); 
}

void CRSF::begin() {}
void CRSF::crsfPrepareDataPacket(uint8_t packet[], int16_t channels[]) {
    uint16_t uCh[16];
    for(int i=0; i<16; i++) uCh[i] = (uint16_t)channels[i];
    crsfBuildChannelsFrame(packet, uCh);
}
void CRSF::CrsfWritePacket(uint8_t packet[], uint8_t packetLength) {
    Serial1.write(packet, packetLength);
}
void CRSF::crsfPrepareCmdPacket(uint8_t packetCmd[], uint8_t command, uint8_t value) {
    packetCmd[0] = CRSF_ADDRESS_MODULE;
    packetCmd[1] = 6;
    packetCmd[2] = 0x2D; // TYPE_SETTINGS_WRITE
    packetCmd[3] = CRSF_ADDRESS_MODULE;
    packetCmd[4] = CRSF_ADDRESS_RADIO;
    packetCmd[5] = command;
    packetCmd[6] = value;
    packetCmd[7] = crsf_crc8(&packetCmd[2], 5);
}
