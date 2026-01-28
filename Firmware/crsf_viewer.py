import serial
import time
import struct

# Configuration
SERIAL_PORT = 'COM10'  # CHANGE THIS to your FTDI port
BAUD_RATE = 400000

def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0xD5
            else:
                crc <<= 1
            crc &= 0xFF
    return crc

def parse_crsf_channels(payload):
    # CRSF Channel packing is 11-bits per channel, packed into bytes
    # This is a bit tricky to unpack manually
    # Payload is 22 bytes for 16 channels
    
    # Simple bit unpacking (Little Endian stream, but CRSF is Big Endian packed? No, it's weird)
    # Actually CRSF is:
    # byte 0: ch0[7:0]
    # byte 1: ch0[10:8] | ch1[4:0] << 3
    # ...
    
    # Let's convert bytes to a giant integer
    val = int.from_bytes(payload, 'little')
    
    channels = []
    for i in range(16):
        channels.append((val >> (i * 11)) & 0x7FF)
        
    return channels

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud...")
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return

    buffer = bytearray()
    
    while True:
        try:
            chunk = ser.read(100)
            if chunk:
                buffer.extend(chunk)
            
            # Simple Parser: Find Sync Byte and Length
            # CRSF Frame: [Sync] [Len] [Type] [Payload...] [CRC]
            # Sync is usually 0xC8 (GPS/FlightController) or 0xEA (Handset) or 0xEE (Transmitter)
            # Our Pico is sending to TX Module, so it sends as handset? Configured as 0xEA or 0xEE in code.
            
            while len(buffer) > 4:
                sync = buffer[0]
                length = buffer[1] # Length includes Type, Payload, CRC
                
                # Check for likely CRSF Sync bytes
                if (sync == 0xEA or sync == 0xC8 or sync == 0xEE) and length <= 62:
                    if len(buffer) >= length + 2:
                        frame = buffer[:length+2]
                        
                        # Validate CRC
                        payload_crc = buffer[2:length+1] # Type + Payload
                        calced_crc = crc8(payload_crc)
                        frame_crc = buffer[length+1]
                        
                        if calced_crc == frame_crc:
                            type_byte = buffer[2]
                            payload = buffer[3:length+1]
                            
                            if type_byte == 0x16: # RC Channels
                                channels = parse_crsf_channels(payload)
                                # Print nicely
                                print(f"RC: A:{channels[0]:4} E:{channels[1]:4} T:{channels[2]:4} R:{channels[3]:4} | A1:{channels[4]} A2:{channels[5]}", end='\r')
                            elif type_byte == 0x2D: # MSP/Config write
                                print(f"\nConfig Command Received: {list(payload)}")
                            else:
                                print(f"\nFrame Type: {hex(type_byte)} Len: {length}")
                                
                            # Consume frame
                            buffer = buffer[length+2:]
                            continue
                        else:
                            # Bad CRC
                            # print("Bad CRC") 
                            pass
                
                # If we get here, either sync incorrect or CRC fail
                # Shift buffer by 1 to search for next sync
                buffer = buffer[1:]
                
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
             # print(f"Error: {e}")
             pass

if __name__ == "__main__":
    main()
