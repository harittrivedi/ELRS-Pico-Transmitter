import serial
import serial.tools.list_ports
import time
import struct
import math
import sys

# Configuration
BAUD_RATE = 115200    # USB Serial ignores baud, but good practice

# CRSF Constants
CRSF_SYNC = 0xC8      
CRSF_TYPE_RC = 0x16
CRSF_CRC_POLY = 0xD5

def get_serial_port():
    # 1. Check Command Line Args
    if len(sys.argv) > 1:
        return sys.argv[1]
        
    # 2. List Available Ports
    ports = list(serial.tools.list_ports.comports())
    
    if not ports:
        print("No ports detected.")
        return input("Enter COM port manually (e.g. COM10): ")
        
    print("\n--- Available Serial Ports ---")
    for i, p in enumerate(ports):
        print(f"[{i}] {p.device} ({p.description})")
    print("------------------------------")
    
    try:
        selection = input(f"Select Port [0-{len(ports)-1}] or type name: ")
        if selection.isdigit():
            idx = int(selection)
            if 0 <= idx < len(ports):
                return ports[idx].device
        return selection # Treat as manual name if not valid index
    except KeyboardInterrupt:
        sys.exit(0)

def crsf_crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ CRSF_CRC_POLY
            else:
                crc = crc << 1
        crc &= 0xFF
    return crc

def build_crsf_channels(channels):
    # Channels is a list of 16 integers (0-2047)
    # Pack 16x 11-bit channels into 22 bytes
    p = bytearray(22)
    
    ch = [c & 0x7FF for c in channels] 
    
    p[0]  = ch[0] & 0xFF
    p[1]  = (ch[0] >> 8) | ((ch[1] << 3) & 0xFF)
    p[2]  = (ch[1] >> 5) | ((ch[2] << 6) & 0xFF)
    p[3]  = (ch[2] >> 2) & 0xFF
    p[4]  = (ch[2] >> 10) | ((ch[3] << 1) & 0xFF)
    p[5]  = (ch[3] >> 7) | ((ch[4] << 4) & 0xFF)
    p[6]  = (ch[4] >> 4) | ((ch[5] << 7) & 0xFF)
    p[7]  = (ch[5] >> 1) & 0xFF
    p[8]  = (ch[5] >> 9) | ((ch[6] << 2) & 0xFF)
    p[9]  = (ch[6] >> 6) | ((ch[7] << 5) & 0xFF)
    p[10] = (ch[7] >> 3) & 0xFF
    p[11] = ch[8] & 0xFF
    p[12] = (ch[8] >> 8) | ((ch[9] << 3) & 0xFF)
    p[13] = (ch[9] >> 5) | ((ch[10] << 6) & 0xFF)
    p[14] = (ch[10] >> 2) & 0xFF
    p[15] = (ch[10] >> 10) | ((ch[11] << 1) & 0xFF)
    p[16] = (ch[11] >> 7) | ((ch[12] << 4) & 0xFF)
    p[17] = (ch[12] >> 4) | ((ch[13] << 7) & 0xFF)
    p[18] = (ch[13] >> 1) & 0xFF
    p[19] = (ch[13] >> 9) | ((ch[14] << 2) & 0xFF)
    p[20] = (ch[14] >> 6) | ((ch[15] << 5) & 0xFF)
    p[21] = (ch[15] >> 3) & 0xFF

    dest_addr = 0xEE 
    length = 24 
    
    frame = bytearray()
    frame.append(dest_addr)
    frame.append(length)
    frame.append(CRSF_TYPE_RC)
    frame.extend(p)
    frame.append(crsf_crc8(frame[2:]))
    
    return frame

def main():
    port_name = get_serial_port()
    
    try:
        ser = serial.Serial(port_name, BAUD_RATE, timeout=0.01)
        print(f"Opened {port_name}. Sending CRSF sine wave...")
    except Exception as e:
        print(f"Error opening {port_name}: {e}")
        return

    t0 = time.time()
    
    try:
        while True:
            t = time.time()
            dt = t - t0
            
            # Simple animation: Sweep CH1 (Roll) and CH2 (Pitch)
            val1 = 992 + int(800 * math.sin(dt * 2))  # Fast sweep
            val2 = 992 + int(800 * math.cos(dt * 0.5)) # Slow sweep
            
            channels = [992] * 16
            channels[0] = val1 # Aileron
            channels[1] = val2 # Elevator
            channels[2] = 172  # Throttle Down
            channels[4] = 1811 # Aux1 High (Arm)
            
            frame = build_crsf_channels(channels)
            ser.write(frame)
            
            # Read Feedback
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                # Print hex snippet if data received
                if len(data) > 0:
                   print(f"\rRX: {data.hex()[:20]}...", end="")

            time.sleep(0.02) # 50Hz update rate
            
    except KeyboardInterrupt:
        print("\nStopped.")
        ser.close()

if __name__ == "__main__":
    main()
