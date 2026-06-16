import serial  # Import the library to manage UART/Serial communication
import time    # Import library for delays and timing
import struct  # Import library to pack/unpack C-style binary data (floats/ints)
import sys     # Used to exit the program cleanly if needed


# Checking the correct port, run the following command on the terminal
# ls /dev/tty{ACM,USB}*

'''
For a stable naming convention on the port
# Find your device's idVendor and idProduct
udevadm info -a -n /dev/ttyACM0 | grep -E "idVendor|idProduct"

# Then create /etc/udev/rules.d/99-robot-micro.rules:
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="YYYY", SYMLINK+="robot_micro"


sudo -S  chmod 666 /dev/ttyUSB0
'''

def main():
    # --- CONFIGURATION VARIABLES ---
    # We define the size of the package. 
    # floats are 4 bytes. 6 values * 4 bytes = 24 bytes total per message.
    # If you want to test 8 or 10 data, change this to 8 or 10.
    NUM_VALUES_TX = 7
    NUM_VALUES_RX = 10
    PACKET_SIZE_TX = NUM_VALUES_TX * 4 
    PACKET_SIZE_RX = NUM_VALUES_RX * 4 
    
    # 'struct' format string. 
    # '<' means little-endian (standard for ARM micros).
    # 'f' means float. We repeat 'f' 6 times: '<ffffff'
    # If you want to send Integers (like for Color or Health), use 'i' instead of 'f'.
    # For now, we use all floats to keep it uniform and easy to test.
    STRUCT_FORMAT_TX = '<' + ('f' * NUM_VALUES_TX)
    STRUCT_FORMAT_RX = '<' + ('f' * NUM_VALUES_RX)

    t_start = time.monotonic()
    
    print("--- Starting Serial Communication ---")

    # --- PORT SETUP ---
    # We initialize the serial port object.
    try:
        ser = serial.Serial(
            port="/dev/ttyACM0",       # The physical port on the MiniPC (Jetson/Pi)
            baudrate=500000,           # 500.000 bps as requested
            bytesize=serial.EIGHTBITS, # 8 Data bits.
            
            # --- PARITY CONFIGURATION ---
            # "9 bits including parity" usually means 8 Data + 1 Parity.
            # If this fails, change to serial.PARITY_NONE.
            parity=serial.PARITY_EVEN, 
            
            stopbits=serial.STOPBITS_ONE, # 1 Stop bit
            timeout=0.01               # Read timeout: wait 10ms max then move on if no data
        )
        print(f"Serial port -> STATUS: OK\n")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return


    
    # Initialize variables for the data we want to send
    # Yaw, Pitch, Shoot, NavX, NavY, Angle
    
    tx_yaw = 0.0
    tx_pitch = 0.0
    tx_shoot = 0.0 # 1.0 for yes, 0.0 for no
    tx_nav_x = 0.0
    tx_nav_y = 0.0
    tx_angle = 0.0

    try:
        while True:
            # --- 1. RECEIVING DATA (Micro -> MiniPC) ---
            # We check if there are enough bytes in the buffer to form a full packet.
            if ser.in_waiting >= PACKET_SIZE_RX:
                
                # Read exactly 24 bytes (or however big the packet is)
                rx_bytes = ser.read(PACKET_SIZE_RX)
                
                # Check if we actually got the full amount (safety check)
                if len(rx_bytes) == PACKET_SIZE_RX:
                    try:
                        # Unpack the bytes into a tuple of numbers
                        # We expect: Color, Start, Health, Ammo, Center, Resupply
                        unpacked_data = struct.unpack(STRUCT_FORMAT_RX, rx_bytes)
                        
                        print(f"[RX] Recv from Micro: {[f'{v:.2f}' for v in unpacked_data]}")
                        
                    except struct.error as e:
                        print(f"Unpacking error: {e}")
            
            # --- 2. LOGIC UPDATE ---
            # Update the dummy data to simulate movement/changes
            tx_yaw += 0.1
            tx_pitch += 0.05
            if tx_yaw > 100: tx_yaw = 0 # Reset to avoid huge numbers
            tx_timestamp = time.monotonic() - t_start
            
            # --- 3. SENDING DATA (MiniPC -> Micro) ---
            # Prepare the list of values to send
            data_to_send = [
                tx_yaw,    # Yaw
                tx_pitch,  # Pitch
                tx_shoot,  # Shoot (1 or 0) TBD
                tx_nav_x,  # X
                tx_nav_y,  # Y
                tx_angle   # Angle
            ]
            
            # Pack these numbers into binary bytes
            #tx_bytes = struct.pack(STRUCT_FORMAT_TX, *data_to_send)
            tx_bytes = struct.pack(STRUCT_FORMAT_TX, tx_timestamp, *data_to_send)
            # Write the bytes to the wire
            ser.write(tx_bytes)
            
            print(f"[TX] Sent to Micro:   ts={tx_timestamp:.6f}s  |  {[f'{v:.2f}' for v in data_to_send]}")

            # --- 4. TIMING ---
            # 1ms interval. 
            # Note: Python sleep is not perfectly precise. 0.001 = 1ms.
            # Printing to console takes time, so you might see it running slower than 1ms.
            time.sleep(0.001) 

    except KeyboardInterrupt:
        print("\nStopping Serial Communication...")
        ser.close() # Close port nicely on Ctrl+C

if __name__ == '__main__':
    main()
