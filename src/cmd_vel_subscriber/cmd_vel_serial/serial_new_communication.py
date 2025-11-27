import serial  # Import the library to manage UART/Serial communication
import time    # Import library for delays and timing
import struct  # Import library to pack/unpack C-style binary data (floats/ints)
import sys     # Used to exit the program cleanly if needed

def main():
    # --- CONFIGURATION VARIABLES ---
    # We define the size of the package. 
    # floats are 4 bytes. 6 values * 4 bytes = 24 bytes total per message.
    # If you want to test 8 or 10 data, change this to 8 or 10.
    NUM_VALUES = 6 
    PACKET_SIZE = NUM_VALUES * 4  
    
    # 'struct' format string. 
    # '<' means little-endian (standard for ARM micros).
    # 'f' means float. We repeat 'f' 6 times: '<ffffff'
    # If you want to send Integers (like for Color or Health), use 'i' instead of 'f'.
    # For now, we use all floats to keep it uniform and easy to test.
    STRUCT_FORMAT = '<' + ('f' * NUM_VALUES)

    print("--- Starting Serial Communication ---")

    # --- PORT SETUP ---
    # We initialize the serial port object.
    try:
        ser = serial.Serial(
            port="/dev/ttyTHS0",       # The physical port on the MiniPC (Jetson/Pi)
            baudrate=500000,           # 500.000 bps as requested
            bytesize=serial.EIGHTBITS, # 8 Data bits.
            
            # --- PARITY CONFIGURATION ---
            # "9 bits including parity" usually means 8 Data + 1 Parity.
            # If this fails, change to serial.PARITY_NONE.
            parity=serial.PARITY_EVEN, 
            
            stopbits=serial.STOPBITS_ONE, # 1 Stop bit
            timeout=0.01               # Read timeout: wait 10ms max then move on if no data
        )
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
            if ser.in_waiting >= PACKET_SIZE:
                
                # Read exactly 24 bytes (or however big the packet is)
                rx_bytes = ser.read(PACKET_SIZE)
                
                # Check if we actually got the full amount (safety check)
                if len(rx_bytes) == PACKET_SIZE:
                    try:
                        # Unpack the bytes into a tuple of numbers
                        # We expect: Color, Start, Health, Ammo, Center, Resupply
                        unpacked_data = struct.unpack(STRUCT_FORMAT, rx_bytes)
                        
                        print(f"[RX] Recv from Micro: {unpacked_data}")
                        
                    except struct.error as e:
                        print(f"Unpacking error: {e}")
            
            # --- 2. LOGIC UPDATE ---
            # Update the dummy data to simulate movement/changes
            tx_yaw += 0.1
            tx_pitch += 0.05
            if tx_yaw > 100: tx_yaw = 0 # Reset to avoid huge numbers
            
            # --- 3. SENDING DATA (MiniPC -> Micro) ---
            # Prepare the list of values to send
            data_to_send = [
                tx_yaw,    # Yaw
                tx_pitch,  # Pitch
                tx_shoot,  # Shoot (1 or 0)
                tx_nav_x,  # X
                tx_nav_y,  # Y
                tx_angle   # Angle
            ]
            
            # Pack these numbers into binary bytes
            # The '*' unpacking operator passes the list items as separate arguments
            tx_bytes = struct.pack(STRUCT_FORMAT, *data_to_send)
            
            # Write the bytes to the wire
            ser.write(tx_bytes)
            
            print(f"[TX] Sent to Micro:   {data_to_send}")

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
