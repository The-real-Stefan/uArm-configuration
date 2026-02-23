import serial
import time

PORT = "/dev/ttyACM0"
BAUD = 115200
TIMEOUT = 3  # seconds
FEEDRATE = 10000

def main():
    try:
        with serial.Serial(PORT, BAUD, timeout=TIMEOUT) as ser:
            # Give device time to initialize (important for many USB CDC devices)
            time.sleep(2)
            ser.reset_input_buffer()

            while True:
                try:
                    x = float(input("X: "))
                    y = float(input("Y: "))
                    z = float(input("Z: "))
                except ValueError:
                    print("Invalid number. Try again.")
                    continue

                cmd = f"G0 X{x:.2f} Y{y:.2f} Z{z:.2f} F{FEEDRATE}\r\n"
                print(f"Sending: {cmd.strip()}")

                ser.write(cmd.encode("ascii"))
                ser.flush()

                # Read response (up to timeout)
                response = ser.readline().decode("ascii", errors="ignore").strip()
                if response:
                    print("Response:", response)
                else:
                    print("No response (timeout).")

    except serial.SerialException as e:
        print("Serial error:", e)


if __name__ == "__main__":
    main()
