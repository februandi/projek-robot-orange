import serial
import time

# Fungsi untuk mengirim perintah G-code ke Arduino
def send_gcode_command(ser, command):
    try:
        # Kirim perintah G-code
        ser.write((command + '\r').encode())
        print(f"Mengirim perintah: {command}")

        # Tunggu respon 'ok' dari Arduino
        while True:
            response = ser.readline().decode().strip()
            if response:
                print(f"Respon dari Arduino: {response}")
                if response == 'ok':
                    print("Perintah berhasil dijalankan.")
                    break

    except serial.SerialException as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    try:
        # Buka port serial (sesuaikan dengan port yang digunakan Arduino, misal 'COM3' atau '/dev/ttyUSB0')
        ser = serial.Serial('COM8', 115200, timeout=1)
        time.sleep(2)  # Tunggu Arduino siap

        # Kirim perintah G28 dan tunggu respon 'ok'
        send_gcode_command(ser, "G28")

        # Tunggu input dari user untuk melanjutkan
        input_lanjut = input("Perintah G28 selesai, ketik 'y' untuk melanjutkan: ")
        if input_lanjut.lower() == 'y':
            # Daftar perintah G-code yang akan dikirim
            gcode_commands = [
                "G0 X-83 Y143 Z162 E0 F0",
                "G0 X-83 Y253 Z162 E-500 F0",
                "G0 X68 Y32 Z301 E-500 F0",
                "T29",
                "G0 X68 Y32 Z301 E-834 F0",
                "T0",
                "G0 X68 Y93 Z301 E-834 F0",
                "G4 S1000",
                "G0 X-113 Y93 Z301 E-834 F0"
            ]

            # Kirim perintah G-code secara terus menerus dalam loop
            while True:
                for command in gcode_commands:
                    send_gcode_command(ser, command)

        ser.close()

    except serial.SerialException as e:
        print(f"Error: {e}")
