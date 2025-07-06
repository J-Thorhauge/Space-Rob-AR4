import serial
import sys
import argparse


def main():
    parser = argparse.ArgumentParser(description='A test program.')
    parser.add_argument("-p", "--usb_port", help="USB port.", default="'/dev/ttyACM1'")
    args = parser.parse_args()
    max = 60
    min = -5

    try:
        arduino = serial.Serial(args.usb_port, 1600)
        print("Serial device connected!")
    except serial.SerialException as e:
        print("Wrong port!")
        print("UBB Port: ", args.usb_port)
        return 0

    arduino.write((str(20+40)).encode('utf-8'))

    while True:
        angle = int(input("Servo position: "))
        if min <= angle <= max:
            arduino.write((str(angle+40)+"\n").encode('utf-8'))
        else:
            print("Angle is out of range (",min,"-",max,")")


if __name__ == '__main__':
    main()

