# -*- coding: utf-8 -*-
"""Nice Soldering Pen bootloader
Usage: niceSolderingPenBootloader.exe com_name [file_name]
    com_name: name of serial port: "COM1" or "/dev/ttyS1"
    file_name: name of file (*.bin) to be uploaded. 
        If not present, "firmware.bin" is used"""

import serial, sys

def error(text):
    print(__doc__)
    print("Error: " + text)
    quit()

class bootloader:
    serial = None
    file = None
    
    def read_info(self):
        pass
    
    def read_logs(self):
        pass
    
    def read_app(self):
        pass
    
    def write_app(self):
        pass
    
    def erase_app(self):
        pass

    def __init__(self, serial_name, file_name):
        try:
            self.serial = serial.Serial(serial_name, 1200)
        except serial.SerialException:
            error("Cannot open COM port " + serial_name)
        print("Open serial port " + self.serial.name)
        if file_name:
            try:
                self.file = open(file_name,"rb")
            except IOError:
                error("Cannot open file " + file_name)
            print("Open file " + file_name)
        
    def __del__(self):
        if(self.serial):
            self.serial.close()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        error("The folowing arguments are required: com_name")
    else:
        serial_name = sys.argv[1]
        if len(sys.argv) < 3:
            file_name = "firmware.bin"
        else:
            file_name = sys.argv[2]
        bld = bootloader(serial_name, file_name)
        bld.read_info()
        bld.erase_app()
        bld.write_app()