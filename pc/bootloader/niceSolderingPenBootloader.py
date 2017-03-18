# -*- coding: utf-8 -*-
"""Nice Soldering Pen bootloader
Usage: niceSolderingPenBootloader.exe com_name [file_name]
    com_name: name of serial port: "COM1" or "/dev/ttyS1"
    file_name: name of file (*.bin) to be uploaded. 
        If not present, "firmware.bin" is used"""

import serial, sys, crcmod, time

def error(text):
    print(__doc__)
    print("Error: " + text)
    quit()

class bootloader:
    serial = None
    file = None
    crc_fun=None
    COMMAND_ERASE = b'\x81'
    COMMAND_WRITE = b'\x82'
    COMMAND_INFO = b'\x84'
    COMMAND_RUN_APP = b'\x88'
    RESPONSE_OK = b'o'
    RESPONSE_ERROR = b'e'
    WRITE_BLOCK_SIZE = 128
    WRITE_BLOCK_COUNT = 128
    APP_INFO_SIZE=1024
    CRC_SIZE = 4
    RETRY_NUMBER = 4
    # timeout big enough for erase cycle
    TIMEOUT_S = 2
    
    def read_info(self):
        length = self.APP_INFO_SIZE+self.CRC_SIZE
        time.sleep(0.15)
        for i in range(self.RETRY_NUMBER):
            print("Reading application info...")
            self.serial.flush()
            self.serial.write(self.COMMAND_INFO)
            data=self.serial.read(length)
            if len(data)== length:
                crc_data=int.from_bytes(data[self.APP_INFO_SIZE:(length)],\
                    'little')
                data=data[0:self.APP_INFO_SIZE]
                crc=self.crc_fun(data)
                if crc_data==crc:
                    print(data)
                    break
        
    def write_app(self):
        if self.file:
            print("Writing flash...")
            t_start = time.time()
            for cnt in range(self.WRITE_BLOCK_COUNT):
                data=self.file.read(self.WRITE_BLOCK_SIZE)
                if len(data)==0:
                    break
                elif len(data)<self.WRITE_BLOCK_SIZE:
                    # Fill remaining block area
                    data=data + b'\xFF'*(self.WRITE_BLOCK_SIZE-len(data))
                err=self.RESPONSE_OK
                for i in range(self.RETRY_NUMBER):
                    time.sleep(0.15)
                    self.serial.write(self.COMMAND_WRITE)
                    response=self.serial.read(1)
                    if response==self.RESPONSE_OK:
                        self.serial.write(data+self.crc_fun(data).to_bytes(4,'little'))
                        response=self.serial.read(1)
                        if response==self.RESPONSE_OK:
                            print("%1.1f%%" % ((cnt+1)/self.WRITE_BLOCK_COUNT*100),end="\r")
                            err=self.RESPONSE_OK
                            break
                        else:
                            err=response[0]
                if err!=self.RESPONSE_OK:
                    print("Write error 0x%X" % response[0])
                    break
            print("Elapsed %1.1fs" % (time.time()-t_start))
        else:
            print("No file to write")
        
    
    def erase_app(self):
        for cnt in range(self.WRITE_BLOCK_COUNT):
            time.sleep(0.15)
            self.serial.write(self.COMMAND_ERASE)
            response=self.serial.read(1)
            if response==self.RESPONSE_OK:
                print("Chip erased")
                return
        print("Erase error 0x%X" % response[0])

    def run_app(self):
        self.serial.write(self.COMMAND_RUN_APP)

    def __init__(self, serial_name, file_name):
        self.crc_fun=crcmod.mkCrcFun(0x104C11DB7, 0xFFFFFFFF, rev=False)
        try:
            self.serial = serial.Serial(serial_name, 1200, inter_byte_timeout=self.TIMEOUT_S)
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
            print("Close serial port " + self.serial.name)
            self.serial.close()

if __name__ == "__main__":
    if 1:
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
            bld.run_app()
    else:
        serial_name="COM45"
        bld = bootloader(serial_name, "")
        bld.read_info()
