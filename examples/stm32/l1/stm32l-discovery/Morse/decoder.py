import sys
import serial
import re
port = serial.Serial('/dev/ttyUSB0', 115200)

MORSE_CODE_DICT = { '01':'A', '1000':'B', 
                    '1010':'C', '100':'D', '0':'E', 
                    '0010':'F', '110':'G', '0000':'H', 
                    '00':'I', '0111':'J', '101':'K', 
                    '0100':'L', '11':'M', '10':'N', 
                    '111':'O', '0110':'P', '1101':'Q', 
                    '010':'R', '000':'S', '1':'T', 
                    '001':'U', '0001':'V', '011':'W', 
                    '1001':'X', '1011':'Y', '1100':'Z', 
                    '01111':'1', '00111':'2', '00011':'3', 
                    '00001':'4', '00000':'5', '10000':'6', 
                    '11000':'7', '11100':'8', '11110':'9', 
                    '11111':'0'} 

def decode(line):
    i = 0
    while (chr(line[i]) != '1'):
        i+=1
    line = line[i+1:]
    return MORSE_CODE_DICT[line.decode('utf-8')]

while True:
    line = port.readline()
    line = line[:-2]
    if re.match(b"0{7}1", line):
        pass
    elif re.match(b"[01]{8}", line):
        symbol=decode(line)
        print ("sym:",symbol)
    else:
        print ("lin:",line)
    # elif line == 'Too much symbols':
    #     print line
