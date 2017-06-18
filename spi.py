import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)

GPIO.setup(32, GPIO.IN)
import spidev
import time

def dmp(A):
    s = ""
    cnt = 0
    for a in A:
        s+="%0.2X " % a
        cnt=(cnt+1)%32
        if cnt == 0:
            s+="\n"
    print s
write1 = [0x85E0000B]
write2 = [0x8A079600]
#write3 = [0x81F804AF]
write3 = [0x81f80006]
write4 = [0x40000000]

print GPIO.input(32)
spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz = 1000000000
print "Writing 1"
spi.writebytes(write1)
print "Writing 2"
spi.writebytes(write2)

print "Writing 3"
spi.writebytes(write3)
print "Writing 4"
spi.writebytes(write4)
while True:
    print "Reading..." 
    if (GPIO.input(32) == 0): 
        dmp(spi.readbytes(11*32))
    time.sleep(1)


spi.close()
