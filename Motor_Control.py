import time
from roboclaw_3 import Roboclaw

rc = Roboclaw("/dev/ttyACM0",115200)
address = 0x80

def angle2encoder(angle):
    max_encoder_reading = 18140.64
    encoder_low = -1 * int((angle / 360) * max_encoder_reading)
    encoder_high = int((angle / 360) * max_encoder_reading) + 1
    return encoder_low, encoder_high

def displayspeed():
    enc1 = rc.ReadEncM1(address)
    speed1 = rc.ReadSpeedM1(address)
    if (enc1[0]==1):
        print("Encoder1:"),
        print (enc1[1]),
        angle = enc1[1] *360/18140.64
        print ("Angle")
        print(angle)
        # print (format(enc1[2],'02x')),
    else:
        print ("failed")


def motor_control(angle):
    
    speed_val = 3000
    accel = 12000
    deccel = 12000

    lower, higher = angle2encoder(angle)
    for N in range(lower, higher):
        rc.SpeedAccelDeccelPositionM1(address, accel, speed_val, deccel, N, 0)
        time.sleep(1)
        displayspeed()           
        current_pos = rc.ReadEncM1(address)[1]
        while True:
            current_pos = rc.ReadEncM1(address)[1]
            if abs(current_pos - N) >= 0.5:
                rc.SpeedAccelDeccelPositionM1(address, accel, speed_val, deccel, N, 0)
                time.sleep(1)
                displayspeed()
                return False
            else:
                return True