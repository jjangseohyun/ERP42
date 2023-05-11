#!/usr/bin/env python3
# -*- coding: utf-8 -*-한글 주석 선언

'''
Specification : RS232
DATA Ordering : Little Endian (PCU to UPPER) / Big Endian (UPPER to PCU)
Cycle Time : 20 msec
Baud:115200, parity : None, Stop : 1
언맨드 제어기 (이하 PCU)
USER PC or 제어기 (이하 UPPER)

UPPER to PCU: pc에서 차로보내는 값
[S,T,X,AorM,E-STOP,GEAR,SPEED0,SPEED1,STEER0,STEER1,BRAKE,ALIVE,ETX0,ETX1]

PCU to UPPER: 차에서 PC로 오는 값
[S,T,X,AorM,E-STOP,GEAR,SPEED0,SPEED0,STEER0,STEER1,BRAKE,ALIVE,ETX0,ETX1]

STX -> Start of TEXT// [0x53, 0x54, 0x58]
AorM -> Auto or Manual, 0x00 : manual mode , 0x01 : auto mode
ESTOP -> Emergency STOP, 0x00 : E-STOP Off, 0x01 : E-STOP On
GEAR -> 0x00 : forward drive, 0x01 : neutral, 0x02 : backward drive
SPEED -> actual speed (KPH) * 10 // 0~200
STEER -> actual steering dgree (dgree) * 71, 오차율 : 4%, negative is left steer // -2000~2000
BRAKE -> 1 : no braking, 150 : full braking // 1~150
ENC -> encoder counting // -2^31~2^31
ALIVE -> increasing each one step // 0~255
ETX: End of TEXT // [0x0D,0x0A]

Packet Default
byte name : [[0x53, 0x54, 0x58], 0x01, 0x00, GEAR, SPEED0, SPEED1, STEER0, STEER1, BRAKE, 0, [0x0D, 0x0A]]
'''

import rospy
from erp42_serial.msg import ESerial
from erp42_serial.msg import Evelocity
from std_msgs.msg import Float64
import struct
import time
import serial
from math import pi
# Set a PORT Number & baud rate
PORT = '/dev/ttyUSB0'
BaudRate = 115200
SERIAL = serial.Serial(PORT,BaudRate)


class ERP42_Serial_Data:                #ERP42_Serial_Data 클래스 선언
    def __init__(self):                 #init 클래스 속성값 초기화
        self.STX = (0x53, 0x54, 0x58) #default !BBB 시작비트를 나타냄
        self.AorM = 0x01 #default B         
        self.ESTOP = 0x00 #B
        self.GEAR = 0 #0,1,2 B
        self.SPEED = 0 #0~200 H 
        self.STEER = 0 #-2000~2000 h
        self.BRAKE = 1 #1~150 B
        self.ALIVE = 0 #0~254 B
        self.ETX = (0x0D,0x0A) #default BB
        
    def set_params(self,GEAR=1,SPEED=0,STEER=0,BRAKE=1):
        self.GEAR = GEAR #0,1,2
        self.SPEED = SPEED #0~200
        self.STEER = STEER #-2000~2000
        self.BRAKE = BRAKE #0~254
        
        if self.ALIVE <= 254:   #254보다 작다면
            self.ALIVE = self.ALIVE + 1 #값을 1씩더하며
        else:
            self.ALIVE = 0          #255가 된다면 다시 0으로 설정
            
        self.data = [self.STX[0], self.STX[1], self.STX[2], self.AorM, self.ESTOP, 
                     self.GEAR, self.SPEED, self.STEER, self.BRAKE, self.ALIVE, self.ETX[0], self.ETX[1]]

       
    def data_struct(self):
        self.data_pack = struct.pack('!BBBBBBHhBBBB', *self.data)
        
        return self.data_pack

ESD = ERP42_Serial_Data()


def Serial_write(gear,speed,steer,brake):
   
    ESD.set_params(gear,speed,steer,brake)
    data_pack = ESD.data_struct()
    SERIAL.write(data_pack)
    #print(data_pack) 
    #print(f'speed:{speed}, steer:{steer}, brake:{brake}, gear:{gear}')

#msg2, msg3, tmp변수정의
msg2 = Evelocity()      #차량의 현재속도
msg3 = Float64() #for pid control input
tmp = 0         #전역변수

def Serial_read():
    PPR = 25856 #Pulse Per Revolution   회전당 차량의 바퀴 엔코더에 생성되는 펄스 수
    r = 0.27 #radius = 270mm 바퀴의 반지름

    global tmp
    if SERIAL.readable():
        data1 = SERIAL.read(18) #read 18byte
        data1 = struct.unpack('<BBBBBBHhiBBBB', data1)
        #print(f'data recieved:{data1}')
        time1 = time.time()
        encoder1 = data1[8]
        #print(encoder1)

        data2 = SERIAL.read(18) #read 18byte
        data2 = struct.unpack('<BBBBBBHhiBBBB', data2)
        time2 = time.time()
        encoder2 = data2[8]
        #print(encoder2)
        
        if time1 != time2 and encoder1 == encoder2 and tmp != 0:
            enc_diff = encoder2 - tmp
        else:
            enc_diff = encoder2 - encoder1
        
        #print(encoder1, tmp, encoder2, enc_diff)
        
        time_diff = time2 - time1
        
        #현재속도
        current_speed = (2 * pi * enc_diff * r) / (PPR * time_diff) #공식을 통한 현재속도 계산
        # velocity = (2pi * Pulse Count * r) / (Pulse Per Revolution * time_diff)
        print(f'current speed:{current_speed} km/h')       
        msg2.velocity = current_speed
        msg3.data = current_speed
        pub1.publish(msg2); pub2.publish(msg3)
        tmp = encoder2 # save encoder1 temporary


    
    
    
    
msg = ESerial()
gear = 1
speed = 0
steer = 0
brake = 1

def serial_callback(msg):   #메시지를 받아서 데이터 처리하는 callback함수
    global gear         #전역변수 선언
    global speed
    global steer
    global brake
    gear = msg.gear     #값 저장
    speed = msg.speed
    steer = msg. steer
    brake = msg.brake
    print(f'speed:{speed}, steer:{steer}, brake:{brake}, gear:{gear}')  #현재값출력

    
rospy.Subscriber('erp42_serial', ESerial, serial_callback)
pub1 = rospy.Publisher('erp42_velocity', Evelocity, queue_size=100) #erp42_velocity
pub2 = rospy.Publisher('state', Float64, queue_size=100) #erp42_velocity
    

if __name__ == '__main__':
    rospy.init_node('erp42_serial')
    print("Start ERP-42")
    rate = rospy.Rate(1000)
    
    while not rospy.is_shutdown():
        Serial_write(gear,speed,steer,brake)
        Serial_read()
        rate.sleep()

    
