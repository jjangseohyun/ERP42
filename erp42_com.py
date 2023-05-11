#!/usr/bin/env python3
# -*- coding: utf-8 -*-한글 주석 선언
'''
ROS KINETIC, python2.7, Ubuntu 16.04

ERP42 UPPER to PCU 프로토콜

Specification : RS232
DATA Ordering :  Big Endian (UPPER to PCU)
Cycle Time : 20 msec
Baud:115200, parity : None, Stop : 1
언맨드 제어기 (이하 PCU)
USER PC or 제어기 (이하 UPPER)

Packet : 14Bytes
byte name : [S, T, X, AorM, E-STOP, GEAR, SPEED0, SPEED1, STEER0, STEER1, BRAKE, ALIVE, ETX0, ETX1]

STX -> Start bytes// [0x53, 0x54, 0x58]
AorM -> Auto or Manual// manual mode : 0x00 , auto mode : 0x01
ESTOP -> Emergency STOP// E-STOP Off : 0x00, E-STOP On : 0x01
GEAR -> F,N,B gear// forward drive : 0x00, neutral : 0x01, backward drive : 0x02
SPEED -> actual speed (KPH) * 10// 0 ~ 200
STEER -> actual steering dgree (dgree) * 71, 오차율 : 4%, negative is left steer// Right : 0 ~ 2000, Left : -2000 ~ 0 
BRAKE -> 1 : no braking, 150 : full braking// 1 ~ 150
ALIVE -> increasing each one step//1,2,3,4,5......253,254,255,1,2,3.....
ETX -> end bytes//[0x0D, 0x0A]

Packet Default
byte name : [[0x53, 0x54, 0x58], 0x01, 0x00, GEAR, SPEED0, SPEED1, STEER0, STEER1, BRAKE, 0, (0x0D, 0x0A)]
'''
import rospy
from erp42_serial.msg import ESerial
from erp42_serial.msg import Evelocity
from std_msgs.msg import Float64
import os
import struct
import time
import serial

print('serial begin')  

# Set a PORT Number & baud rate(시리얼통신을 위한 설정코드)
PORT = '/dev/ttyUSB0' #코드번호
BaudRate = 115200  #전송속도
ERPS = serial.Serial(PORT,BaudRate) #serial모드의 Seria클래스 사용=>포트경로와 속도를 받게됨

#erp42serial클래스 정의
class erp42Serial :             
    STX = (0x53, 0x54, 0x58)    #패킷시작(Start Of Text)
    AorM = 0x01  #defolat 0x01 A
    ESTOP = 0x00 #defloat 0x00 off
    GEAR = 0  #동적 파라미터
    SPEED = 0 #동적 파라미터
    STEER = 0 #동적 파라미터
    BRAKE = 0 #동적 파라미터
    ALIVE = 0                   #패킷의 수명주기를 관리하는데 사용됨
    ETX = (0x0D, 0x0A)          #패킷의 끝(End Of Text)
    
    '''
    def __init__(self, G, SP, ST, B):
        self.GEAR = G  #동적 파라미터
        self.SPEED = SP #동적 파라미터
        self.STEER = ST #동적 파라미터
        self.BRAKE = B #동적 파라미터
    '''
    
#setparams=클래스 내부에 구현된 메소드(인스턴스변수를 사용하기위해 사용)
#setParams함수 정의를 통해 기어, 속도, 스티어링, 브레이크 정의
    def setParams(self, G=0, SP=0, ST=0, B=0): #__init__과 비슷하지만, class를 초기화 할필요없다.
        self.GEAR = G  #동적 파라미터 forward drive : 0x00, neutral : 0x01, backward drive : 0x02
        self.SPEED = SP #동적 파라미터 actual speed (KPH) * 10// 0 ~ 200
        self.STEER = ST #동적 파라미터  Right : 0 ~ 2000, Left : -2000 ~ 0 
        self.BRAKE = B #동적 파라미터 1 : no braking, 150 : full braking// 1 ~ 150

#패킷이 일정 주기로 전송되어야 하기 때문에
#ALIVE함수값은 0부터 255사이의 값
    def Alive(self): 
        if self.ALIVE <= 254 :      #ALIVE값이 254이하라면 1씩증가시킴
            self.ALIVE += 1
        else :
            self.ALIVE = 0          #ALIVE값이 255라면 0으로 초기화
        

#struct함수 정의 
       
    def Struct(self):
        s = struct.pack('!BBBBBBHhBBBB',self.STX[0], self.STX[1], self.STX[2], self.AorM, self.ESTOP, self.GEAR, self.SPEED, self.STEER, self.BRAKE, self.ALIVE, self.ETX[0], self.ETX[1])
        tu = struct.unpack('!BBBBBBBBBBBBBB', s)
        return tu

    
##-------------------------------------------------##

              #GEAR, SPEED, STEER, BRAKE
            
Sclass = erp42Serial()

'''
g = input("기어 전진 = 0, 중립 = 1, 후진 = 2 : ")
spd = input("속도 0 ~ 200 : ")
ster = input("스티어링 -2000 ~ 2000 : ")
brk = input("브레이크 0 ~ 150 : ")
'''
g = 0
spd = 0
ster = 0
brk = 1
##spd = input("속도 0 ~ 200 : ")
##ster = input("스티어링 -2000 ~ 2000 : ")

#serialwrite함수 정의(4개의 인자)

def serialwrite( g, spd, ster, brk):
    Sclass.Alive()
    Sclass.setParams(g, spd, ster, brk)
    a = Sclass.Struct()
    #print(a)
    ERPS.write(a)

LISEN = []
List = []
turn = 0
def serialread(): #serialread 함수정의
    global LISEN  #global 전역변수
    global List
    global turn
    if ERPS.readable():             #시리얼 포트로부터 읽을 수 있는 데이터가 있는지 확인
        LINE = ERPS.read()          #ERPS객체의 read메소드를 사용해 데이터를 읽어드린다.
        L = struct.unpack('!B', LINE)   #unpack 함수를 통해 1바이트형태로 언패킹된다.
        a = L[0]                           #?
        if a == 0x53 and turn == 0: #STX값이 0x53일경우 0을 반환
            turn = 1
        elif a == 0x54 and turn == 1 :
            turn = 2
        elif a == 0x58 and turn == 2 :
            turn = 3
        elif turn >= 3 and turn <= 15: #turn 변수가 3이상 15이하 일땐
            turn += 1                   #변수가 1씩 증가
            LISEN.append(a)     #패킷 데이터값을 리스트에 추가
        elif turn > 15 :        #15보다 크면
            List = LISEN        #LISTEN리스트를 List에 할당
            turn = 0            #변수를 0으로 초기화
            LISEN = []
            #print(List)
    return List


#encorder list num : 8 9 10 11 
time_turn = 0       #시간간격
time0 = 0           #이전 시간 값 저장
L1 = 0              #왼쪽바퀴 주행 거리
L2 = 0              #오른 쪽 바퀴의 주행거리
msg2 = Evelocity()  #메시지 타입의 변수, 바퀴 속도 값
msg3 = Float64() #for pid control input #바퀴 주행거리 값


def encoder_speed(L):   #
    global time_turn    #전역 변수 선언
    global time0
    global L1
    global L2
    D = 0.01665 # mitter  [1+ = 16.65mm]
    time_set = 0.1      #바퀴 속도 계산을 위해 시간 간격을 0.1초로 설정
    if len(L) == 13 :           #L의 길이가 13일때 주행거리 계산
        #print(L[8],L[9],L[10],L[11])
        L = (L[8]+L[9]*256+L[10]*256**2+L[11]*256**3)*D
        if time_turn == 0:              #변수값이 0이면
            time0 = time.time()         #이전 시간 값을 저장 후
            time_turn = 1               #변수값을 1로 변경
            L1 = L                      
        elif time_turn == 1 and time.time() - time0 >= time_set:   #변수값이 1일때 그리고 time set이 0.1일때
            L2 = L - L1    #바퀴속도 걔산
            V = L2/time_set #m/s           
            msg2.velocity = V; msg3.data = V      
            pub1.publish(msg2); pub2.publish(msg3)
            print(V)            #계산된 속도값 출력
            time_turn = 0



rospy.init_node('erp42_com')

'''
msg = ESerial()
msg.gear = 1
msg.speed = 0
msg.steer = 0
msg.brake = 0
msg.shutdown = 0
'''
g = 1
spd = 0
ster = 0
brk = 0
#serialwrite(msg.gear, msg.speed, msg.steer, msg.brake)
#serialwrite(g, spd, ster, brk)
def callback(msg):   #callback함수 정의
    global g         #전역변수 선언
    global spd         
    global ster        
    global brk              
    g = msg.gear           #msg는 topic으로부터 받아들인 메시지를 의미
    spd = msg.speed     
    ster = msg.steer
    brk = msg.brake


sub = rospy.Subscriber('erp42_serial', ESerial, callback)
pub1 = rospy.Publisher('erp42_velocity', Evelocity, queue_size=100) #erp42_velocity
pub2 = rospy.Publisher('state', Float64, queue_size=100) #erp42_velocity
rate = rospy.Rate(1000)
while not rospy.is_shutdown():
    serialwrite(g, spd, ster, brk)
    L = serialread()
    encoder_speed(L)
    rate.sleep()
    
    
    
    
    #print("gear : %d / speed : %d / steer : %d / brake : %d / velocity : %f" %(g, spd, ster, brk, msg2.velocity))
    
    
    
    
    
    

