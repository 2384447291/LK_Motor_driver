import numpy as np
import struct
from enum import Enum
from Communication import canMsg
from Communication import CanMsgCenter
from Communication import global_CanMsgCenter

TORQUE_MIN = -2000
TORQUE_MAX = 2000
Torque_const = 0.06*32/2000*10 #可能有问题

VEC_MIN = 0
VEC_MAX = 0xFFFF
VEC_const = 180/3.1415926*10
#单位dps

POS_MIN = 36000
POS_MAX = -36000
POS_const = 3.1415926*180*100
#单位d
class MotorMode(Enum):
    POS_MODE = 0
    SPEED_MODE = 1
    TORQUE_MODE = 2

def LIMIT_MIN_MAX(x,min,max):
    if x<=min:
        x=min
    elif x>max:
        x=max      
        
class LKmotor(object):  
    def __init__(self,ID,_Motor_mode): 
        self.MotorMode = _Motor_mode
        self.id = ID
        self.feedback_pos = float(0)
        self.feedback_vec = float(0)
        self.feedback_torque = float(0)
        global_CanMsgCenter.register(self)
        self.disable_motor()
        self.enable_motor()
        self.save_zero()
        self.write_PID(100,100,40,14,50,50)
        self.read_PID()



        ####在这里清零
        ################################
        # self.set_zero()
        ################################



        self.control_p_des = 0.0#rad
        self.control_v_des = 0.0#radps
        self.control_torque = 0.0
        self.control_v_limit = 0.2#radps

        self.control_pos_p = 0
        self.control_pos_i = 0   
        self.control_vec_p = 0
        self.control_vec_i = 0
        self.control_torque_p = 0
        self.control_torque_i = 0

    def enable_motor(self):
        global global_CanMsgCenter
        buf = np.array([0x88,0x00,0x00,0x00,0x00,0x00,0x00,0x00],np.uint8)
        tempmsg = canMsg(0x140+self.id,buf[0:8])
        global_CanMsgCenter.SendMsgQue.put(tempmsg)

    def disable_motor(self):
        global global_CanMsgCenter  
        buf = np.array([0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00],np.uint8)
        tempmsg = canMsg(0x140+self.id,buf[0:8])
        global_CanMsgCenter.SendMsgQue.put(tempmsg)      

    def stop_motor(self):
        global global_CanMsgCenter  
        buf = np.array([0x81,0x00,0x00,0x00,0x00,0x00,0x00,0x00],np.uint8)
        tempmsg = canMsg(0x140+self.id,buf[0:8])
        global_CanMsgCenter.SendMsgQue.put(tempmsg)      

    def save_zero(self):#对于未过减速器的编码器置0
        global global_CanMsgCenter  
        buf = np.array([0x91,0x00,0x00,0x00,0x00,0x00,0x00,0x00],np.uint8)
        tempmsg = canMsg(0x140+self.id,buf[0:8])
        global_CanMsgCenter.SendMsgQue.put(tempmsg)   

    def set_zero(self):#对于未过减速器的编码器置0
        global global_CanMsgCenter  
        buf = np.array([0x19,0x00,0x00,0x00,0x00,0x00,0x00,0x00],np.uint8)
        tempmsg = canMsg(0x140+self.id,buf[0:8])
        global_CanMsgCenter.SendMsgQue.put(tempmsg)   

    def read_PID(self):
        global global_CanMsgCenter  
        buf = np.array([0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00],np.uint8)
        tempmsg = canMsg(0x140+self.id,buf[0:8])
        global_CanMsgCenter.SendMsgQue.put(tempmsg) 
    
    def read_feedback(self):
        global global_CanMsgCenter  
        buf = np.array([0x9C,0x00,0x00,0x00,0x00,0x00,0x00,0x00],np.uint8)
        tempmsg = canMsg(0x140+self.id,buf[0:8])
        global_CanMsgCenter.SendMsgQue.put(tempmsg)        

    def read_encoder(self):
        global global_CanMsgCenter  
        buf = np.array([0x90,0x00,0x00,0x00,0x00,0x00,0x00,0x00],np.uint8)
        tempmsg = canMsg(0x140+self.id,buf[0:8])
        global_CanMsgCenter.SendMsgQue.put(tempmsg)  

    def read_circle_single(self):
        global global_CanMsgCenter  
        buf = np.array([0x94,0x00,0x00,0x00,0x00,0x00,0x00,0x00],np.uint8)
        tempmsg = canMsg(0x140+self.id,buf[0:8])
        global_CanMsgCenter.SendMsgQue.put(tempmsg)  

    def read_circle_multi(self):
        global global_CanMsgCenter  
        buf = np.array([0x92,0x00,0x00,0x00,0x00,0x00,0x00,0x00],np.uint8)
        tempmsg = canMsg(0x140+self.id,buf[0:8])
        global_CanMsgCenter.SendMsgQue.put(tempmsg)  

    def write_PID(self,target_pos_p,target_pos_i,target_vec_p,target_vec_i,target_torque_p,target_torque_i):
        buf=np.zeros((8,1),np.uint8)     
        buf[0] = 0x32
        buf[1] = 0x00
        buf[2] = np.uint8(target_pos_p)
        buf[3] = np.uint8(target_pos_i)
        buf[4] = np.uint8(target_vec_p)
        buf[5] = np.uint8(target_vec_i)
        buf[6] = np.uint8(target_torque_p)
        buf[7] = np.uint8(target_torque_i)
        canid = 0x140+self.id
        tempmsg = canMsg(canid,buf.reshape(8,))
        global_CanMsgCenter.SendMsgQue.put(tempmsg)   

    #LK电机位置有六种控制方式
        #0XA3 #0xA4（有速度限制）给定多圈位置定值
        #0XA5 #0xA6（加速度限制）给定增量（最高360）
        #0XA3 #0xA4（加速度限制）给定增量（上不封顶）
        #_tff范围-2000----2000，对应电流-32A----32A转矩常数为0.06
    def motorcontrol(self,_p_des,_v_des,_v_limit,_tff=0):
        global global_CanMsgCenter  
        if self.MotorMode == MotorMode["POS_MODE"]:
            _v_limit = _v_limit * VEC_const
            LIMIT_MIN_MAX(_v_limit,VEC_MIN,VEC_MAX)
            _v_limit = round(_v_limit)
            velocity=list(struct.pack("H",_v_limit))
            #print(_p_des)
            _p_des = _p_des * POS_const
            LIMIT_MIN_MAX(_p_des,POS_MIN,POS_MAX)
            _p_des = round(_p_des)
            position=list(struct.pack("i",_p_des))
            buf=np.zeros((8,1),np.uint8)     
            buf[0] = 0xA4
            buf[1] = 0x00
            buf[2] = velocity[0]
            buf[3] = velocity[1]
            buf[4] = position[0]
            buf[5] = position[1]
            buf[6] = position[2]
            buf[7] = position[3]
            canid = 0x140+self.id
            tempmsg = canMsg(canid,buf.reshape(8,))
            global_CanMsgCenter.SendMsgQue.put(tempmsg) 
        elif self.MotorMode == MotorMode["SPEED_MODE"]:
            _v_des = _v_des * VEC_const *100
            _v_des = round(_v_des)
            velocity=list(struct.pack("i",_v_des))
            buf=np.zeros((8,1),np.uint8)     
            buf[0] = 0xA2
            buf[1] = 0x00
            buf[2] = 0x00
            buf[3] = 0x00
            buf[4] = velocity[0]
            buf[5] = velocity[1]
            buf[6] = velocity[2]
            buf[7] = velocity[3]
            canid = 0x140+self.id
            tempmsg = canMsg(canid,buf.reshape(8,))
            global_CanMsgCenter.SendMsgQue.put(tempmsg)             
        # elif self.MotorMode == MotorMode["TORQUE_MODE"]:
            # _tff = _tff * Torque_const
            # LIMIT_MIN_MAX(_tff,TORQUE_MIN,TORQUE_MAX)
            # _tff = round(_tff)
            # torque=list(struct.pack("h",_tff))
            # global_CanMsgCenter.LKmotorTorqueBuffer[self.id*2-2] = torque[0]
            # global_CanMsgCenter.LKmotorTorqueBuffer[self.id*2-1] = torque[1]
            # global_CanMsgCenter.need_send_torque_message = True

    def resolve_feedback(self,cmd,packet):
        if cmd == 0x11:#接收成功
            if packet[7] == 0x30:#读取PID
                self.control_pos_p = np.uint8(packet[9])
                self.control_pos_i = np.uint8(packet[10])
                self.control_vec_p = np.uint8(packet[11])
                self.control_vec_i = np.uint8(packet[12])
                self.control_torque_p = np.uint8(packet[13])
                self.control_torque_i = np.uint8(packet[14])
                print("successful change pid, pos_p = %d, pos_i = %d, vec_p = %d, vec_i = %d, torque_p = %d, torque_i = %d" %(self.control_pos_p,self.control_pos_i,self.control_vec_p,self.control_vec_i,self.control_torque_p,self.control_torque_i))
            elif packet[7] == 0xA1 or packet[7] == 0xA4 or packet[7] == 0xA2 or packet[7] == 0x9C:#读取反馈
                    Hex_Data_np = [hex(i) for i in packet[7:15].reshape(8,)]
                    #print(Hex_Data_np)   
                    self.feedback_pos = struct.unpack("H",packet[13:15])#单圈
                    self.feedback_vec = struct.unpack("h",packet[11:13])#dps没过减速器
                    self.feedback_torque = struct.unpack("h",packet[9:11])#-2048-2048
                    self.feedback_pos = self.feedback_pos[0]/(0xFFFF)*3.1415926*2
                    self.feedback_vec = self.feedback_vec[0]/180*3.1415926/10
                    self.feedback_torque = self.feedback_torque[0]*Torque_const
                    if self.feedback_pos > 3.1415926535:
                       self.feedback_pos = self.feedback_pos - 3.1415926535*2 
            elif packet[7] == 0x90:#没过减速器（编码器）
                    self.feedback_pos = struct.unpack("H",packet[13:15])
                    self.feedback_vec = struct.unpack("H",packet[11:13])
                    self.feedback_torque = struct.unpack("H",packet[9:11])
                    self.feedback_pos = self.feedback_pos[0]/(0xFFFF)*3.1415926*2
                    self.feedback_vec = self.feedback_vec[0]/(0xFFFF)*3.1415926*2
                    self.feedback_torque = self.feedback_torque[0]/(0xFFFF)*3.1415926*2
            elif packet[7] == 0x94:#单圈
                    self.feedback_pos = struct.unpack("I",packet[11:15])
                    self.feedback_pos = self.feedback_pos[0]*0.01*0.1/360*3.14
            elif packet[7] == 0x92:#多圈
                    pand_packet = np.zeros((8,1),np.uint8) 
                    pand_packet[0:7] = packet[8:15]
                    temp_7_bit = struct.unpack("Q",pand_packet)
                    if temp_7_bit[0] > 0x7FFFFFFFFFFFFF:
                        pand_packet[7] = 0xFF
                    self.feedback_pos = struct.unpack("q",pand_packet)
                    self.feedback_pos = self.feedback_pos[0]*0.01*0.1/360*3.14                 
        elif cmd == 0x02:#发送失败             
            print('send error Motor %d' %(self.id))     
            Hex_Data_np = [hex(i) for i in packet[7:15].reshape(8,)]
            print(Hex_Data_np)  
    def set(self,_p_des,_v_des):
         self.control_p_des=_p_des
         self.control_v_limit=_v_des
         self.control_v_des=_v_des