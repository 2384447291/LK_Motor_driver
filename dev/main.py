#电机通讯的必要组件
from Communication import CanMsgCenter
from Communication import USB2CAN
from Motor import MotorMode
from Communication import global_CanMsgCenter
import Motor
import Planer
#你最爱的np
import numpy as np 

#绘图组件
import matplotlib
import matplotlib.pyplot as plt  
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from tkinter import *

#多线程组件
from threading import Thread,Lock
import threading
import time

#挂墙时钟
t0 = 0.0
dt= 0.0001

# 机器人参数和规划器
arg=np.mat([0,100,100,0,0,20.49])       # 机器人参数 = [link1 link2 link3 j1bias j2bias j3bias]
n1=40000                                  # 持续间隔数；持续时间 = n * dt
n2=30000
di=1800
pll=Planer.testlineplaner(Planer.finger_v2_ik(arg),n2)
# rpll=Planer.resetplaner(pll.getJointAng(1),n1)


def Motor_control_thread(_interface):
    while True:
        #虚假的1ms（差不多得了，到时候有时序要求，上linux或者单片机吧）
        # time.sleep(0.01)
        #收发信息的驱动
        is_sending = global_CanMsgCenter.UpdateMassage(_interface)
        global_CanMsgCenter.RecieveMassage(_interface)   
#在这里写代码//不要动t0很危险
#------------------------------------------------------------------------------s-----------------------------        
#-----------------------------------------------------------------------------------------------------------  
#-----------------------------------------------------------------------------------------------------------  
#-----------------------------------------------------------------------------------------------------------  
        #运动算法组件  
        global Motor1,Motor2,Motor3        
        # global Motor1          
        i=(int)((time.time()-t0)/dt)


        if i>0 and i<=n1:
            Motor1.set(pll.getJointAng(1)[0,0],1)
            Motor2.set(pll.getJointAng(1)[0,1],1)
            Motor3.set(pll.getJointAng(1)[0,2],1)
        else:# i>p:
            Motor1.set(pll.getJointAng(i-n1+di)[0,0],np.abs((pll.getJointAng(i-n1)[0,0]-pll.getJointAng(i+1-n1)[0,0])/dt-0.001))
            Motor2.set(pll.getJointAng(i-n1+di)[0,1],np.abs((pll.getJointAng(i-n1)[0,1]-pll.getJointAng(i+1-n1)[0,1])/dt-0.001))
            Motor3.set(pll.getJointAng(i-n1+di)[0,2],np.abs((pll.getJointAng(i-n1)[0,2]-pll.getJointAng(i+1-n1)[0,2])/dt-0.001))

        # #============================复位用======================
        # global Motor1
        # Motor1.set(0,6)


#在这里写代码
#-----------------------------------------------------------------------------------------------------------        
#-----------------------------------------------------------------------------------------------------------  
#-----------------------------------------------------------------------------------------------------------  
#----------------------------------------------------------------------------------------------------------- 
#数据采样组件
def update_data():
    while True:
        #虚假的1ms（差不多得了，到时候有时序要求，上linux或者单片机吧）
        # time.sleep(0.01)
        global time_mem,data_mem,t0
        global global_CanMsgCenter
        #更新数据    
        #顺序为p_set,p_now,v_set,v_now,torque
        append_data = np.zeros((1,1),np.float64) 
        for motor in global_CanMsgCenter.registedmotor:
            append_data = np.append(append_data,[motor.control_p_des,motor.feedback_pos,
                                     motor.control_v_des,motor.feedback_vec,
                                     motor.control_torque,motor.feedback_torque])
        append_data = np.delete(append_data, 0) 
        append_data = np.reshape(append_data,(-1,1))
        datalock.acquire()
        time_mem = np.append(time_mem,time.time()-t0)
        data_mem = np.append(data_mem,append_data,axis=1)
        if time_mem.size > 500:#差不多10s500个数据
            time_mem = np.delete(time_mem, 0) 
            data_mem = np.delete(data_mem, 0, 1)   
        datalock.release()
        
def video_loop():  #动态图像现实窗口
    global time_mem,data_mem
    global global_CanMsgCenter
    datalock.acquire()
    temp_time_mem = time_mem
    temp_data_mem = data_mem
    datalock.release()
    f.clf()
    f.subplots_adjust(hspace=0.4, wspace = 0.4)
    plotlist = []
    for i in range(1,global_CanMsgCenter.num_motor+1):
        for j in range(1,4):
            temp_plot = f.add_subplot(global_CanMsgCenter.num_motor,3,i*3+j-3)
            plotlist.append(temp_plot)
    #绘图
    i = 0
    for a in plotlist:
        a.plot(temp_time_mem, temp_data_mem[i*2,:], color='red',linewidth=1.0)#实际
        a.plot(temp_time_mem, temp_data_mem[i*2+1,:], color='black',linewidth=1.0)#反馈
        a.tick_params(axis='x',
                 labelsize=4, # y轴字体大小设置
                  ) 
        a.tick_params(axis='y',
            labelsize=4, # y轴字体大小设置
            ) 
        a.set_xlim(xmin = temp_time_mem[0], xmax = temp_time_mem[-1])
        i_title = i%3
        if  i_title == 0:
            a.set_title('Motor{}Pos'.format(i//3), fontsize=6)
        elif i_title == 1:
            a.set_title('Motor{}Vec'.format(i//3), fontsize=6)
        elif i_title == 2:
            a.set_title('Motor{}Torque'.format(i//3), fontsize=6) 
        i+=1  
    canvas.draw()
    root.after(10, video_loop)

if __name__ == '__main__' :
    #挂墙时间
    t0 = time.time()
    #定义USB
    m_USB = USB2CAN()


    #定义电机(注册4个以上电机UI会爆，懒得修了，你自己手动改吧)=============================
    Motor1 = Motor.LKmotor(1,MotorMode["POS_MODE"])
    Motor2 = Motor.LKmotor(2,MotorMode["POS_MODE"])
    Motor3 = Motor.LKmotor(3,MotorMode["POS_MODE"])
    
    #开启电机控制线程
    Motor_threading=threading.Thread(target=Motor_control_thread,args=[m_USB],name='Motor_control_thread')
    Motor_threading.daemon = True
    Motor_threading.start()    






    #---------------------------------------------------画图----------------------------------------------
    #开启数据更新线程
    Data_update_threading=threading.Thread(target=update_data,name='Update_data_thread')
    Data_update_threading.daemon = True
    Data_update_threading.start()      
    #绘图主线程
    data_mem = np.zeros((global_CanMsgCenter.num_motor*6,1),np.float64) 
    time_mem = np.zeros((1,1),np.float64) 
    datalock = Lock()
    root = Tk()
    root.title("Set title")
    root.geometry('1000x500')
    
    """
    图像画布设置
    """
    panel = Label(root)  # initialize image panel
    panel.place(x=0,y=0,anchor='nw')
    root.config(cursor="arrow")
    
    matplotlib.use('TkAgg')
    f = Figure(figsize=(7, 5/(5-global_CanMsgCenter.num_motor)), dpi=130)
    canvas = FigureCanvasTkAgg(f, master=root)
    canvas.draw()
    canvas.get_tk_widget().place(x=0,y=0 ,anchor='nw')
    video_loop()
    root.mainloop()
    #---------------------------------------------------画图----------------------------------------------
        