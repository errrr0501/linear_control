#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import pandas as pd
import math
from math import cos ,sin, acos ,pi, radians, degrees
import matplotlib.pyplot as plt
from matplotlib import pyplot as plt
from matplotlib import animation 
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


    

def Line_Traj_3_DOF():
    # varargin = Line_Traj_3_DOF.varargin
    # nargin = Line_Traj_3_DOF.nargin

   
    ## parameter for robot manipulator hardware
    DOF=3
    Link=np.array([10,30,30])
    
    ParameterGear = np.array([[110,105,80]])
    
    
    ## parameter for trajectory planning
    CurrentPos = np.array([-20,10,0])
    GoalPos = np.array([-20,40,20])
    
    
    #     CurrentPos = [  -20 40 5 ];        # start pos
#     GoalPos    = [  -20 40 20 ];        # target pos
    
    
    VelStart = 0
    VelDrv = 50
    VelAcc = 100
    VelTick = 0.04
    
    max_mov_dis=np.dot(VelDrv,VelTick)
    Error=np.linalg.norm(GoalPos - CurrentPos)
    Vector=(GoalPos - CurrentPos) / Error
    
    VelState = ('VEL_IDEL')
    
    VelNow = 0
    
    VelDis = 0
    
    #max speed's displacement   50(cm/s)*0.05(s) = 2.5 cm/sample_time
    
    ## calculate
    CurrentAngle = IK(CurrentPos,Link)
    Info=FK(DOF,CurrentAngle,Link)
    GoalAngle = IK(GoalPos,Link)
    Info=FK(DOF,GoalAngle,Link)
    Path= Current
    
    Past_RPM=zeros(1,3)
    MotorACC=zeros(1,3)
    Rec_P=PathPosT
    Rec_ErrP=PathPos
    Rec_V=0
    Rec_DD=0
    Rec_Ang=PathAngleT
    Rec_RPM=zeros(3,1)
    Rec_Acc=zeros(3,1)
    
    while (abs(Error) > 1e-05):

        #         
        #record each axis and cal the pos of now
        Feedback_Ang=PathAngle
        Info=FK(DOF,Feedback_Ang,Link)
        DrawRobotManipulator(DOF,Info.JointPos,Info.JointDir,1,np.array([- 145,30]))
        title('RRR Manipulator')
        Vel=CalculateMovementDistance(Vel,Error)
        PathPos = PathPos + np.dot(Vector,VelDis)
        PathAngle = IK(PathPos,Link)
        Ang_V=np.dot((PathAngle - Feedback_Ang),pi) / 180 / VelTick
        RPM=np.dot(60,Ang_V) / (np.dot(2,pi))
        Error=np.linalg.norm(GoalPos - PathPos)
        pause(VelTick)
        MotorACC=(np.multiply(Ang_V,ParameterGear) - Past_RPM) / VelTick
        Past_RPM=np.multiply(Ang_V,ParameterGear)
        Rec_P=np.array([Rec_P,Info.Pos])
        Rec_V=np.array([Rec_V,VelNow])
        Rec_Ang=np.array([Rec_Ang,PathAngleT])
        Rec_RPM=np.array([Rec_RPM,RPM.T])
        Rec_Acc=np.array([Rec_Acc,MotorACC.T])

    
    
    # record2
    Rec_P=np.array([Rec_P,PathPosT])
    Rec_Ang=np.array([Rec_Ang,PathAngleT])
    Rec_V=np.array([Rec_V,0])
    Rec_RPM=np.array([Rec_RPM,zeros(3,1)])
    Rec_Acc=np.array([Rec_Acc,zeros(3,1)])
    
    figure(1)
    plot3(Rec_P(1,arange()),Rec_P(2,arange()),Rec_P(3,arange()),'bo')
    t=np.dot((arange(1,size(Rec_V,2))),VelTick)
    figure(2)
    plot(t,Rec_V,'-bo')
    axis(np.array([min(t),max(t),min(Rec_V),max(Rec_V) + 2]))
    final_pos=Rec_P(arange(),end()).T
    
    #     DrawAngle( t , Rec_Ang );
    DrawRPM(t,Rec_RPM)
    #     DrawACC( t, Rec_Acc );
    
    return

    
    ## status ###################
    
def CalculateMovementDistance(Vel,Error):
    # varargin = CalculateMovementDistance.varargin
    # nargin = CalculateMovementDistance.nargin

    VelDis = 0
    if (cmp(VelState,'VEL_IDEL')):
        VelNow = 0
        VelState = 'VEL_ACC'
        VelDis = np.dot(VelNow,VelTick)
    else:
        if (cmp(VelState,'VEL_ACC')):
            tmp=VelNow
            VelNow = tmp + (np.dot(VelAcc,VelTick))
            VelDis = np.dot(np.dot((tmp + VelNow),VelTick),0.5)
            IsDec,Vel=IsDeceleration(Vel,Error,nargout=2)
            #         IsDec=0;
            if (IsDec == 0):
                if (VelNow >= VelDrv):
                    VelNow = VelDrv
                    VelState = 'VEL_KEEP'
                    VelDis = np.dot(tmp,VelTick) + (np.dot(VelTick,(VelNow - tmp))) / 2
            else:
                VelState = 'VEL_DEC'
            #==
        else:
            if (cmp(VelState,'VEL_KEEP')):
                IsDec,Vel=IsDeceleration(Vel,Error,nargout=2)
                if (IsDec == 0):
                    VelNow = VelDrv
                    VelDis = np.dot(VelNow,VelTick)
                else:
                    VelState = 'VEL_DEC'
            else:
                if (cmp(VelState,'VEL_DEC')):
                    VelNow = VelNow - dot(VelAcc,VelTick)
                    if (VelNow <= VelStart):
                        VelNow = copy(VelStart)
                    VelDis = np.dot(VelNow,VelTick) + (np.dot(VelAcc,VelTick ** 2)) / 2.0
                else:
                    print('err')
    
    if (VelDis > abs(Error)):
        VelDis = abs(Error)
    
    
    if (Error < 0):
        VelDis = np.dot(-1.0,VelDis)
    
    return Vel
    

    
    ## trapezoid speed curve ###############
    
def IsDeceleration(Vel=None,Error=None,*args,**kwargs):
    # varargin = IsDeceleration.varargin
    # nargin = IsDeceleration.nargin

    # the time which cost in the speed of now to stop
    t_dec=((VelNow - VelStart) / VelAcc)

    # according the time cost to cal the distance
    max_move_dis=np.dot(VelTick,VelNow)
    d1=(np.dot(t_dec,VelNow)) / 2
    d2=Error - d1
    t1=d2 / VelNow
    t2=VelTick - t1
    #     d1d2 = [d1 d2 d1+d2]
    
    
    if (d2 <= max_move_dis):
        t1t2=[t1,t2,t1 + t2]
        d1d2=[d1,d2,d1 + d2]
        tmp=VelNow
        VelNow = tmp - dot(VelAcc,t2)
        VelDis = np.dot(VelNow,VelTick) + np.dot(np.dot(0.5,abs(tmp - VelNow)),VelTick)
        isDec=1
    else:
        isDec=0
    
    return isDec,Vel
    

    
    ##FK
    
def FK(DOF,Angle,Link):
    # varargin = FK.varargin
    # nargin = FK.nargin


    Angle=np.dot(Angle,pi) / 180
    
    # (1) decide each joint starting point
    # (2) decide each joint's z axis
    # (3) define each joint's x,y axis
    # (4) indicate coordinate system a,s,n of tips 
    
    # ================ Step2 bulid DH Parameter Table ================  
    # 4 Parameter
    # ai : distance(zi-1, zi) along xi
    # alphai : angle(zi-1, zi) about xi
    # di : distance(oi-1,xi) along zi-1
    # thetai : angle(xi-1, xi) about zi
    
    #                   DH Parameter Table
    #+---+-----------+-----------+-----------+-----------+
    #| j |        a  |     alpha |        d  |     theta |
    #+---+-----------+-----------+-----------+-----------+
    #| 1 |          0|         90|         d1|         90|
    #| 2 |         a1|          0|          0|          0|
    #| 3 |         a2|          0|          0|          0|
    #+---+-----------+-----------+-----------+-----------+
    
    # DH Parameter       a  |     alpha |        d  |     theta |
    DHParameter=np.array([[0,     pi / 2,     Link[0],    pi / 2],[Link[1],0,0,0],[Link[2],0,0,0]])
    DH = pd.DataFrame(DHParameter)
    # starting setting
    JointPos=np.array([[0],[0],[0]])
    JointDir=np.array([[1,0,0],[0,1,0],[0,0,1]])
    T0_3=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    
    for i in np.arange(0,DOF):
        A=GenerateTransformationMatrices(Angle[i],DH.iloc[i])
        print(A)
        T0_3=np.dot(T0_3,A)                                   #??????????????????????????????????????????????????????????
        JointPos=np.array([JointPos,T0_3(np.arange[1:5:3])])
        JointDir=np.array([JointDir,T0_3(np.arange[1:4],np.arange[1:4])])
    
    
    Info.Pos = JointPos(np.arange[1:5:3])
    Info.JointPos = JointPos
    Info.JointDir = JointDir
    return Info

    ## DH homogeneous transfor matrix
    
def GenerateTransformationMatrices(Theta,DH_Parameter):
    # varargin = GenerateTransformationMatrices.varargin
    # nargin = GenerateTransformationMatrices.nargin

    #   
    #   cal T0_1~T5_6 = A1~A6 
    #   DH_Parameter (0) = a
    #   DH_Parameter (1) = alpha      
    #   DH_Parameter (2) = d      
    #   DH_Parameter (3) = theta
    
    C_Theta=math.cos(Theta + DH_Parameter[3])
    S_Theta=math.sin(Theta + DH_Parameter[3])
    C_Alpha=math.cos(DH_Parameter[1])
    S_Alpha=math.sin(DH_Parameter[1])
    A=np.array([[C_Theta,np.dot(np.dot(- 1,S_Theta),C_Alpha),np.dot(S_Theta,S_Alpha),np.dot(DH_Parameter[0],C_Theta),S_Theta,np.dot(C_Theta,C_Alpha),np.dot(np.dot(- 1,C_Theta),S_Alpha),np.dot(DH_Parameter[0],S_Theta),0,S_Alpha,C_Alpha,DH_Parameter[2]],[0,0,0,1]])
    return A
    

    
    ## IK
    
def IK(Goal,Link):
    # varargin = IK.varargin
    # nargin = IK.nargin
    Angle = np.array([0,0,0])
    #first joint theta1
    Angle[0]= -math.atan2(Goal[0],Goal[1])
    
    #third joint theta3
    cosTheta3=((Goal[2] - Link[0]) ** 2 + Goal[0] ** 2 + Goal[1] ** 2 - (Link[1]**2 + Link[2]**2)) / (np.dot(np.dot(2.0,Link[1]),Link[2]))
    c3=cosTheta3
    Angle[2]=math.atan2(np.dot(-1,math.sqrt(1 - c3**2)),c3)

    #second joint theta2
    r=math.sqrt(Goal[0] ** 2 + Goal[1] ** 2)
    Theta3=Angle[2]
    Angle[1]=math.atan2(Goal[2] - Link[0],r) - math.atan2(np.dot(Link[2],math.sin(Theta3)),Link[1] + np.dot(Link[2],math.cos(Theta3)))
    
    Angle=np.dot(Angle,180) / pi
    return Angle
    

    ##plot
    
def DrawRobotManipulator(DOF,JointPos,JointDir,FigNum,View):
    # varargin = DrawRobotManipulator.varargin
    # nargin = DrawRobotManipulator.nargin

    figure(FigNum)
    
    plt(JointPos(1,arange(1,end())),JointPos(2,arange(1,end())),JointPos(3,arange(1,end())),'linewidth',4)
    
    plt(JointPos(1,arange(1,end())),JointPos(2,arange(1,end())),JointPos(3,arange(1,end())),'ro','linewidth',7)
    
    xlabel('X axis')
    ylabel('Y axis')
    zlabel('Z axis')
    X=10
    Y=10
    Z=20
    
    BaseX=np.array([X,X,- X,- X,X])
    BaseY=np.array([Y,- Y,- Y,Y,Y])
    BaseZ=np.array([0,0,0,0,0])
    patch(BaseX,BaseY,BaseZ,'k')
    #-------------------------------------
    BaseX=np.array([X,X,X,X,X])
    BaseY=np.array([Y,- Y,- Y,Y,Y])
    BaseZ=np.array([0,0,- Z,- Z,0])
    patch(BaseX,BaseY,BaseZ,'k')
    #-------------------------------------
    BaseX=np.array([X,- X,- X,X,X])
    BaseY=np.array([Y,Y,Y,Y,Y])
    BaseZ=np.array([0,0,- Z,- Z,0])
    patch(BaseX,BaseY,BaseZ,'k')
    #-------------------------------------
    BaseX=np.array([- X,- X,- X,- X,- X])
    BaseY=np.array([- Y,Y,Y,- Y,- Y])
    BaseZ=np.array([0,0,- Z,- Z,0])
    patch(BaseX,BaseY,BaseZ,'k')
    #-------------------------------------
    BaseX=np.array([X,- X,- X,X,X])
    BaseY=np.array([- Y,- Y,- Y,- Y,- Y])
    BaseZ=np.array([0,0,- Z,- Z,0])
    patch(BaseX,BaseY,BaseZ,'k')
    for i in arange(1,DOF + 1):
        #-----------------------------------------
        nUnit_v=JointPos(arange(),i) + np.dot(5,JointDir(arange(),np.dot(3,(i - 1)) + 1))
        nBase=np.array([JointPos(arange(),i),nUnit_v])
        plt(nBase(1,arange(1,end())),nBase(2,arange(1,end())),nBase(3,arange(1,end())),'y','linewidth',2)
        sUnit_v=JointPos(arange(),i) + np.dot(5,JointDir(arange(),np.dot(3,(i - 1)) + 2))
        sBase=np.array([JointPos(arange(),i),sUnit_v])
        plt(sBase(1,arange(1,end())),sBase(2,arange(1,end())),sBase(3,arange(1,end())),'g','linewidth',2)
        aUnit_v=JointPos(arange(),i) + np.dot(5,JointDir(arange(),np.dot(3,(i - 1)) + 3))
        aBase=np.array([JointPos(arange(),i),aUnit_v])
        plt(aBase(1,arange(1,end())),aBase(2,arange(1,end())),aBase(3,arange(1,end())),'r','linewidth',2)
        plt.show()
    
    view(View)
    #axis equal
    axis(np.array([- 50,50,- 30,80,- 20,80]))
    return
    

    ##
    

def DrawAngle(t=None,Rec_Ang=None,*args,**kwargs):
    # varargin = DrawAngle.varargin
    # nargin = DrawAngle.nargin

    figure(3)
    subplot(3,1,1)
    plt(t,Rec_Ang(1,arange()))
    title('\theta1')
    xlabel('Sec')
    ylabel('degree')
    plt.grid
    subplot(3,1,2)
    plt(t,Rec_Ang(2,arange()))
    title('\theta2')
    xlabel('Sec')
    ylabel('degree')
    plt.grid
    subplot(3,1,3)
    plt(t,Rec_Ang(3,arange()))
    title('\theta3')
    xlabel('Sec')
    ylabel('degree')
    plt.grid
    return
    

    
    

def DrawRPM(t=None,Rec_RPM=None,*args,**kwargs):
    # varargin = DrawRPM.varargin
    # nargin = DrawRPM.nargin

    figure(4)
    subplot(3,1,1)
    plt(t,Rec_RPM(1,arange()),'-bo')
    title('\theta1')
    xlabel('Sec')
    ylabel('RPM')
    plt.grid
    subplot(3,1,2)
    plt(t,Rec_RPM(2,arange()),'-bo')
    title('\theta2')
    xlabel('Sec')
    ylabel('RPM')
    plt.grid
    subplot(3,1,3)
    plt(t,Rec_RPM(3,arange()),'-bo')
    title('\theta3')
    xlabel('Sec')
    ylabel('RPM')
    plt.grid
    return

    
    

def DrawACC(t=None,Rec_ACC=None,*args,**kwargs):
    # varargin = DrawACC.varargin
    # nargin = DrawACC.nargin

    figure(5)
    plt.subplot(3,1,1)
    plt(t,Rec_ACC(1,arange()),'-bo')
    title('\theta1')
    xlabel('Sec')
    ylabel('Acc')
    plt.grid
    plt.subplt(3,1,2)
    plt(t,Rec_ACC(2,arange()),'-bo')
    title('\theta2')
    xlabel('Sec')
    ylabel('Acc')
    plt.grid
    plt.subplot(3,1,3)
    plt(t,Rec_ACC(3,arange()),'-bo')
    title('\theta3')
    xlabel('Sec')
    ylabel('Acc')
    plt.grid
    return
    
# def main():
#     fig = plt.figure(figsize=(6,6))
#     #ax = Axes3D(fig)
#     ax = plt.subplot(211, aspect='equal')
#     ax.set_aspect('equal')

#     r2d = 1/180*pi

#     JointAngle = [0,0,0]
#     Od = [0,0,0]
#     pos = [ [None] * 3 for i in range(4) ]
#     ax = plt.subplot(111, projection='3d')
    
#     a_input=int(input('求各軸角度(輸入1)  末端點位(輸入2)  點到點動畫(輸入3):'))
#     if a_input==1:
# 	    print('請輸入末端點位置：')
# 	    Od = np.array([float(x) for x in input().split()])
# 	    print(Od)

# 	    kinematics(pos,Od,JointAngle)

# 	    x = kinematics(pos,Od,JointAngle)[:,0]
# 	    y = kinematics(pos,Od,JointAngle)[:,1]
# 	    z = kinematics(pos,Od,JointAngle)[:,2]
# 	    print(inver_Kinematics(JointAngle,Od)) #show pos
# 	    print(kinematics(pos,Od,JointAngle)) #show angle
# 	    ax.plot(x,y,z,"o-",color="#00aa00", ms=4, mew=0.5,label='aaa') 

# 	    ax.set_zlabel('Z')
# 	    ax.set_ylabel('Y')
# 	    ax.set_xlabel('X')
# 	    plt.title('this is title') 
# 	    plt.legend() 

# 	    plt.show()
#     elif a_input==2:
# 	    print('請輸入各軸角度(三軸)：')
# 	    JointAngle = np.array([float(x) for x in input().split()])
    
# 	    direct_Kinematics(JointAngle)
# 	    print(direct_Kinematics(JointAngle))
	
# 	    x = direct_Kinematics(JointAngle)[:,0]
# 	    y = direct_Kinematics(JointAngle)[:,1]
# 	    z = direct_Kinematics(JointAngle)[:,2]
# 	    ax.plot(x,y,z,"o-",color="#00aa00", ms=4, mew=0.5,label='aaa') # 'b*'表示藍色*狀線，label是指定義圖例

# 	    ax.set_zlabel('Z')  # 座標軸
# 	    ax.set_ylabel('Y')
# 	    ax.set_xlabel('X')
# 	    plt.title('this is title') # 設置標題
# 	    plt.legend() # 顯示上面定義的圖例

# 	    plt.show()
#        # 繪製動畫 輸入兩點
#     elif a_input==3:
# 	    print('請輸入起始末端點位置：')
# 	    Od = np.array([float(x) for x in input().split()])
# 	    print(Od)
# 	    print('請輸入終點末端點位置：')
# 	    O_end = np.array([float(x) for x in input().split()])
# 	    num = 100
# 	    Cnt = (O_end - Od)/num

# 	    ani = animation.FuncAnimation(fig,
#                               func = animate,
#                               frames=100,
#                               init_func=init,
#                               interval=10,
#                               repeat=False)
	
# 	    ax.set_zlabel('Z')  # 座標軸
# 	    ax.set_ylabel('Y')
# 	    ax.set_xlabel('X')
# 	    plt.title('this is title') # 設置標題
# 	    plt.legend() # 顯示上面定義的圖例

# 	# ani.save('basic_animation.mp4', fps=30, extra_args=['-vcodec', 'libx264'])
# 	    plt.show()
#     else:
#         print('See you next time')

if __name__ == '__main__':
     Line_Traj_3_DOF()