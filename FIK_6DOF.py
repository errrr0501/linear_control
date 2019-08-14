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
import pdb



def FIK_6DOF(X,Y,Z,Roll,Pitch,Yaw):
    # varargin = FIK_6DOF.varargin
    # nargin = FIK_6DOF.nargin

    DOF=6

    DH_Parameter=np.array([[0, pi/2, 10, pi/2],
                           [60,   0,   0,  0],
                           [0, pi/2,   0, pi/2],
                           [0, -pi/2, 50, 0],
                           [0, pi/2,  0, 0],
                           [0,    0, 20, 0]])
    Pos=np.array([[X],[Y],[Z]])
    Euler_RPY=np.array([Roll,Pitch,Yaw])
    JointAngle=InverseKinematics(Pos,Euler_RPY,DOF,DH_Parameter)
    #JointAngle*180/pi
    print(JointAngle)
    print(JointAngle*180/pi)
    Info=ForwardKinematics(DOF,JointAngle,DH_Parameter)
    # DrawRobotManipulator(DOF,Info.JointPos,Info.JointDir)
    # figure(1)
    # str_x=num2str(roundn(Info.P[0],- 2))
    # str_y=num2str(roundn(Info.P[1],- 2))
    # str_z=num2str(roundn(Info.P[2],- 2))
    # str_Pitch=num2str(roundn(np.dot(Info.Pitch,180) / pi,- 2))
    # str_Roll=num2str(roundn(np.dot(Info.Roll,180) / pi,- 2))
    # str_Yaw=num2str(roundn(np.dot(Info.Yaw,180) / pi,- 2))
    # str=np.array(['X Y Z = ',str_x,'   ',str_y,'   ',str_z,'     ','R P Y = ',str_Roll,'   ',str_Pitch,'   ',str_Yaw])
    # title(str)
       
	# direct_Kinematics(JointAngle)
	#print(direct_Kinematics(JointAngle))
	

    fig = plt.figure(figsize=(8,8))
    
    ax = plt.subplot(211, aspect='equal')
    ax.set_aspect('equal')
    
    r2d = 1/180*pi

    Od = [0,0,0]
    pos = [ [None] * 3 for i in range(4) ]
    ax = plt.subplot(111, projection='3d')
    
    x = Info.JointPos[:,0]
    y = Info.JointPos[:,1]
    z = Info.JointPos[:,2]
   
    ax.plot(x,y,z,"o-",color="#00aa00", ms=4, mew=0.5,label='Arm')

    ax.set_zlabel('Z')  # 座標軸
    ax.set_ylabel('Y')
    ax.set_xlabel('X')
    plt.title('6DOF_arm') # 設置標題
    plt.legend() # 顯示上面定義的圖例
    plt.show()
    return
    
    
# ======================================================================== 
# =========================================================================

class Cmd():
    def __init__(self):
        self.Roll = 0.1
        self.Pitch = 0.1
        self.Yaw  = 0.1
        self.P = 0.1
        self.R = 0.1
        self.Elbow = 0.1
        self.Wrist = 0.1
        self.L1 = 0.1
        self.L2 = 0.1
        self.L3 = 0.1
        self.L4 = 0.1
        self.Angle =0.1 


def InverseKinematics(Pos,Euler_RPY,DOF,DH_Parameter):
    # varargin = InverseKinematics.varargin
    # nargin = InverseKinematics.nargin

    if abs(Euler_RPY[1] > 90):
        print('*** note: pitch>90 degree, feed back one pi lost *** \n')

    DesiredCmd = Cmd()

    DesiredCmd.Roll = Euler_RPY[0]*pi / 180
    DesiredCmd.Pitch = Euler_RPY[1]*pi / 180
    DesiredCmd.Yaw = Euler_RPY[2]*pi / 180
    
    
    A=DesiredCmd.Yaw
    B=DesiredCmd.Pitch
    C=DesiredCmd.Roll
    RX=np.array([[1,0,0],[0,cos(A), -math.sin(A)],[0,math.sin(A),math.cos(A)]])
    
    RY=np.array([[math.cos(B),0,math.sin(B)],[0,1,0],[-math.sin(B),0,math.cos(B)]])
    
    RZ=np.array([[math.cos(C),-math.sin(C),0],[math.sin(C),cos(C),0],[0,0,1]])
    
    OrienMat=np.dot(np.dot(RZ,RY),RX)
    
    DesiredCmd.P = Pos
    
    DesiredCmd.R = OrienMat
    
    DesiredCmd.Elbow = -1
    
    DesiredCmd.Wrist = -1
    
    DesiredCmd.L1 = DH_Parameter[0,2]
    DesiredCmd.L2 = DH_Parameter[1,0]
    DesiredCmd.L3 = DH_Parameter[3,2]
    DesiredCmd.L4 = DH_Parameter[5,2]

    
    DesiredCmd.Angle = np.zeros((1,DOF))
    
    DesiredCmd = InversePosition(DesiredCmd)     
    DesiredCmd = InverseOrientation(DesiredCmd,DH_Parameter)
    JointAngle = DesiredCmd.Angle
    return JointAngle
    
    

def InversePosition(DesiredCmd):
    # varargin = InversePosition.varargin
    # nargin = InversePosition.nargin
    # print(DesiredCmd.R)
    # print(DesiredCmd.R[:,2])
    g = np.array([DesiredCmd.R[:,2]])
    WristPos=DesiredCmd.P - np.array([np.dot(DesiredCmd.L4,g.reshape(-1,1))])
    
    if (abs(WristPos[0,0]) < 0.0001 and abs(WristPos[0,1]) < 0.0001):
        DesiredCmd.Angle[0,0]=0
    else:
        DesiredCmd.Angle[0,0]=math.atan2(-WristPos[0,0],WristPos[0,1])
    
    xc_2=WristPos[0,0] ** 2
    yc_2=WristPos[0,1] ** 2
    s=WristPos[0,2] - DesiredCmd.L1
    D=(xc_2 + yc_2 + s**2 - DesiredCmd.L2**2 - DesiredCmd.L3**2) / (2.0*DesiredCmd.L2*DesiredCmd.L3)
    DesiredCmd.Angle[0,2]=math.atan2(DesiredCmd.Elbow*math.sqrt(1 - D**2),D)
    DesiredCmd.Angle[0,1]=math.atan2(s,math.sqrt(xc_2 + yc_2)) - math.atan2(DesiredCmd.L3*math.sin(DesiredCmd.Angle[0,2]),DesiredCmd.L2 + DesiredCmd.L3*math.cos(DesiredCmd.Angle[0,2]))
    return DesiredCmd
    
    
    

def InverseOrientation(DesiredCmd,DH_Parameter):
    # varargin = InverseOrientation.varargin
    # nargin = InverseOrientation.nargin

    s1=math.sin(DesiredCmd.Angle[0,0] + DH_Parameter[0,3])
    c1=math.cos(DesiredCmd.Angle[0,0] + DH_Parameter[0,3])
    s23=math.sin(DesiredCmd.Angle[0,1] + DH_Parameter[1,3] + DesiredCmd.Angle[0,2] + DH_Parameter[2,3])
    c23=math.cos(DesiredCmd.Angle[0,1] + DH_Parameter[1,3] + DesiredCmd.Angle[0,2] + DH_Parameter[2,3])
    R0_3=np.array([[c1*c23,s1,c1*s23],[s1*c23,-c1,s1*s23],[s23,0,-c23]])
    
    R3_6=np.dot(R0_3.T,DesiredCmd.R)
    
    
    DesiredCmd.Angle[0,4]=math.atan2(DesiredCmd.Wrist*math.sqrt(1-R3_6[2,2]**2),R3_6[2,2])
    if (abs(R3_6[2,2]) > 0.9999):
        DesiredCmd.Angle[0,3]=0
        DesiredCmd.Angle[0,5]=math.atan2(R3_6[1,0],R3_6[0,0])
    else:
        if (DesiredCmd.Wrist > 0):
            DesiredCmd.Angle[0,3]=math.atan2(R3_6[1,2],R3_6[0,2])
            DesiredCmd.Angle[0,5]=math.atan2(R3_6
            [2,1],- R3_6[2,0])
        else:
            DesiredCmd.Angle[0,3]=math.atan2(- R3_6[1,2],-R3_6[0,2])
            DesiredCmd.Angle[0,5]=math.atan2(- R3_6[2,1],R3_6[2,0])
    
    return DesiredCmd



class Information():
    def __init__(self):
        self.JointPos = 0.1
        self.JointDir = 0.1
        self.T0_6 = 0.1
        self.Yaw = 0.1
        self.Pitch = 0.1
        self.Roll = 0.1
     
def ForwardKinematics(DOF,JointAngle,DH_Parameter):
    # varargin = ForwardKinematics.varargin
    # nargin = ForwardKinematics.nargin
#step 1
    Info = Information()
    Info.JointPos = np.array([[0],[0],[0]])
    
    Info.JointDir = np.array([[1,0,0],[0,1,0],[0,0,1]])
    
    Info.T0_6 = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    
    for i in range(0,DOF):
        A=GenerateTransformationMatrices(JointAngle[0,i],DH_Parameter[i,])
        
        Info.T0_6 = np.dot(Info.T0_6,A)
        g = np.delete(Info.T0_6,[3],axis = 0)
        k = g[:,3].reshape(3,1)
        #t = Info.JointPos
        Info.JointPos = np.concatenate((Info.JointPos,k),axis = 1)
        f = np.delete(Info.T0_6,[3],axis = 1)
        p = np.array([f[0,:],f[1,:],f[2,:]])
        # z = np.array([Info.JointDir[:,0],Info.JointDir[:,1],Info.JointDir[:,2]])
        # z = z.T
        
        Info.JointDir = np.concatenate((Info.JointDir,p),axis = 1)
        
    
    ## Step 3
    g = np.delete(Info.T0_6,[3],axis = 0)
    k = g[:,3].reshape(3,1)
    f = np.delete(Info.T0_6,[3],axis = 1)
    p = np.array([f[0,:],f[1,:],f[2,:]])
    Info.P = k
    Info.R = p
    R0_6 = p
    

    cal_err=1*10 **-8
    if (abs(R0_6[2,0] - 1) < cal_err):
        Info.Yaw = math.atan2(-R0_6[0,1], -R0_6[0,2])
        Info.Pitch = -pi/2
        Info.Roll = 0
        print('*** Gimbal Lock at Pitch = -90 degree. ***\n')
        print('*** Yaw change to Yaw+Roll degree. ***\n')
        print('*** Roll change to zero degree. ***\n')
    else:
        if (abs(R0_6[2,0] + 1) < cal_err):
            Info.Yaw = math.atan2(R0_6[0,1],R0_6[0,2])
            Info.Pitch = pi/2
            Info.Roll = 0
            fprintf('*** Gimbal Lock at Pitch = +90 degree. ***\n')
            fprintf('*** Yaw change to Yaw-- degree. ***\n')
            fprintf('*** Roll change to zero degree. ***\n')
        else:
            Info.Yaw = math.atan2(R0_6[2,1],R0_6[2,2])
            Info.Pitch = math.asin(- R0_6[2,0])
            Info.Roll = math.atan2(R0_6[1,0],R0_6[0,0])

    
    Info.Roll = normalize(Info.Roll)
    Info.Pitch = normalize(Info.Pitch)
    Info.Yaw = normalize(Info.Yaw)
    #pdb.set_trace()
    return Info
    

    

def normalize(ang):
    # varargin = normalize.varargin
    # nargin = normalize.nargin

    while (ang > pi):

        ang = ang - 2*pi


    
    while (ang < - pi):

        ang = ang + 2*pi

    
    return ang
    

    
    

def GenerateTransformationMatrices(Theta,DH_Parameter):
    # varargin = GenerateTransformationMatrices.varargin
    # nargin = GenerateTransformationMatrices.nargin

    
    C_Theta=math.cos(Theta + DH_Parameter[3])
    S_Theta=math.sin(Theta + DH_Parameter[3])
    C_Alpha=math.cos(DH_Parameter[1])
    S_Alpha=math.sin(DH_Parameter[1])
    A=np.array([[C_Theta,-1*S_Theta*C_Alpha,S_Theta*S_Alpha,DH_Parameter[0]*C_Theta],[S_Theta,C_Theta*C_Alpha,- 1*C_Theta*S_Alpha,DH_Parameter[0]*S_Theta],[0,S_Alpha,C_Alpha,DH_Parameter[2]],[0,0,0,1]])
    return A
    
    

def DrawRobotManipulator(DOF,JointPos,JointDir):
    # varargin = DrawRobotManipulator.varargin
    # nargin = DrawRobotManipulator.nargin

    # figure(1)
    
    # hold off
    plt.plot(JointPos[0,:],JointPos[1,:],JointPos[2,:],'linewidth',4)
    plt.show()
    plt.plot(JointPos[0,:],JointPos[1,:],JointPos[2,:],'ro','linewidth',7)
    plt.grid()
    xlabel('X axis')
    ylabel('Y axis')
    zlabel('Z axis')
    X=10
    Y=10
    Z=20
    pdb.set_trace()
    
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
    for i in arange(1,DOF + 1).reshape(-1):
        if (i == DOF + 1):
            #-----------------------------------------
            nUnit_v=JointPos(np.arange(),i) + np.dot(10,JointDir(np.arange(),np.dot(3,(i - 1)) + 1))
            nBase=np.arrray([JointPos(np.arange(),i),nUnit_v])
            plt(nBase(1,arange(1,end())),nBase(2,np.arange(1,end())),nBase(3,np.arange(1,end())),'r','linewidth',2)
            sUnit_v=JointPos(np.arange(),i) + np.dot(10,JointDir(np.arange(),np.dot(3,(i - 1)) + 2))
            sBase=np.array([JointPos(np.arange(),i),sUnit_v])
            plt(sBase(1,np.arange(1,end())),sBase(2,np.arange(1,end())),sBase(3,np.arange(1,end())),'g','linewidth',2)
            aUnit_v=JointPos(np.arange(),i) + np.dot(10,JointDir(np.arange(),np.dot(3,(i - 1)) + 3))
            aBase=np.array([JointPos(np.arange(),i),aUnit_v])
            plt(aBase(1,np.arange(1,end())),aBase(2,np.arange(1,end())),aBase(3,np.arange(1,end())),'c','linewidth',2)
            plt.show()
    
    axis('equal')
    return



if __name__ == '__main__':
    X = 10
    Y = 50
    Z = 20
    rpy = np.array([20,-10,-90]*1)
    rpy = np.array([10,89,-90]*1)
    Roll = rpy[0]
    Pitch = rpy[1]
    Yaw  = rpy[2]     

    FIK_6DOF(X,Y,Z,Roll,Pitch,Yaw)

	
 