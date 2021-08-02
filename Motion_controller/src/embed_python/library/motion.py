import group
from overload import *
import ctypes
from innertypes import * 

"""
# via,trget are PR or P or [x,y,z,a,b,c] or [j1,j2,j3,j4,j5,j6]
# Offset C_VEC (x,y,z,a,b,c)
# Offset J_VEC (J1,J2,J3,J4,J5,J6)
# Offset PR[x]
# movej velocity unit is percent
"""

"""
smooth type
"""
FINE = 0
SD = 1
SV = 2

"""
frame type
"""
TF = 0
UF = 1

"""
coordiante type
"""
COOR_JOINT = 0
COOR_CARTE = 1

##############################################
# Move joint functions
##############################################

@overload(POSTURE,float,int,float)
def MoveJ(target, vel, smooth_type, smooth_val):
    group.MoveJ(target, vel, smooth_type, smooth_val)
    pass

@overload(POSTURE,float,int,float,float)
def MoveJ(target, vel, smooth_type, smooth_val, acc):
    group.MoveJwithAcc(target, vel, smooth_type, smooth_val, acc)
    pass

@overload(POSTURE,float,int,float,int,POSTURE)
def MoveJ(target, vel, smooth_type, smooth_val, offset_type, offset_val):
    group.MoveJwithOffset(target, vel, smooth_type, smooth_val, offset_type, offset_val)
    pass

@overload(POSTURE,float,int,float,float,int,POSTURE)
def MoveJ(target, vel, smooth_type, smooth_val, acc, offset_type, offset_val):
    group.MoveJwithAccOffset(target, vel, smooth_type, smooth_val, acc, offset_type, offset_val)
    pass

##############################################
# Move liner functions
##############################################

@overload(POSTURE,float,int,float)
def MoveL(target, vel, smooth_type, smooth_val):
    group.MoveL(target, vel, smooth_type, smooth_val)
    pass

@overload(POSTURE,float,int,float,float)
def MoveL(target, vel, smooth_type, smooth_val, acc):
    pass

@overload(POSTURE,float,int,float,int,POSTURE)
def MoveL(target, vel, smooth_type, smooth_val, offset_type, offset_val):
    pass

@overload(POSTURE,float,int,float,float,int,POSTURE)
def MoveL(target, vel, smooth_type, smooth_val, acc, offset_type, offset_val):
    pass


##############################################
# Move circular functions
##############################################

@overload(POSTURE,POSTURE,float,int,float)
def MoveC(via, target, vel, smooth_type, smooth_val):
    group.MoveC(via, target, vel, smooth_type, smooth_val)
    pass

@overload(POSTURE,POSTURE,float,int,float,float)
def MoveC(via, target, vel, smooth_type, smooth_val, acc):
    pass

@overload(POSTURE,POSTURE,float,int,float,int,POSTURE)
def MoveC(via, target, vel, smooth_type, smooth_val, offset_type, offset_val):
    pass

@overload(POSTURE,POSTURE,float,int,float,float,int,POSTURE)
def MoveC(via, target, vel, smooth_type, smooth_val, acc, offset_type, offset_val):
    pass