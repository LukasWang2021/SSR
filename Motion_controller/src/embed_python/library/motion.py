import group
from overload import *
import ctypes
from rtmtypes import * 

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
COORD_JOINT = 0
COORD_CARTE = 1

##############################################
# Move joint functions
##############################################

@overload(Posture,float,int,float)
def MoveJ(target, vel, smooth_type, smooth_val):
    group.MoveJ(target, vel, smooth_type, smooth_val)
    pass

@overload(Posture,float,int,float,float)
def MoveJ(target, vel, smooth_type, smooth_val, acc):
    group.MoveJwithAcc(target, vel, smooth_type, smooth_val, acc)
    pass

@overload(Posture,float,int,float,int,int)
def MoveJ(target, vel, smooth_type, smooth_val, offset_type, offset_val):
    group.MoveJwithOffset(target, vel, smooth_type, smooth_val, offset_type, offset_val)
    pass

@overload(Posture,float,int,float,float,int,int)
def MoveJ(target, vel, smooth_type, smooth_val, acc, offset_type, offset_val):
    group.MoveJwithAccOffset(target, vel, smooth_type, smooth_val, acc, offset_type, offset_val)
    pass

@overload(dict,float,int,float)
def MoveJ(target, vel, smooth_type, smooth_val):
    p = Posture()
    p.coord = target['coord']
    p.arm   = target['posture'][0]
    p.elbow = target['posture'][1]
    p.wrist = target['posture'][2]
    p.flip  = target['posture'][3]
    p.turn1 = target['turn'][0]
    p.turn2 = target['turn'][1]
    p.turn3 = target['turn'][2]
    p.turn4 = target['turn'][3]
    p.turn5 = target['turn'][4]
    p.turn6 = target['turn'][5]
    p.turn7 = target['turn'][6]
    p.turn8 = target['turn'][7]
    p.turn9 = target['turn'][8]
    p.pos1 = target['position'][0]
    p.pos2 = target['position'][1]
    p.pos3 = target['position'][2]
    p.pos4 = target['position'][3]
    p.pos5 = target['position'][4]
    p.pos6 = target['position'][5]
    p.pos7 = target['position'][6]
    p.pos8 = target['position'][7]
    p.pos9 = target['position'][8]
    group.MoveJ(target, vel, smooth_type, smooth_val)
    pass


##############################################
# Move liner functions
##############################################

@overload(Posture,float,int,float)
def MoveL(target, vel, smooth_type, smooth_val):
    group.MoveL(target, vel, smooth_type, smooth_val)
    pass

@overload(Posture,int,int,float)
def MoveL(target, vel, smooth_type, smooth_val):
    group.MoveL(target, float(vel), smooth_type, smooth_val)
    pass

@overload(Posture,float,int,int)
def MoveL(target, vel, smooth_type, smooth_val):
    group.MoveL(target, vel, smooth_type, float(smooth_val))
    pass

@overload(Posture,int,int,int)
def MoveL(target, vel, smooth_type, smooth_val):
    group.MoveL(target, float(vel), smooth_type, float(smooth_val))
    pass

@overload(Posture,float,int,float,float)
def MoveL(target, vel, smooth_type, smooth_val, acc):
    group.MoveLwithAcc(target, vel, smooth_type, smooth_val, acc)
    pass

@overload(Posture,float,int,float,int,int)
def MoveL(target, vel, smooth_type, smooth_val, offset_type, offset_val):
    group.MoveLwithOffset(target, vel, smooth_type, smooth_val, offset_type, offset_val)
    pass

@overload(Posture,float,int,float,float,int,int)
def MoveL(target, vel, smooth_type, smooth_val, acc, offset_type, offset_val):
    group.MoveLwithAccOffset(target, vel, smooth_type, smooth_val, acc, offset_type, offset_val)
    pass


##############################################
# Move circular functions
##############################################

@overload(Posture,Posture,float,int,float)
def MoveC(via, target, vel, smooth_type, smooth_val):
    group.MoveC(via, target, vel, smooth_type, smooth_val)
    pass

@overload(Posture,Posture,float,int,float,float)
def MoveC(via, target, vel, smooth_type, smooth_val, acc):
    group.MoveCwithAcc(via, target, vel, smooth_type, smooth_val, acc)
    pass

@overload(Posture,Posture,float,int,float,int,int)
def MoveC(via, target, vel, smooth_type, smooth_val, offset_type, offset_val):
    group.MoveCwithOffset(via, target, vel, smooth_type, smooth_val, offset_type, offset_val)
    pass

@overload(Posture,Posture,float,int,float,float,int,int)
def MoveC(via, target, vel, smooth_type, smooth_val, acc, offset_type, offset_val):
    group.MoveCwithAccOffset(via, target, vel, smooth_type, smooth_val, acc, offset_type, offset_val)
    pass