# -*- coding: utf-8 -*-
"""
Created on Mon Aug 15 17:32:51 2022

@author: wuym
"""

import motion
import rtmtypes
import controller
import device
import vector
import matrix
import sysmodel

pi = 3.1415926

def calibration():
    point_x_pos = rtmtypes.Posture(motion.COORD_JOINT,
            1, 1, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0)

    point_x_neg = rtmtypes.Posture(motion.COORD_JOINT,
            1, 1, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, pi, 0, 0, 0)


    point_y_pos = rtmtypes.Posture(motion.COORD_JOINT,
            1, 1, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, -pi / 2, 0, 0, 0)

    point_y_neg = rtmtypes.Posture(motion.COORD_JOINT,
            1, 1, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, pi / 2, 0, 0, 0)

    point_z_neg = rtmtypes.Posture(motion.COORD_JOINT,
            1, 1, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, -pi / 2, pi, 0, 0, 0)

    point_z_pos = rtmtypes.Posture(motion.COORD_JOINT,
            1, 1, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, pi / 2, pi, 0, 0, 0)


    motion.MoveJ(point_y_pos, 0.2, motion.FINE, -1.0, 0.5)
    controller.Delay(2.0)
    sn_val_y_pos = device.ForceRawValue(0)
    print("###########sn_val_y_pos", sn_val_y_pos)

    motion.MoveJ(point_x_pos, 0.2, motion.FINE, -1.0, 0.5)
    controller.Delay(2.0)
    sn_val_x_pos = device.ForceRawValue(0)
    print("###########sn_val_x_pos", sn_val_x_pos)

    motion.MoveJ(point_y_neg, 0.2, motion.FINE, -1.0, 0.5)
    controller.Delay(2.0)
    sn_val_y_neg = device.ForceRawValue(0)
    print("###########sn_val_y_neg", sn_val_y_neg)

    motion.MoveJ(point_x_neg, 0.2, motion.FINE, -1.0, 0.5)
    controller.Delay(2.0)
    sn_val_x_neg = device.ForceRawValue(0)
    print("###########sn_val_x_neg", sn_val_x_neg)

    motion.MoveJ(point_z_pos, 0.2, motion.FINE, -1.0, 0.5)
    controller.Delay(2.0)
    sn_val_z_pos = device.ForceRawValue(0)
    print("###########sn_val_z_pos", sn_val_z_pos)

    motion.MoveJ(point_z_neg, 0.2, motion.FINE, -1.0, 0.5)
    controller.Delay(2.0)
    sn_val_z_neg = device.ForceRawValue(0)
    print("###########sn_val_z_neg", sn_val_z_neg)

    motion.MoveJ(point_x_pos, 0.2, motion.FINE, -1.0, 0.5)

    # calibration of F_offset and F_gravity 
    #x = (AT*A)-1 * AT * b
    fa = matrix.Matrix([[1.0, 0.0, 0.0, 1.0], 
                        [0.0, 1.0, 0.0, 0.0], 
                        [0.0, 0.0, 1.0, 0.0], 
                        [1.0, 0.0, 0.0,-1.0], 
                        [0.0, 1.0, 0.0, 0.0], 
                        [0.0, 0.0, 1.0, 0.0], 
                        [1.0, 0.0, 0.0, 0.0], 
                        [0.0, 1.0, 0.0, 1.0], 
                        [0.0, 0.0, 1.0, 0.0], 
                        [1.0, 0.0, 0.0, 0.0], 
                        [0.0, 1.0, 0.0,-1.0], 
                        [0.0, 0.0, 1.0, 0.0], 
                        [1.0, 0.0, 0.0, 0.0], 
                        [0.0, 1.0, 0.0, 0.0], 
                        [0.0, 0.0, 1.0, 1.0], 
                        [1.0, 0.0, 0.0, 0.0], 
                        [0.0, 1.0, 0.0, 0.0], 
                        [0.0, 0.0, 1.0,-1.0]])

    fb = matrix.Matrix([[sn_val_x_pos[0]], [sn_val_x_pos[1]], [sn_val_x_pos[2]], 
                        [sn_val_x_neg[0]], [sn_val_x_neg[1]], [sn_val_x_neg[2]], 
                        [sn_val_y_pos[0]], [sn_val_y_pos[1]], [sn_val_y_pos[2]], 
                        [sn_val_y_neg[0]], [sn_val_y_neg[1]], [sn_val_y_neg[2]], 
                        [sn_val_z_pos[0]], [sn_val_z_pos[1]], [sn_val_z_pos[2]], 
                        [sn_val_z_neg[0]], [sn_val_z_neg[1]], [sn_val_z_neg[2]]]) # force of xyz

    Tfa = fa.transpose()

    x = (Tfa * fa).inv() * Tfa * fb
    print("###########F", x.value, Tfa.value, fb.value)

    #calibration of the center of mass and torque offset 
    #x = (AT*A)-1 * AT * b

    F_offset  = vector.Vector(x.value[0:3])
    F_gravity = x.value[-1]
    print("###########F_offset", F_offset.value, F_gravity)
    #print("###########F_offset", F_offset.value)

    tb = matrix.Matrix([[sn_val_x_pos[3]], [sn_val_x_pos[4]], [sn_val_x_pos[5]],
                        [sn_val_x_neg[3]], [sn_val_x_neg[4]], [sn_val_x_neg[5]],
                        [sn_val_y_pos[3]], [sn_val_y_pos[4]], [sn_val_y_pos[5]],
                        [sn_val_y_neg[3]], [sn_val_y_neg[4]], [sn_val_y_neg[5]],
                        [sn_val_z_pos[3]], [sn_val_z_pos[4]], [sn_val_z_pos[5]],
                        [sn_val_z_neg[3]], [sn_val_z_neg[4]], [sn_val_z_neg[5]]]) # torque of xyz

    tmp = matrix.Matrix(F_offset.value).transpose()
    F_offset = vector.Vector(tmp.value[0])
    #print("###########F_offset", F_offset.value)

    fx_pos = F_offset - vector.Vector(sn_val_x_pos[0:3])
    fx_neg = F_offset - vector.Vector(sn_val_x_neg[0:3])
    fy_pos = F_offset - vector.Vector(sn_val_y_pos[0:3])
    fy_neg = F_offset - vector.Vector(sn_val_y_neg[0:3])
    fz_pos = F_offset - vector.Vector(sn_val_z_pos[0:3])
    fz_neg = F_offset - vector.Vector(sn_val_z_neg[0:3])

    ta = []
    for v in fx_pos.skew_append_eye():
        ta.append(v)
        pass
    for v in fx_neg.skew_append_eye():
        ta.append(v)
        pass
    for v in fy_pos.skew_append_eye():
        ta.append(v)
        pass
    for v in fy_neg.skew_append_eye():
        ta.append(v)
        pass
    for v in fz_pos.skew_append_eye():
        ta.append(v)
        pass
    for v in fz_neg.skew_append_eye():
        ta.append(v)
        pass

    ta = matrix.Matrix(ta)
    #print("@@@@@", ta.value)
    Tta = ta.transpose()
    #print("@@@@@", Tta.value)
    x = (Tta * ta).inv() * Tta *tb
    print("###########T", x.value)

    T_offset = x.value[0:3]
    M_center = x.value[3:6]
    print("###########T_offset", T_offset, M_center)

    unit = 16777216
    # 173 F_offset*3 F_gravity*1 T_offset*3 M_center*3
    sysmodel.SetParam(1, 127, int(F_offset.value[0] * unit))
    sysmodel.SetParam(1, 128, int(F_offset.value[1] * unit))
    sysmodel.SetParam(1, 129, int(F_offset.value[2] * unit))

    sysmodel.SetParam(1, 130, int(F_gravity[0] * unit))

    sysmodel.SetParam(1, 131, int(T_offset[0][0] * unit))
    sysmodel.SetParam(1, 132, int(T_offset[1][0] * unit))
    sysmodel.SetParam(1, 133, int(T_offset[2][0] * unit))

    sysmodel.SetParam(1, 134, int(M_center[0][0] * unit))
    sysmodel.SetParam(1, 135, int(M_center[1][0] * unit))
    sysmodel.SetParam(1, 136, int(M_center[2][0] * unit))

    sysmodel.SaveParam(1)

    device.ForceReloadParam(0)









