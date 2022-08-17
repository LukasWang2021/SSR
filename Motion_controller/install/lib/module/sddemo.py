import motion
import innertypes
import time

P = []

#p0
pos = innertypes.POSTURE(motion.COORD_CARTE,
    1,1,1,0,
    0,0,0,0,0,0,0,0,0,
    275,500,250,0,0,-3.141592654,0,0,0)

P.append(pos)

#p1
pos = innertypes.POSTURE(motion.COORD_CARTE,
    1,1,1,0,
    0,0,0,0,0,0,0,0,0,
    500,0,250,0,0,-3.141592654,0,0,0)

P.append(pos)

#p2
pos = innertypes.POSTURE(motion.COORD_CARTE,
    1,1,1,0,
    0,0,0,0,0,0,0,0,0,
    275,374,430,2.443460953,-1.570796327,0.785398163,0,0,0)

P.append(pos)

#p3
pos = innertypes.POSTURE(motion.COORD_CARTE,
    1,1,1,0,
    0,0,0,0,0,0,0,0,0,
	477,274,600,2.443460953,-1.570796327,0.785398163,0,0,0)

P.append(pos)

#p4
pos = innertypes.POSTURE(motion.COORD_CARTE,
    1,1,1,0,
    0,0,0,0,0,0,0,0,0,
    275,-500,250,0,0,-3.141592654,0,0,0)

P.append(pos)

#p5
pos = innertypes.POSTURE(motion.COORD_CARTE,
    1,1,1,0,
    0,0,0,0,0,0,0,0,0,
    200,0,250,0,0,0,0,0,0)

P.append(pos)

#p6
pos = innertypes.POSTURE(motion.COORD_CARTE,
    1,1,1,0,
    0,0,0,0,0,0,0,0,0,
    -23.642,39.61,-0.5,-0.535,0.013,0.001,0,0,0)

P.append(pos)

#p7
pos = innertypes.POSTURE(motion.COORD_CARTE,
    1,1,1,0,
    0,0,0,0,0,0,0,0,0,
    400,100,250,0,0,-3.14159,0,0,0)

P.append(pos)

#p8
pos = innertypes.POSTURE(motion.COORD_CARTE,
    1,1,1,0,
    0,0,0,0,0,0,0,0,0,
    500,0,250,0,0,-3.14159,0,0,0)

P.append(pos)

#p9
pos = innertypes.POSTURE(motion.COORD_CARTE,
    1,1,1,0,
    0,0,0,0,0,0,0,0,0,
    400,-100,250,0,0,-3.14159,0,0,0)

P.append(pos)

#p10
pos = innertypes.POSTURE(motion.COORD_CARTE,
    1,1,1,0,
    0,0,0,0,0,0,0,0,0,
    300,0,250,0,0,-3.14159,0,0,0)


P.append(pos)

def demoL():
    motion.MoveJ(P[0], 0.5, motion.SD, 25.0, 1.0)
    motion.MoveL(P[1], 500.0, motion.SD, 25.0, 1.0)
    motion.MoveL(P[4], 500.0, motion.SD, 25.0, 1.0)
    pass

def demoC():
    motion.MoveJ(P[7], 0.5, motion.FINE, -1.0, 1.0)
    motion.MoveC(P[8], P[9], 500.0, motion.SD, 30.0, 1.0)
    motion.MoveC(P[10], P[7], 500.0, motion.SD, 30.0, 1.0)
    pass

def run():
    demoL()
    demoC()
    pass