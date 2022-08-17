import motion
import innertypes
import time

P = []

#p0
pos = innertypes.POSTURE(motion.COORD_JOINT,
    1,1,1,0,
    0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0)

P.append(pos)

#p1
pos = innertypes.POSTURE(motion.COORD_JOINT,
    1,1,1,0,
    0,0,0,0,0,0,0,0,0,
    2.617993878,1.745329252,-1.047197551,3.141592654,-1.570796327,6.283185307,0,0,0)

P.append(pos)

#p2
pos = innertypes.POSTURE(motion.COORD_JOINT,
    1,1,1,0,
    0,0,0,0,0,0,0,0,0,
    -0.674744289,-0.290614774,-0.799954209,0,-0.480227344,0.896052038,0,0,-1.000003848)

P.append(pos)

#p3
pos = innertypes.POSTURE(motion.COORD_CARTE,
    1,1,1,0,
    0,0,0,0,0,0,0,0,0,
    450,-200,300,-1.571005766,0,-3.140999242,0,0,0)

P.append(pos)

#p4
pos = innertypes.POSTURE(motion.COORD_JOINT,
    1,1,1,0,
    0,0,0,0,0,0,0,0,0,
    0,0,-0.000034907,0,-1.570796327,0,0,0,0)

P.append(pos)

#p5
pos = innertypes.POSTURE(motion.COORD_CARTE,
    1,1,1,0,
    0,0,0,0,0,0,0,0,0,
    380.003,0,298.238,0,0.000034907,3.141592654,0,0,0)

P.append(pos)

#p6
pos = innertypes.POSTURE(motion.COORD_CARTE,
    1,1,1,0,
    0,0,0,0,0,0,0,0,0,
    380.003,-205.254,501.301,-0.000017453,0.000034907,-3.141592654,0,0,0)

P.append(pos)

def demoJ():
    motion.MoveJ(P[0], 1.0, motion.FINE, -1.0, 0.5)
    motion.MoveJ(P[1], 1.0, motion.FINE, -1.0, 0.5)

def demoL():
    motion.MoveJ(P[2], 1.0, motion.FINE, -1.0, 0.5)
    motion.MoveL(P[3], 50.0, motion.FINE, -1.0, 0.5) # mm/s


def demoC():
    motion.MoveJ(P[4], 1.0, motion.FINE, -1.0, 0.5)
    motion.MoveC(P[6], P[5], 50.0, motion.FINE, -1.0, 0.5) # mm/s

def run():
    demoJ()
    demoL()
    demoC()


