import motion
import innertypes

def testJ():
    pos = innertypes.POSTURE(motion.COORD_JOINT,
    1,1,1,1,
    0,0,0,0,0,0,0,0,0,
    0.314,0.314,0.314,0.314,0.314,0.314,0,0,0)

    motion.MoveJ(pos, 1.0, motion.FINE, -1.0, 0.5)

def testL():
    pos = innertypes.POSTURE(motion.COORD_CARTE,
    1,1,1,1,
    0,0,0,0,0,0,0,0,0,
    300,0,600,0,0,-3.141592654,0,0,0)

    motion.MoveL(pos, 1.0, motion.FINE, -1.0, 0.5);


def testC():
    via = innertypes.POSTURE(motion.COORD_JOINT,
    1,1,1,1,
    0,0,0,0,0,0,0,0,0,
    0.314,0.314,0.314,0.314,0.314,0.314,0,0,0)

    tgt = innertypes.POSTURE(motion.COORD_JOINT,
    1,1,1,1,
    0,0,0,0,0,0,0,0,0,
    0.314,0.314,0.314,0.314,0.314,0.314,0,0,0)

    motion.MoveL(via, tgt, 1.0, motion.FINE, -1.0, 0.5);