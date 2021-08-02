import motion
import innertypes

def test():
    pos = innertypes.POSTURE(1,1,2,3,4,1,2,3,4,5,6,7,8,9,1,2,3,4,5,6,7,8,9)
    motion.MoveJ(pos,100.0,123,50.0)
