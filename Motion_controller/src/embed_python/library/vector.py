
class Vector:
    def __init__(self, val=[]):
        self.value = val
        pass

    def __add__(self, right:list):
        ret = []
        for i in range(len(self.value)):
            ret.append(self.value[i] + right.value[i])
        return Vector(ret)
    
    def __sub__(self, right):
        ret = []
        for i in range(len(self.value)):
            ret.append(self.value[i] - right.value[i])
        return Vector(ret)

    def skew(self):
        ret = [[0.0,            -self.value[2], self.value[1]],
               [self.value[2],  0.0,              -self.value[0]],
               [-self.value[1], self.value[0],  0.0]]
        return ret
    
    def skew_append_eye(self):
        ret = [[0.0,                -self.value[2], self.value[1], 1.0, 0.0, 0.0],
               [self.value[2],     0.0,            -self.value[0], 0.0, 1.0, 0.0],
               [-self.value[1],    self.value[0], 0.0,             0.0, 0.0, 1.0]]
        return ret

    def cross(self):
        pass



