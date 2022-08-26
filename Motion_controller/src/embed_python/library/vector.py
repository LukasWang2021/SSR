class Vector:
    def __init__(self):
        self.value = []
        pass

    def __add__(self, right:list):
        ret = Vector()
        for i in range(len(self.value)):
            ret.value.append(self.value[i] + right.value[i])
        return ret
    
    def __sub__(self, right):
        ret = Vector()
        for i in range(len(self.value)):
            ret.value.append(self.value[i] - right.value[i])
        return ret

    def skew(self):
        ret = [[0,              -self.value[2], self.value[1]],
               [self.value[2],  0,              -self.value[0]],
               [-self.value[1], self.value[0],  0]]
        return ret
    
    def skew_append_eye(self):
        ret = [[0,                -self.value[2], self.value[1], 1, 0, 0],
               [self.value[2],     0,            -self.value[0], 0, 1, 0],
               [-self.value[1],    self.value[0], 0,             0, 0, 1]]
        return ret



