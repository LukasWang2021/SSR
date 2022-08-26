import linealg
class Matrix():
    def __init__(self, val=[]):
        self.value = val
        pass
    
    def __add__(self, right):
        ret = Matrix()
        for i in range(len(self.value)):
            ret.value.append([])
            for j in range(len(self.value[i])):
                ret.value[i].append(self.value[i][j] + right.value[i][j])
        return ret
    
    def __sub__(self, right):
        ret = Matrix()
        for i in range(len(self.value)):
            ret.value.append([])
            for j in range(len(self.value[i])):
                ret.value[i].append(self.value[i][j] - right.value[i][j])
        return ret
    
    def __mul__(self, right):
        ret = Matrix()
        for i in range(len(self.value)):
            ret.value.append([])
            for j in range(len(self.value[i])):
                ret.value[i].append(self.value[i][j] - right.value[i][j])
        return ret
    
    def cross(self, right):
        pass
    
    def transpose(self):
        pass

    def inv(self):
        ret = Matrix()
        ret.value = linealg.inv(self.value)
        return ret

    def eigens(self):
        pass
