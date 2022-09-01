import linealg

class Matrix():
    def __init__(self, val=[]):
        self.value = val
        pass
    
    def __add__(self, right):
        ret = []
        for i in range(len(self.value)):
            ret.append([])
            for j in range(len(self.value[i])):
                ret[i].append(self.value[i][j] + right.value[i][j])
        return Matrix(ret)
    
    def __sub__(self, right):
        ret = []
        for i in range(len(self.value)):
            ret.append([])
            for j in range(len(self.value[i])):
                ret[i].append(self.value[i][j] - right.value[i][j])
        return Matrix(ret)
    
    def __mul__(self, right):
        m = []
        for i in range(len(self.value)):
            m.append([])
            for j in range(len(right.value[i])):
                sum_val = 0
                for k in range(len(self.value[i])):
                    sum_val += self.value[i][k] * right.value[k][j]
                m[i].append(sum_val)
        return Matrix(m)
    
    def transpose(self):
        m = []
        for i in range(len(self.value[0])):
            m.append([])
            for j in range(len(self.value)):
                m[i].append(self.value[j][i])
        return Matrix(m)

    def inv(self):
        m = linealg.inv(self.value)
        return Matrix(m)

    def eigens(self):
        pass
