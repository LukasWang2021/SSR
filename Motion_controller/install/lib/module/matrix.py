"""
__add__(self,another)        self + rhs        加法
__iadd__(self,another)       self++            自增    
__radd__(self,another)       another+self      右加
__sub__(self,another)        self - rhs        减法
__isub__(self,another)       self--            自减
__mul__(self,another)        self * rhs        乘法
__truediv__(self,another)    self / rhs        除法
__floordiv__(self,another)   self // rhs       向下整除
__mod__(self,another)        self % rhs        取模(求余)
__pow__(self,another)        self ** rhs       幂运算
__lt__(self,another)         self < rhs        小于
__gt__(self,another)         self > rhs        大于
__le__(self,another)         self <= rhs       小于等于
 __ge__(self,another)        self >= rhs       大于等于
__eq__(self,another)         self == rhs       等于
__ne__(self,another)         self != rhs       不等于
"""

import linealg

class Matrix():
    def __init__(self):
        self.value = []
        pass
    
    def __add__(self, right):
        pass
    
    def __sub__(self, right):
        pass
    
    def __mul__(self, right):
        pass
    
    def cross(self, right):
        pass

    def inv(self):
        pass
    
    def eigens(self):
        pass