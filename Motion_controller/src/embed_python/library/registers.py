# -*- coding: utf-8 -*-
"""
Created on Sun Jan 19 14:44:49 2020

@author: wuym

__add__(self,another)        self + rhs        加法
__sub__(self,another)        self - rhs        减法
__mul__(self,another)        self * rhs        乘法
__truediv__(self,another)    self / rhs        除法
__floordiv__(self,another)   self // rhs       地板除
__mod__(self,another)        self % rhs        取模(求余)
__pow__(self,another)        self ** rhs       幂运算
__lt__(self,another)         self < rhs        小于
__gt__(self,another)         self > rhs        大于
__le__(self,another)         self <= rhs       小于等于
 __ge__(self,another)        self >= rhs       大于等于
__eq__(self,another)         self == rhs       等于
__ne__(self,another)         self != rhs       不等于
"""

class RegP:
    def update(self):
        pass
    
    def __init__(self, index=0, val=0):
        if index == 0:
            pass
        else:
            #do ctype call first
            pass
        self.idx = index
        self.name = "default"
        self.comment = "default"
        self.value = {
            "coord": 0,#1 joint, 2 cart
            "position": [0,0,0,0,0,0,0,0,0],
            "turn": [0,0,0,0,0,0,0,0,0],
            "posture": [0,0,0,0],
            "group": 0,
        }

        self.x = 0; self.j1 = 0
        self.y = 0; self.j2 = 0
        self.z = 0; self.j3 = 0
        self.a = 0; self.j4 = 0
        self.b = 0; self.j5 = 0
        self.c = 0; self.j6 = 0
        self.j7 = 0
        self.j8 = 0
        self.j9 = 0
        pass

    def __add__(self, another):
        rtn = RegP(0)
        for count in range(len(self.value["position"])):
            rtn.value["position"][count] = self.value["position"][count] + another.value["position"][count]
        return rtn
    
    def __iadd__(self, another):
        for count in range(len(self.value["position"])):
            self.value["position"][count] += another.value["position"][count]
        return self
    
    def __sub__(self, another):
        rtn = RegP(0)
        for count in range(len(self.value["position"])):
            rtn.value["position"][count] = self.value["position"][count] - another.value["position"][count]
        return rtn
    
    def __isub__(self, another):
        for count in range(len(self.value["position"])):
            self.value["position"][count] -= another.value["position"][count]
        return self
   
    def __setattr__(self, item, data=0):
        #print(data, type(data))
        if isinstance(data, RegR) or isinstance(data, RegM):
            self.__dict__[item] = data.value
        elif isinstance(data, int) or isinstance(data, float):
            self.__dict__[item] = data
        elif isinstance(data, RegP):
            pass
        elif isinstance(data, str):
            pass
        elif isinstance(data, dict):
            pass
        elif isinstance(data, type):
            pass
        else:
            #print(type(data), data)
            print("error type at RegP")
            
    def __getattr__(self, item):
        return self.__dict__[item]


class RegR:
    idx = 0
    name = "default"
    comment = "default"
    value = 0.0
    def __init__(self, index=0):
        if index == 0:
            pass
        else:
            #do ctype call first
            pass
        # self.idx = index
        # self.name = "default"
        # self.comment = "default"
        # self.value = 0.0
        pass
    
    def __add__(self, another):
        rtn = RegR(0)
        if isinstance(another, int) or isinstance(another, float):
            rtn.value = self.value + another
        elif isinstance(another, RegR):
            rtn.value = self.value + another.value
        else:
            print("error type")
        return rtn
    
    def __radd__(self, another):
        rtn = RegR(0)
        if isinstance(another, int) or isinstance(another, float):
            rtn.value = self.value + another
        elif isinstance(another, RegR):
            rtn.value = self.value + another.value
        else:
            print("error type")
        return rtn
    
    def __iadd__(self, another):
        if isinstance(another, float) or isinstance(another, int):
            self.value += another
        elif isinstance(another, RegR):
            self.value += another.value
        else:
            print("error type in RegR")
            return
        return self
    
    def __sub__(self, another):
        rtn = RegR(0)
        rtn.value = self.value - another.value
        return rtn
    
    def __isub__(self, another):
        self.value -= another.value
        return self
    
    def __eq__(self, another):
        if isinstance(another, float) or isinstance(another, int):
            return self.value == another
        elif isinstance(another, RegR):
            return self.value == another.value
        else:
            return False
        
class RegM:
    idx = 0
    name = "default"
    comment = "default"
    value = 0
    def __init__(self, index=0):
        if index == 0:
            pass
        else:
            #do ctype call first
            pass
        # self.idx = index
        # self.name = "default"
        # self.comment = "default"
        # self.value = 0
        pass
    
    def __add__(self, another):
        rtn = RegR(0)
        rtn.value = self.value + another.value
        return rtn
    
    def __iadd__(self, another):
        if isinstance(another, float) or isinstance(another, int):
            self.value += another
        elif isinstance(another, RegM):
            self.value += another.value
        else:
            print("error type")
            return
        return self
    
    def __sub__(self, another):
        rtn = RegR(0)
        rtn.value = self.value - another.value
        return rtn
    
    def __isub__(self, another):
        self.value -= another.value
        return self
    
    def __eq__(self, another):
        if isinstance(another, float) or isinstance(another, int):
            return self.value == another
        elif isinstance(another, RegR):
            return self.value == another.value
        else:
            return False
    
class RegS:
    idx = 0
    name = "default"
    comment = "default"
    value = ""
    def __init__(self, index=0):
        if index == 0:
            pass
        else:
            #do ctype call first
            pass
        # self.idx = index
        # self.name = "default"
        # self.comment = "default"
        # self.value = ""
        pass
    
    def __add__(self, another):
        rtn = RegR(0)
        rtn.value = self.value + another.value
        return rtn
    
    def __iadd__(self, another):
        self.value += another.value
        return self

class PrList:
    regs = {0:RegP(0)}
    def __init__(self):
        for count in range(2):
            self.regs.update({count:RegP(count)})
        pass
    
    def __getitem__(self, index):
        #call c function
        #self.regs.update{index:data}
        return self.regs[index]

    def __setitem__(self, index, data):
        self.regs[index].value["position"] = data.value["position"]
        #call c function to update
        pass
    
PR = PrList()

class RrList:
    regs = {}
    def __init__(self):
        for count in range(10):
            self.regs.update({count:RegR(count)})
        pass
    
    def __getitem__(self, index):
        if(isinstance(index, RegM) or isinstance(index, RegR)):
            return self.regs[index.value]
        #call c function
        #self.regs.update{index:data}
        return self.regs[index]

    def __setitem__(self, index, data):
        if isinstance(data, float) or isinstance(data, int):
            self.regs[index].value = data
        elif isinstance(data, RegR) or isinstance(data, RegM):
            self.regs[index].value = data.value
        else:
            print("error type")
        #call c function to update
        pass
R = RrList()

class MrList:
    regs = {}
    def __init__(self):
        for count in range(10):
            self.regs.update({count:RegM(count)})
        pass
    
    def __getitem__(self, index):
        #call c function
        #self.regs.update{index:data}
        return self.regs[index]

    def __setitem__(self, index, data):
        if isinstance(data, float) or isinstance(data, int):
            self.regs[index].value = data
        elif isinstance(data, RegR) or isinstance(data, RegM):
            self.regs[index].value = int(data.value)
        else:
            print("error type")
        #call c function to update
        pass
MR = MrList()


class SrList:
    regs = {}
    def __init__(self):
        for count in range(10):
            self.regs.update({count:RegS(count)})
        pass
    
    def __getitem__(self, index):
        #call c function
        #self.regs.update{index:data}
        return self.regs[index]

    def __setitem__(self, index, data):
        if isinstance(data, RegS):
            self.regs[index].value = data
        else:
            self.regs[index].value = str(data)
        #call c function to update
        pass
SR = SrList()
    
    
