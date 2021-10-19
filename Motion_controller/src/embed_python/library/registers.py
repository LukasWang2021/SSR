# -*- coding: utf-8 -*-
"""
Created on Sun Jan 19 14:44:49 2020

@author: wuym

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
    def __str__(self):
        return str(self.value)

    def __add__(self, another):
        for count in range(len(self.value["position"])):
            self.value["position"][count] = self.value["position"][count] + another.value["position"][count]
        return self
    
    def __iadd__(self, another):
        for count in range(len(self.value["position"])):
            self.value["position"][count] += another.value["position"][count]
        return self
    
    def __sub__(self, another):
        for count in range(len(self.value["position"])):
            self.value["position"][count] = self.value["position"][count] - another.value["position"][count]
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
    def __str__(self):
        return str(self.value)
    def __add__(self, another):
        if isinstance(another, int) or isinstance(another, float):
            self.value = self.value + another
        elif isinstance(another, RegR):
            self.value = self.value + another.value
        else:
            print("error type:::RegR __add__")
        return self
    
    def __radd__(self, another):
        if isinstance(another, int) or isinstance(another, float):
            self.value = self.value + another
        elif isinstance(another, RegR):
            self.value = self.value + another.value
        else:
            print("error type:::RegR __radd__")
        return self
    
    def __iadd__(self, another):
        if isinstance(another, float) or isinstance(another, int):
            self.value += another
        elif isinstance(another, RegR):
            self.value += another.value
        else:
            print("error type:::RegR __iadd__")
            return
        return self
    
    def __sub__(self, another):
        if isinstance(another, int) or isinstance(another, float):
            self.value = self.value - another
        elif isinstance(another, RegR) or isinstance(another,RegM):    
            self.value = self.value - another.value
        else:
            print("error type:::RegR __sub__")
        return self
    def __rsub__(self, another):
        if isinstance(another, int) or isinstance(another, float):
            self.value = another - self.value
        else:
            print("error type:::RegR __rsub__")   
        return self
    def __isub__(self,another):
        if isinstance(another, int) or isinstance(another, float):
            self.value -= another
        elif isinstance(another, RegR):    
            self.value -= another.value
        else:
            print("error type:::RegR __isub__")   
        return self
    def __mul__(self, another):
        if isinstance(another, int) or isinstance(another, float):
            self.value = self.value * another
        elif isinstance(another, RegR) or isinstance(another,RegM):
            self.value = self.value * another.value
        else:
            print("error type:::RegR __mul__")
        return self
    def __truediv__(self,another):
        if isinstance(another, int) or isinstance(another, float):
            self.value = self.value / another
        elif isinstance(another, RegR) or isinstance(another,RegM):
            self.value = self.value / another.value
        else:
            print("error type:::RegR __truediv__")
        return self
    def __floordiv__(self,another):  
        if isinstance(another, int) or isinstance(another, float):
            self.value = self.value // another
        elif isinstance(another, RegR) or isinstance(another,RegM):
            self.value = self.value // another.value
        else:
            print("error type::: RegR __floordiv__")
        return self
    def __mod__(self,another):
        if isinstance(another, int) or isinstance(another, float):
            self.value = self.value % another
        elif isinstance(another, RegR) or isinstance(another,RegM):
            self.value = self.value % another.value
        else:
            print("error type::: RegR __mod__")
        return self
    def __pow__(self,another):
        if isinstance(another, int) or isinstance(another, float):
            self.value = self.value ** another
        elif isinstance(another, RegR) or isinstance(another,RegM):
            self.value = self.value ** another.value
        else:
            print("error type:::RegR __pow__")
        return self
    def __lt__(self,another):
        if isinstance(another, int) or isinstance(another, float):
            return  self.value < another
        elif isinstance(another, RegR) or isinstance(another,RegM):
            return self.value < another.value
        else:
            print("error type:::RegR __lt__")
            return False
    def __le__(self, another):
        if isinstance(another, float) or isinstance(another, int):
            return self.value <= another
        elif isinstance(another, RegR) or isinstance(another,RegM):
            return self.value <= another.value
        else:
            print("error type:::RegR __le__")
            return False     
    def __gt__(self, another):
        if isinstance(another, float) or isinstance(another, int):
            return self.value > another
        elif isinstance(another, RegR) or isinstance(another,RegM) :
            return self.value > another.value
        else:
            print("error type:::RegR __gt__")
            return False
    def __ge__(self, another):
        if isinstance(another, float) or isinstance(another, int):
            return self.value >= another
        elif isinstance(another, RegR) or isinstance(another,RegM):
            return self.value >= another.value
        else:
            print("error type:::RegR __ge__")
            return False
    
    def __eq__(self, another):
        if isinstance(another, float) or isinstance(another, int):
            return self.value == another
        elif isinstance(another, RegR) or isinstance(another,RegM):
            return self.value == another.value
        else:
            print("error type:::RegR __eq__")
            return False
    def __ne__(self, another):
        if isinstance(another, float) or isinstance(another, int):
            return self.value != another
        elif isinstance(another, RegR) or isinstance(another,RegM):
            return self.value != another.value
        else:
            print("error type:::RegR __ne__")
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
    def __str__(self):
        return str(self.value)

    def __add__(self, another):
        if isinstance(another, float) or isinstance(another, int):
            self.value = int(self.value + another)
        elif isinstance(another, RegM):
            self.value = self.value + another.value
        elif isinstance(another, RegR):
            self.value = self.value + int(another.value)
        else:
            print("error type::: RegM __add__")
        return self
    def __radd__(self, another):
        if isinstance(another, float) or isinstance(another, int):
            self.value = int(self.value + another)
        elif isinstance(another, RegM):
            self.value = self.value + another.value
        elif isinstance(another, RegR):
            self.value = self.value + int(another.value)
        else:
            print("error type::: RegM __radd__")
        return self
    def __iadd__(self, another):
        if isinstance(another, float) or isinstance(another, int):
            self.value += int(another)
        elif isinstance(another, RegM):
            self.value += another.value
        else:
            print("error type::: RegM __iadd__")
        return self
    
    def __sub__(self, another):
        if isinstance(another, float) or isinstance(another, int):
            self.value = int(self.value - another)
        elif isinstance(another, RegM):
            self.value = self.value - another.value
        elif isinstance(another, RegR):
            self.value = self.value - int(another.value)
        else:
            print("error type::: RegM __sub__")
        return self
    
    def __isub__(self, another):
        if isinstance(another, float) or isinstance(another, int):
            self.value -= int(another)
        elif isinstance(another, RegM):
             self.value -= another.value
        else:
            print("error type::: RegM __isub__")
        return self
    def __mul__(self, another):
        if isinstance(another, int) or isinstance(another, float):
            self.value = int(self.value * another)
        elif isinstance(another, RegM):
            self.value = self.value * another.value
        elif isinstance(another, RegR):
            self.value = int(self.value * another.value)
        else:
            print("error type::: RegM __mul__")
        return self
    def __truediv__(self,another):
        if isinstance(another, int) or isinstance(another, float):
            self.value = int(self.value / another)
        elif isinstance(another, RegR) or isinstance(another,RegM):
            self.value = int(self.value / another.value)
        else:
            print("error type::: RegM __truediv__")
        return self
    def __floordiv__(self,another):  
        if isinstance(another, int) or isinstance(another, float):
            self.value = self.value // another
        elif isinstance(another, RegR) or isinstance(another,RegM):
            self.value = self.value // another.value
        else:
            print("error type::: RegM __floordiv__")
        return self
    def __mod__(self,another):
        if isinstance(another, int) or isinstance(another, float):
            self.value = int(self.value % another)
        elif isinstance(another, RegR) or isinstance(another, RegM):
            self.value = int(self.value % another.value)
        else:
            print("error type::: RegM __mod_")
        return self
    def __pow__(self,another):
        if isinstance(another, int) or isinstance(another, float):
            self.value = int(self.value ** another)
        elif isinstance(another, RegR) or isinstance(another, RegM):
            self.value = self.value ** another.value
        else:
            print("error type::: RegM __pow__")
        return self
    def __lt__(self,another):
        if isinstance(another, int) or isinstance(another, float):
            return  self.value < another
        elif isinstance(another, RegR) or isinstance(another, RegM):
            return self.value < another.value
        else:
            print("error type::: RegM __lt__")
            return False
    def __le__(self, another):
        if isinstance(another, float) or isinstance(another, int):
            return self.value <= another
        elif isinstance(another, RegR) or isinstance(another, RegM):
            return self.value <= another.value
        else:
            print("error type::: RegM __le__")
            return False       
    def __gt__(self, another):
        if isinstance(another, float) or isinstance(another, int):
            return self.value > another
        elif isinstance(another, RegR) or isinstance(another, RegM):
            return self.value > another.value
        else:
            print("error type::: RegM __gt__")
            return False
    def __ge__(self, another):
        if isinstance(another, float) or isinstance(another, int):
            return self.value >= another
        elif isinstance(another, RegR) or isinstance(another, RegM):
            return self.value >= another.value
        else:
            print("error type::: RegM __ge__")
            return False  
    def __eq__(self, another):
        if isinstance(another, int):
            return self.value == another
        elif isinstance(another, RegR) or isinstance(another, RegM):
            return self.value == another.value
        else:
            print("error type::: RegM __eq__")
            return False
    def __ne__(self, another):
        if isinstance(another, float) or isinstance(another, int):
            return self.value != another
        elif isinstance(another, RegR) or isinstance(another, RegM):
            return self.value != another.value
        else:
            print("error type::: RegM __ne__")
            return False
class RegS:
    # idx = 0
    # name = "default"
    # comment = "default"
    # value = ""
    def __init__(self, index=0):
        if index == 0:
            pass
        else:
            #do ctype call first
            pass
        self.idx = index
        self.name = "default"
        self.comment = "default"
        self.value = ""
        pass
    def __str__(self):
        return self.value

    def __add__(self, another):
        if isinstance(another, str) :
           self.value = self.value + another
        elif isinstance(another, RegS):
            self.value = self.value + another.value
        else:
            print("error type::: RegS __add__")
        return self
    def __radd__(self, another):
        if isinstance(another, str) :
           self.value = self.value + another
        elif isinstance(another, RegS):
            self.value = self.value + another.value
        else:
            print("error type::: RegS __radd__")
        return self
    def __iadd__(self, another):
        if isinstance(another, str) :
           self.value += another
        elif isinstance(another, RegS):
           self.value += another.value
        else:
            print("error type::: RegS __iadd__")
        return self
    def __mul__(self,another):
        if isinstance(another, int):
            self.value = self.value * another
        elif isinstance(another, RegM):
            self.value = self.value * another.value
        else:
            print("error type::: RegS __mul__")
        return self
    def __eq__(self, another):
        if isinstance(another, str):
            return self.value == another
        elif isinstance(another, RegS):
            return self.value == another.value
        else:
            print("error type::: RegS __eq__")
            return False
    def __ne__(self, another):
        if isinstance(another, str):
            return self.value != another
        elif isinstance(another, RegS):
            return self.value != another.value
        else:
            print("error type::: RegS __ne__")
            return False
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

def is_number(s):   
    try:
        float(s)
        return True
    except ValueError:
        pass
    try:
        import unicodedata
        unicodedata.numeric(s)
        return True
    except (TypeError, ValueError):
        pass
    return False

#函数功能: 提取字符串中第一个数字,支持科学计数法. 如果字符串中不包含数字则返回0
def getNumFromString(sss):
	slen = len(sss)
	ret_value = 0 #返回值
	sflag = 0 #标记是否遇到第一个数字字符 0-未遇到 1-已遇到
	zflag = 1 #数字整数部分标记  默认1-整数
	pnflag = 1 #正负号标记   默认1-正数
	scientificEnumerationFlag = 0 #科学计数法标记
	cnt = 1 #小数部分长度计数
	power_num = 0 #幂
	power_pnFlag = 1
	#print("字符串长度=%s"%slen)
	for i in range(slen):
		if sss[i].isdigit():
			sflag = 1
			if scientificEnumerationFlag:
				power_num = power_num*10 + int(sss[i])
				#print("current step power_num=%s"%power_num)
			else:
				if zflag == 1:
					ret_value = ret_value*10 + int(sss[i])
					#print(ret_value)
				else:	
					divnum = 10**cnt
					ret_value = ret_value + float(sss[i])/(divnum)
					#print("div_num=%s, current step ret_value=%s"%(divnum,ret_value))
					cnt+=1
		else:
			if sflag == 1:
				if sss[i] == '.':
					zflag = 0
				elif sss[i] == 'e' and (sss[i+1] == '+' or sss[i+1] == '-'):
					scientificEnumerationFlag = 1 #开启科学计数法
					if(sss[i+1] == '+'):
						power_pnFlag = 1
					else:
						power_pnFlag = -1
					#print("幂的符号=%s"%power_pnFlag)
				else:
					if sss[i-1] == 'e' and (sss[i] == '+' or sss[i] == '-'):
						continue
					else:
						#print("-------------break------------")
						break
			else:
				if sss[i] == '-':
					pnflag = -1
	return pnflag*ret_value*(10**(power_num*power_pnFlag)) #符号*返回值*(10**(幂*幂的符号))

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
        elif isinstance(data,RegS):
            self.regs[index].value = getNumFromString(data.value)
        elif isinstance(data, str):
            self.regs[index].value = float(getNumFromString(data))
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
            self.regs[index].value = int(data)
        elif isinstance(data, RegR) or isinstance(data, RegM):
            self.regs[index].value = int(data.value)
        elif isinstance(data,RegS):
            self.regs[index].value = int(getNumFromString(data.value))
        elif isinstance(data, str):
            self.regs[index].value = int(getNumFromString(data))
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
            self.regs[index].value = data.value
        else:
            self.regs[index].value = str(data)
        #call c function to update
        pass
SR = SrList()
    
    
