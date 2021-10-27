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
import ctypes
import register as reg

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

class POSTURE(ctypes.Structure):
    _fields_ = [('coord',ctypes.c_int), 
                ('arm',ctypes.c_int),
                ('elbow',ctypes.c_int),
                ('wrist',ctypes.c_int),
                ('flip',ctypes.c_int),
                ('turn1',ctypes.c_int),
                ('turn2',ctypes.c_int),
                ('turn3',ctypes.c_int),
                ('turn4',ctypes.c_int),
                ('turn5',ctypes.c_int),
                ('turn6',ctypes.c_int),
                ('turn7',ctypes.c_int),
                ('turn8',ctypes.c_int),
                ('turn9',ctypes.c_int),
                ('pos1',ctypes.c_double),
                ('pos2',ctypes.c_double),
                ('pos3',ctypes.c_double),
                ('pos4',ctypes.c_double),
                ('pos5',ctypes.c_double),
                ('pos6',ctypes.c_double),
                ('pos7',ctypes.c_double),
                ('pos8',ctypes.c_double),
                ('pos9',ctypes.c_double),]

class RegP:
    def update(self):
        pass
    
    def __init__(self, index=0, val=0):
        self.idx = index
        self.name = "default"
        self.comment = "default"
        self.value = {
            "coord": 0,#1 joint, 2 cart
            "posture": [0,0,0,0],
            "turn": [0,0,0,0,0,0,0,0,0],
            "position": [0,0,0,0,0,0,0,0,0],
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
    '''
    def __setattr__(self, item, data=0):
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
        elif isinstance(data, tuple):
            self.value
        else:
            print(type(data), data)
            print("error type at RegP")
            
    def __getattr__(self, item):
        print("===============> item=", type(item))
        print("===============> dict=", self.__dict__)
        return self.__dict__[item]
    '''
     
    def setValue(self,spd):
        setPrData = POSTURE()
        if isinstance(spd, dict):
            setPrData.coord = spd['coord']
            setPrData.arm   = spd['posture'][0]
            setPrData.elbow = spd['posture'][1]
            setPrData.wrist = spd['posture'][2]
            setPrData.flip  = spd['posture'][3]
            setPrData.turn1 = spd['turn'][0]
            setPrData.turn2 = spd['turn'][1]
            setPrData.turn3 = spd['turn'][2]
            setPrData.turn4 = spd['turn'][3]
            setPrData.turn5 = spd['turn'][4]
            setPrData.turn6 = spd['turn'][5]
            setPrData.turn7 = spd['turn'][6]
            setPrData.turn8 = spd['turn'][7]
            setPrData.turn9 = spd['turn'][8]
            setPrData.pos1 = spd['position'][0]
            setPrData.pos2 = spd['position'][1]
            setPrData.pos3 = spd['position'][2]
            setPrData.pos4 = spd['position'][3]
            setPrData.pos5 = spd['position'][4]
            setPrData.pos6 = spd['position'][5]
            setPrData.pos7 = spd['position'][6]
            setPrData.pos8 = spd['position'][7]
            setPrData.pos9 = spd['position'][8]

            reg.SetPR(self.idx, setPrData)
        else:
            print("RegP setValue input typeerror")
        
    def getValue(self):
        Tt = reg.GetPR(self.idx) #获取返回元组数据
        self.value['coord'] = Tt[0]
        self.value['posture'] = Tt[1:5] #1,2,3,4
        self.value['turn'] = Tt[5:14]#5,6,7,8,9,10,11,12,13,14
        self.value['position'] = Tt[14:]#14~22
        return self.value


class RegR:
    def __init__(self, index):
        self.idx = index
        # self.name = "default"
        # self.comment = "default"
        self.value = 0.0
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
            reg.SetRR(self.idx, self.value)
        elif isinstance(another, RegR):
            self.value += another.value
            reg.SetRR(self.idx, self.value)
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
            reg.SetRR(self.idx, self.value)
        elif isinstance(another, RegR):    
            self.value -= another.value
            reg.SetRR(self.idx, self.value)
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
    def setValue(self,ddd):
        reg.SetRR(self.idx, ddd)
    def getValue(self):
        self.value = reg.GetRR(self.idx)
        return self.value
class RegM:
    def __init__(self, index):
        self.idx = index
        # self.name = "default"
        # self.comment = "default"
        self.value = 0
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
            reg.SetMR(self.idx, self.value)
        elif isinstance(another, RegM):
            self.value += another.value
            reg.SetMR(self.idx, self.value)
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
            reg.SetMR(self.idx, self.value)
        elif isinstance(another, RegM):
            self.value -= another.value
            reg.SetMR(self.idx, self.value)
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
    def setValue(self,ddd):
        #print("set MR[%d]=%d"%(self.idx,ddd))
        return reg.SetMR(self.idx, ddd)
    def getValue(self):
        self.value = reg.GetMR(self.idx)
        return self.value

class RegS:
    # idx = 0
    # name = "default"
    # comment = "default"
    # value = ""
    def __init__(self, index):
        self.idx = index
        #self.name = "default"
        #self.comment = "default"
        self.value = ""
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
           reg.SetSR(self.idx, self.value)
        elif isinstance(another, RegS):
           self.value += another.value
           reg.SetSR(self.idx, self.value)
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
    def setValue(self,ssss):
        #print("set SR[%d] = %s"%(self.idx, ssss))
        reg.SetSR(self.idx, ssss)
    
    def getValue(self):
        SR_n = self.idx
        #print("SR---getvalue, SR_n=%s----"%SR_n)
        #self.value = reg.GetSR(self.idx).encode('utf-8').strip()
        ret_getSR = reg.GetSR(SR_n)
        #print("ret_getSR = %s"%ret_getSR) 
        return ret_getSR
class PrList:
    regs = {0:RegP(0)}
    def __init__(self):
        for count in range(3): #1,2
            self.regs.update({count:RegP(count)})
        pass
    
    def __getitem__(self, index):
        t_index = 1
        if isinstance(index,RegM):
            t_index = index.value
        elif isinstance(index,RegR):
            t_index = int(index.value)
        elif isinstance(index,RegS):
            t_index = int(getNumFromString(index.value))
        elif isinstance(index,str):
            t_index = int(getNumFromString(index)) 
        else:
            t_index = int(index)
        return self.regs[t_index].getValue()

    def __setitem__(self, index, data):
        set_data = 0
        s_index = 1
        if isinstance(index,RegM):
            s_index = index.value
        elif isinstance(index,RegR):
            s_index = int(index.value)
        elif isinstance(index,RegS):
            s_index = int(getNumFromString(index.value))
        elif isinstance(index,str):
            s_index = int(getNumFromString(index)) 
        else:
            s_index = int(index)
        if isinstance(data, RegP):
            #print("PrList __setitem__ data type=RegP")
            set_data = data.value #self.regs[index].value["position"] = data.value["position"]
        elif isinstance(data, tuple):
            #print("PrList __setitem__ data type=tuple")
            set_data = data
        elif isinstance(data, dict):
            #print("PrList __setitem__ data type=dict")
            set_data = data
        else:
            print("PrList __setitem__ data type error!!! type(data)=%s"%type(data))
        self.regs[s_index].setValue(set_data)
    
PR = PrList() 


class RrList:
    regs = {}
    def __init__(self):
        for count in range(1,4):#0不可用
            self.regs.update({count:RegR(count)})
        pass
    
    def __getitem__(self, index):
        t_index = 1
        if isinstance(index,RegM):
            t_index = index.value
        elif isinstance(index,RegR):
            t_index = int(index.value)
        elif isinstance(index,RegS):
            t_index = int(getNumFromString(index.value))
        elif isinstance(index,str):
            t_index = int(getNumFromString(index)) 
        else:
            t_index = int(index)
        #print("t_index = %s"%t_index)
        return self.regs[t_index].getValue()

    def __setitem__(self, index, data):
        setPRData = 0.0
        s_index = 1
        if isinstance(index,RegM):
            s_index = index.value
        elif isinstance(index,RegR):
            s_index = int(index.value)
        elif isinstance(index,RegS):
            s_index = int(getNumFromString(index.value))
        elif isinstance(index,str):
            s_index = int(getNumFromString(index)) 
        else:
            s_index = int(index)
        if isinstance(data, float) or isinstance(data, int):
            setPRData = data
        elif isinstance(data, RegR) or isinstance(data, RegM):
            setPRData = data.value
        elif isinstance(data,RegS):
            setPRData = float(getNumFromString(data.value))
        elif isinstance(data, str):
            setPRData = float(getNumFromString(data))
        else:
            print("error type")
        self.regs[s_index].setValue(setPRData)

R = RrList()

class MrList:
    regs = {}
    def __init__(self):
        for count in range(1,4):# 0不可用
            self.regs.update({count:RegM(count)})
        pass
    
    def __getitem__(self, index):
        t_index = 1
        if isinstance(index,RegM):
            t_index = index.value
        elif isinstance(index,RegR):
            t_index = int(index.value)
        elif isinstance(index,RegS):
            t_index = int(getNumFromString(index.value))
        elif isinstance(index,str):
            t_index = int(getNumFromString(index)) 
        else:
            t_index = int(index)
        #print("t_index = %s"%t_index)
        return self.regs[t_index].getValue()

    def __setitem__(self, index, data):
        set_data = 0
        s_index = 1
        if isinstance(index,RegM):
            s_index = index.value
        elif isinstance(index,RegR):
            s_index = int(index.value)
        elif isinstance(index,RegS):
            s_index = int(getNumFromString(index.value))
        elif isinstance(index,str):
            s_index = int(getNumFromString(index)) 
        else:
            s_index = int(index)
        if isinstance(data, float) or isinstance(data, int):
            set_data= int(data)
        elif isinstance(data, RegR) or isinstance(data, RegM):
            set_data = int(data.value)
        elif isinstance(data,RegS):
            set_data = int(getNumFromString(data.value))
        elif isinstance(data, str):
            set_data = int(getNumFromString(data))
        else:
            print("error type") 
        #print("__setitem__ MR[%d]=%d"%(index,set_data))
        self.regs[s_index].setValue(set_data)
MR = MrList()

class SrList:
    regs = {}
    def __init__(self):
        for count in range(1,4):   #SR[0]不可用, 实际只有1~3
            self.regs.update({count:RegS(count)})
        pass
    def __getitem__(self, index):
        t_index = 1
        if isinstance(index,RegM):
            t_index = index.value
        elif isinstance(index,RegR):
            t_index = int(index.value)
        elif isinstance(index,RegS):
            t_index = int(getNumFromString(index.value))
        elif isinstance(index,str):
            t_index = int(getNumFromString(index)) 
        else:
            t_index = int(index)
        #print("t_index = %s"%t_index)
        return self.regs[t_index].getValue()

    def __setitem__(self, index, data):
        set_string = ""
        s_index = 1
        if isinstance(index,RegM):
            s_index = index.value
        elif isinstance(index,RegR):
            s_index = int(index.value)
        elif isinstance(index,RegS):
            s_index = int(getNumFromString(index.value))
        elif isinstance(index,str):
            s_index = int(getNumFromString(index)) 
        else:
            s_index = int(index)
        if isinstance(data, RegS):
            set_string = data.value   
        else:
            set_string = str(data)
        self.regs[s_index].setValue(set_string)

SR = SrList() #注意:SR[i] i从1开始, SR[0]不可用
    
    



