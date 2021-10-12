
import device

ON = 1
OFF = 0

class DeviceDO:
    def __init__(self, idx):
        self.idx = idx
        self.value = 0
        pass

    def get_do_bit(self):
        self.value = device.GetDO(self.idx)
        return self.value
    
    def set_do_bit(self, n_pin, val):
        return device.SetDO(n_pin,val)


class DeviceDI:
    def __init__(self,idx):
        self.idx = idx
        self.value = 0

    def get_di_bit(self):  
        self.value = device.GetDI(self.idx)  
        return self.value


class DoList:
    ios = {}  
    def __init__(self):
        for counts in range(10):
            self.ios.update({counts:DeviceDO(counts)}) 
        pass

    def __getitem__(self,item):
        return self.ios[item].get_do_bit()
    
    def __setitem__(self, key, value):
        return self.ios[key].set_do_bit(key, value)


class DiList:
    ios = {}   
    def __init__(self):
        for counts in range(10):
            self.ios.update({counts:DeviceDI(counts)}) 
        pass

    def __getitem__(self,item):
        return self.ios[item].get_di_bit()


DI = DiList()
DO = DoList()


