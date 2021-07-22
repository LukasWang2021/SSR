# -*- coding: utf-8 -*-
"""
Created on Sun Jan 19 14:44:49 2020

@author: wuym
"""
import device

ON = 1
OFF = 0

class IO:
    def __init__(self, index=0):
        if index == 0:
            pass
        else:
            #do ctype call first
            pass
        self.idx = index
        self.value = 0
        pass


class IoList:
    ios = {}
    def __init__(self):
        for counts in range(10):
            self.ios.update({counts:IO(counts)})
        pass
    
  
DI = IoList()
DO = IoList()