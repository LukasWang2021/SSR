# -*- coding: utf-8 -*-
"""
Created on Sun Jan 19 14:05:22 2020

@author: wuym
"""
import time

# WAIT Null R[1] warning
# WAIT R[1]==1 Null Null
# WAIT DO[1]==1 30 warning
# WAIT Null 30 Null
# wait condition true in spec time if not execute the option
# wait time out and execute option
def wait(_condition=None, _timeout=None, _option=None):
    if(_condition==None and _timeout==None):
        print("WARNING:no condition or timeout specified for WAIT")
        return

    if(_condition != None and eval(_condition)):
        return
    else:
        #wait for condition with default timeout
        if(_timeout != None):
            outtime = _timeout
        else:
            outtime = 30 #use default timeout defined in config file

        start_time = end_time = time.time()
        while(end_time - start_time < outtime):
            if(_condition != None and eval(_condition)):#get condition may need sleep
                pass
            end_time = time.time()
        if(_option):#do option
            exec(_option)
        print("wait condition timeout %f" % outtime)
        return

    if(_timeout):
        #wait for timeout
        time.sleep(_timeout)
        #do option
        if(_option):
            exec(_option)
        return

