import time 
import ctypes
import register as reg

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

def testR():
    reg.SetRR(1,100)
    a = reg.GetRR(1)
    print(a)
    if a == 100:
        print("RR SUCCESS")
    else:
        print("RR FAIL")

def testMR():
    reg.SetMR(2,200)
    b = reg.GetMR(2)
    print(b)
    if b == 200:
        print("MR SUCCESS")
    else:
        print("MR FAIL")

def testSR():
    sr = "hello"
    reg.SetSR(3,sr)
    c = reg.GetSR(3)
    print(c)
    if c == "hello":
        print("SR SUCCESS")
    else:
        print("SR FAIL")

def testPR():
    pr = POSTURE(1,2,3,4,5,1,2,3,4,5,6,7,8,9,1,2,3,4,5,6,7,8,9)
    reg.SetPR(4,pr)
    d = reg.GetPR(4)
    print(d)
#    if cmp(d, pr) == 0:
#        print("PR SUCCESS")
#    else:
#        print("PR FAIL")

#PR[1] = PR[2]