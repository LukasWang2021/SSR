import ctypes

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

