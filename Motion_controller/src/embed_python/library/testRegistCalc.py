"""
文档说明:  本文档编写了一些对寄存器赋值及基本运算的测试代码
肖志才测试通过
2021/10/27 
"""

from registers import *
import register as reg

def testSR():
    sr = "hello"
    reg.SetSR(3,sr)
    c = reg.GetSR(3)
    print(c)
    if c == "hello":
        print("SR SUCCESS")
    else:
        print("SR FAIL")
def test_set_SR():
	SR[2] = 'abcdefghijk'
	SR[1] = SR[2]
	if SR[1] == 'abcdefghijk':
		print("1# test SR[1] = SR[2] success")
	else:
		print("1# test SR[1]=SR[2] fail. SR[1]=%s"%SR[1])

def test_set_MR():
	MR[1] = 99.9
	if MR[1] == 99:
		print("2_1# test MR[1]=float success")
	else:
		print("2_1# test MR[1]=float fail. MR[1]=%s"%MR[1])
	MR[2] = 100
	if MR[2] == 100:
		print("2_2# test MR[2]=int success")
	else:
		print("2_2# test MR[2]=int fail. MR[2]=%s"%MR[2])
	R[1] = 99.9
	MR[1] = R[1]
	if MR[1] == 99:
		print("2_3# test MR[1] = R[1] success")
	else:
		print("2_3# test MR[1] = R[1] fail. MR[3]=%s"%MR[1])
	MR[1] = 65535
	MR[2] = MR[1]
	if MR[2] == 65535:
		print("2_4# test MR[2] = MR[1] success")
	else:
		print("2_4# test MR[2] = MR[1] fail. MR[2]=%s"%MR[2])
	SR[1] = "abc123fge"
	MR[1] = SR[1]
	if MR[1] == 123:
		print("2_4# test MR[1]=SR[1] success")
	else:
		print("2_4# test MR[1]=SR[1] fail. MR[1]=%s"%MR[1])

	SR[1] = "abc123fgerjb234fdgdb45db"
	MR[1] = SR[1]
	if MR[1] == 123:
		print("2_5# test MR[1]=SR[1] <int> success")
	else:
		print("2_5# test MR[1]=SR[1] <int> fail. MR[1]=%s"%MR[1])
	SR[2] = "abc-123.456fgerjb234dsvb78dsdgs"
	MR[1] = SR[2]
	if MR[1] == -123:
		print("2_6# test MR[1]=SR[2] <float> success")
	else:
		print("2_6# test MR[1]=SR[2] <float> fail. MR[1]=%s"%MR[1])
	SR[3] = "fgffdg-2.384e+6fsbsd45675fgf"
	MR[1] = SR[3]
	if MR[1] == -2384000:
		print("2_7# test MR[1]=SR[3] <e> success")
	else:
		print("2_7# test MR[1]=SR[3] <e> fail. MR[1]=%s"%MR[1])
	MR[1] = "abc123fgerjb234fdgdb45db"
	if MR[1] == 123:
		print("2_8# test MR[1]=str <int> success")
	else:
		print("2_8# test MR[1]=str <int> fail. MR[1]=%s"%MR[1])
	MR[1] = "abc-123.456fgerjb234dsvb78dsdgs"
	if MR[1] == -123:
		print("2_9# test MR[1]=str <float> success")
	else:
		print("2_9# test MR[1]=str <float> fail. MR[1]=%s"%MR[1])
	MR[1] = "fgffdg-2.384e+6fsbsd45675fgf"
	if MR[1] == -2384000:
		print("2_10# test MR[1]=str <e> success")
	else:
		print("2_10# test MR[1]=str <e> fail. MR[1]=%s"%MR[1])

def test_set_R():
	R[1] = 99.9
	if R[1] == 99.9:
		print("3_1# test R[1]=float success")
	else:
		print("3_1# test R[1]=float fail. R[1]=%s"%R[1])
	R[1] = 100
	if R[1] == 100.0:
		print("3_2# test R[1]=int success")
	else:
		print("3_2# test R[1]=int fail. R[1]=%s"%R[1])
	R[1] = 1.00e-04
	R[2] = R[1]
	if R[2] == 0.0001:
		print("3_3# test R[2]=R[1] success")
	else:
		print("3_3# test R[2]=R[1] fail. R[2]=%s"%R[2])
	MR[1] = 100
	R[1] = MR[1]
	if R[1] == 100.0:
		print("3_4# test R[1]=MR[1] success")
	else:
		print("3_4# test R[1]=MR[1] fail. R[1]=%s"%R[1])
	SR[1] = "abc123fgerjb234fdgdb45db"
	R[1] = SR[1]
	if R[1] == 123.0:
		print("3_5# test R[1]=SR[1] <int> success")
	else:
		print("3_5# test R[1]=SR[1] <int> fail. R[1]=%s"%R[1])
	SR[2] = "abc-123.456fgerjb234dsvb78dsdgs"
	R[1] = SR[2]
	if R[1] == -123.456:
		print("3_6# test R[1]=SR[2] <float> success")
	else:
		print("3_6# test R[1]=SR[2] <float> fail. R[1]=%s"%R[1])
	SR[3] = "fgffdg-2.384e+6fsbsd45675fgf"
	R[1] = SR[3]
	if R[1] == -2384000.0:
		print("3_7# test R[1]=SR[3] <e> success")
	else:
		print("3_7# test R[1]=SR[3] <e> fail. R[1]=%s"%R[1])
	R[1] = "abc123fgerjb234fdgdb45db"
	if R[1] == 123.0:
		print("3_8# test R[1]=str <int> success")
	else:
		print("3_8# test R[1]=str <int> fail. R[1]=%s"%R[1])
	R[1] = "abc-123.456fgerjb234dsvb78dsdgs"
	if R[1] == -123.456:
		print("3_9# test R[1]=str <float> success")
	else:
		print("3_9# test R[1]=str <float> fail. R[1]=%s"%R[1])
	R[1] = "fgffdg-2.384e+6fsbsd45675fgf"
	if R[1] == -2384000.0:
		print("3_10# test R[1]=str <e> success")
	else:
		print("3_10# test R[1]=str <e> fail. R[1]=%s"%R[1])

def test_MR_R():
	R[1] = 99.99999
	MR[1] = R[1]
	if MR[1] == 99:
		print("4# test MR[1] = R[1] success")
	else:
		print("4# test MR[1] = R[1] fail MR[1]=%s"%MR[1])

def test_R_MR():
	MR[1] = 12345
	R[1] = MR[1] 
	if R[1] == 12345.0:
		print("5# test R[1] = MR[1] success")
	else:
		print("5# test R[1] = MR[1] fail MR[1]=%s"%R[1])

def test_SR_MR():
	MR[1] = 65535
	SR[1] = MR[1]
	if SR[1] == '65535': 
		print("6# test SR[1] = MR[1] success")
	else:
		print("6# test SR[1] = MR[1] fail  SR[1]=%s"%SR[1])

def test_SR_R():
	R[1] = 1234.5678
	SR[1] = R[1]
	if SR[1] == '1234.5678': 
		print("7# test SR[1] = R[1] success")
	else:
		print("7# test SR[1] = R[1] fail  SR[8]=%s"%SR[1])

def test_R_SR():
	SR[1] = '65535.987654321'
	R[1] = SR[1]
	if R[1]- 65535.987654321 < 0.00000001:
		print("8# test R[1] = SR[1] success")
	else:
		print("8# test R[1] = SR[1] fail R[1]=%s"%R[1])

def test_MR_SR_float():
	SR[1] = '65535.987654321'
	MR[1] = SR[1]
	if MR[1] == 65535:
		print("9# test MR[1] = (float)SR[1] success")
	else:
		print("9# test MR[1] = (float)SR[1] fail MR[1]=%s"%MR[1])
def test_MR_SR_int():
	SR[2] = "12345"
	MR[1] = SR[2]
	if MR[1] == 12345:
		print("10# test MR[1] = (int)SR[2] success")
	else:
		print("10# test MR[1] = (int)SR[2] fail MR[1]=%s"%MR[1])

def test_MR_add():
	MR[1]=1
	MR[2]=99
	MR[3]=MR[1]+MR[2]
	if MR[3] == 100:
		print("11# test MR __add__ test success")
	else:
		print("11# test MR __add__ test fail MR[3]=%d"%MR[3])
	MR[1]=1
	MR[1]+=2
	if MR[1] == 3:
		print("12# test MR __iadd__ int test success")
	else:
		print("12# test MR __iadd__ int test fail. MR[1]=%d"%MR[1])
	MR[1]=1
	MR[1]+=2.123
	if MR[1] == 3:
		print("13# test MR __iadd__ float test success")
	else:
		print("13# test MR __iadd__ float test fail MR[1]=%d"%MR[1])

def test_R_add():
	R[1]=1.1
	R[2]=99.2
	R[3]=R[1]+R[2]+1.2345
	if R[3] == 101.5345:
		print("14# test R __add__ test success")
	else:
		print("14# test R __add__ test fail")
	R[1]=1.123
	R[1]+=2
	if R[1] == 3.123:
		print("15# test R __iadd__ int test success")
	else:
		print("15# test R __iadd__ int test fail")
	R[1]=1.1234
	R[1]+=2.4321
	if R[1]-3.5555 < 0.00000001:
		print("16# test R __iadd__ float test success")
	else:
		print("16# test R __iadd__ float test fail.R[1]=%s"%R[1])
	
def test_SR_add():
	SR[1] = "qwe"
	SR[2] = "rty"
	SR[3] = SR[1]+SR[2]
	if SR[3] == "qwerty":
		print("17# test SR __add__ test success")
	else:
		print("17# test SR __add__ test fail. SR[3]=%s"%SR[3])
	SR[1] = "qwe"
	SR[1] += "rty"
	if SR[1] == "qwerty":
		print("18# test SR __iadd__ test success")
	else:
		print("18# test SR __iadd__ test fail.SR[1]=%s"%SR[1])

def test_R_sub():
	R[1] = 1.234
	R[2] = 0.123
	R[3] = R[1]-R[2]
	if R[3] == 1.111:
		print("19# test R __sub__ R[1]-R[2]  success")
	else:
		print("19# test R __sub__ R[1]-R[2]  fail. R[3]=%s"%R[3])
	R[1] = R[1] - 1.105
	if R[1] - 0.129 < 0.00000001:
		print("20# test R __sub__ R[1]-float  success")
	else:
		print("20# test R __sub__ R[1]-float  fail. R[1]=%s"%R[1])
	R[1] = 100.234	
	R[1] -= 1
	if R[1] == 99.234:
		print("21# test R __isub__ R[1] success")
	else:
		print("21# test R __isub__ R[1] fail. R[1]=%s"%R[1])

def test_MR_sub():
	MR[1] = 65535
	MR[2] = 53190
	MR[3] = MR[1]-MR[2]
	if MR[3] == 12345:
		print("22# test MR __sub__ MR[1]-MR[2]  success")
	else:
		print("22# test MR __sub__ MR[1]-MR[2]  fail. MR[3]=%s"%MR[3])
	MR[1] = 54321	
	MR[2] = MR[1] - 12345
	if MR[2] == 41976:
		print("23# test MR __sub__ MR[1]-int  success")
	else:
		print("23# test MR __sub__ MR[1]-int  fail. MR[2]=%s"%MR[2])
	MR[1] = 1000	
	MR[2] = MR[1] - 456.123 
	if MR[2] == 543:
		print("24# test MR __sub__ MR[1]-float  success")
	else:
		print("24# test MR __sub__ MR[1]-float  fail. MR[2]=%s"%MR[2])
	MR[1] = 1000	
	MR[1]-=1
	if MR[1] == 999:
		print("25# test MR __isub__ MR[1] success")
	else:
		print("25# test MR __isub__ MR[1] fail. MR[0]=%s"%MR[1])

def test_R_mul():
	R[1] = 1.234
	R[2] = 5.678
	R[3] = R[1]*R[2]
	if R[3] == 7.006652:
		print("26# test R __mul__ R[1]*R[2]  success")
	else:
		print("26# test R __mul__ R[1]*R[2]  fail. R[3]=%s"%R[3])
	R[1] = 5.4321	
	R[1] = R[1] * 1.2345
	if R[1] == 6.70592745:
		print("27# test R __mul__ R[1]*float  success")
	else:
		print("27# test R __mul__ R[1]*float  fail. R[1]=%s"%R[1])
	R[1] = 5.4321	
	R[2] = R[1] * 1234
	if R[2] == 6703.2114:
		print("28# test R __mul__ R[1]*int  success")
	else:
		print("28# test R __mul__ R[1]*int  fail. R[2]=%s"%R[2])
	R[1] = 5.4321
	MR[1] = 1234	
	R[2] = R[1] * MR[1]
	if R[2] == 6703.2114:
		print("29# test R __mul__ R[1]*MR[1]  success")
	else:
		print("29# test R __mul__ R[1]*MR[1]  fail. R[2]=%s"%R[2])

def test_MR_mul():
	MR[1] = 12
	MR[2] = 67
	MR[3] = MR[1]*MR[2]
	if MR[3] == 804:
		print("30# test MR __mul__ MR[1]*MR[2]  success")
	else:
		print("30# test MR __mul__ MR[1]*MR[2]  fail. MR[3]=%s"%MR[3])
	MR[1] = 543	
	MR[2] = MR[1] * 2167
	if MR[2] == 1176681:
		print("31# test MR __mul__ MR[1]*int  success")
	else:
		print("31# test MR __mul__ MR[1]*int  fail. MR[2]=%s"%MR[2])
	MR[1] = 45678	
	MR[2] = MR[1] * 456.123 #20834786.394
	if MR[2] == 20834786:
		print("32# test MR __mul__ MR[1]*float  success")
	else:
		print("32# test MR __mul__ MR[1]*float  fail. MR[2]=%s"%MR[2])
	MR[1] = 45678	
	R[1] = 456.123
	MR[2] = MR[1] * R[1] #20834786.394
	if MR[2] == 20834786:
		print("33# test MR __mul__ MR[1]*R[1]  success")
	else:
		print("33# test MR __mul__ MR[1]*R[1]  fail. MR[2]=%s"%MR[2])

def test_SR_mul():
	SR[1] = "abc"
	SR[2] = SR[1]*3
	if SR[2] == "abcabcabc":
		print("34# test SR __mul__ SR[1]*3  success")
	else:
		print("34# test SR __mul__ SR[1]*3  fail. SR[2]=%s"%SR[2])
	SR[1] = "xyz"
	MR[1] = 5
	SR[2] = SR[1]*MR[1]
	if SR[2] == "xyzxyzxyzxyzxyz":
		print("35# test SR __mul__ SR[1]*MR[1]  success")
	else:
		print("35# test SR __mul__ SR[1]*MR[1]  fail. SR[2]=%s"%SR[2])

def test_R_div():
	#------------------true-div----------------
	R[1] = 123.456789
	R[2] = R[1]/34567
	if R[2] == 0.0035715216536002545:
		print("36# test R __truediv__ R[1]/int success")
	else:
		print("36# test R __truediv__ R[1]/int fail.R[2]=%s"%R[2])
	
	R[1] = 123.456789
	R[2] = R[1]/567.123489
	if R[2] == 0.21768942989416545:
		print("37# test R __truediv__ R[1]/float success")
	else:
		print("37# test R __truediv__ R[1]/float fail.R[2]=%s"%R[2])
	
	R[1] = 123.456789
	R[2] = 567.123489
	R[3] = R[1]/R[2]
	if R[3]-0.21768942989416545 < 0.00000001:
		print("38# test R __truediv__ R[1]/R[2] success")
	else:
		print("38# test R __truediv__ R[1]/R[2] fail.R[0]=%s"%R[3])
	
	R[1] = 567.123489
	MR[1] = 34567
	R[2] = R[1]/MR[1]
	if R[2] == 0.01640650010125264:
		print("39# test R __truediv__ R[1]/MR[1] success")
	else:
		print("39# test R __truediv__ R[1]/MR[1] fail. R[2]=%s"%R[2])
	#----------------------floordiv
	R[1] = 987654.321
	R[2] = R[1]//255
	if R[2] == 3873.0:
		print("40# test R __floordiv__ R[1]//int success")
	else:
		print("40# test R __floordiv__ R[1]//int fail.R[2]=%s"%R[2])
	
	R[1] = 987654.321
	R[2] = R[1]//255.789
	if R[2] == 3861.0:
		print("41# test R __floordiv__ R[1]//float success")
	else:
		print("41# test R __floordiv__ R[1]//float fail.R[2]=%s"%R[2])
	
	R[1] = 987654.321
	R[2] = 255.789
	R[3] = R[1]//R[2]
	if R[3] ==3861.0:
		print("42# test R __floordiv__ R[1]//R[2] success")
	else:
		print("42# test R __floordiv__ R[1]//R[2] fail R[3]=%s"%R[3])
	
	R[1] = 987654.321
	MR[1] = 255
	R[2] = R[1]//MR[1]
	if R[2] == 3873.0:
		print("43# test R __floordiv__ R[1]//MR[1] success")
	else:
		print("43# test R __floordiv__ R[1]//MR[1] fail. R[2]=%s"%R[2])

def test_MR_div():
	#------------------true-div----------------
	MR[1] = 6789
	MR[2] = MR[1]/67
	if MR[2] == 101:
		print("44# test MR __truediv__ MR[1]/int success")
	else:
		print("44# test MR __truediv__ MR[1]/int fail.MR[2]=%s"%MR[2])
	
	MR[1] = 6789
	MR[2] = MR[1]/345.678 
	if MR[2] == 19:
		print("45# test MR __truediv__ MR[1]/float success")
	else:
		print("45# test MR __truediv__ MR[1]/float fail.MR[2]=%s"%MR[2])
	
	MR[1] = 6789
	MR[2] = 345
	MR[3] = MR[1]/MR[2]#19.678260869565218
	if MR[3] == 19:
		print("46# test MR __truediv__ MR[1]/MR[2] success")
	else:
		print("46# test MR __truediv__ MR[1]/MR[2] fail.MR[3]=%s"%MR[3])
	
	MR[1] = 65533
	R[1] = 67.123
	MR[2] = MR[1]/R[1] 
	if MR[2] - 976.3121433785736 < 0.00000001:
		print("47# test MR __truediv__ MR[1]/R[1] success")
	else:
		print("47# test MR __truediv__ MR[1]/R[1] fail. MR[2]=%s"%MR[2])
	#----------------------floordiv
	MR[1] = 1234
	MR[2] = MR[1]//3
	if MR[2] == 411:
		print("48# test MR __floordiv__ MR[1]//int success")
	else:
		print("48# test MR __floordiv__ MR[1]//int fail.MR[2]=%s"%MR[2])
	
	MR[1] = 9876
	MR[2] = MR[1]//3.14 
	if MR[2] == 3145:
		print("49# test MR __floordiv__ MR[1]//float success")
	else:
		print("49# test MR __floordiv__ MR[1]//float fail.MR[2]=%s"%MR[2])
	
	MR[1] = 9876
	MR[2] = 3
	MR[3] = MR[1]//MR[2]
	if MR[3] == 3292:
		print("50# test MR __floordiv__ MR[1]//MR[2] success")
	else:
		print("50# test MR __floordiv__ MR[1]//MR[2] fail.MR[3]=%s"%MR[3])
	
	MR[1] = 987
	R[1] = 67.1
	MR[2] = MR[1]//R[1] 
	if MR[2] == 14:
		print("51# test MR __floordiv__ MR[1]//R[1] success")
	else:
		print("51# test MR __floordiv__ MR[1]//R[1] fail. MR[2]=%s"%MR[2])

def test_R_mod():
	R[1] = 123.456
	R[2] = 7.89
	R[3] = R[1] % R[2]
	if R[3] == 5.106000000000008:
		print("52# test R __mod__ R[1] % R[2] success")
	else:
		print("52# test R __mod__ R[1] % R[2] fail.R[3]=%s"%R[3])
	R[1] = 123.456
	MR[1] = 7
	R[2] = R[1] % MR[1]
	if R[2] == 4.456000000000003:
		print("53# test R __mod__ R[1] % MR[1] success")
	else:
		print("53# test R __mod__ R[1] % MR[1] fail.R[2]=%s"%R[2])
	
	R[1] = 123.456
	R[2] = R[1] % 7.89
	if R[2] == 5.106000000000008:
		print("54# test R __mod__ R[1] % float success")
	else:
		print("54# test R __mod__ R[1] % float fail.R[2]=%s"%R[2])
	R[1] = 123.456
	R[2] = R[1] % 7
	if R[2] == 4.456000000000003:
		print("55# test R __mod__ R[1] % int success")
	else:
		print("55# test R __mod__ R[1] % int fail.R[2]=%s"%R[2])

def test_MR_mod():
	MR[1] = 123456
	R[1] = 7.89
	MR[2] = MR[1] % R[1] #1.170000000005003
	if MR[2] == 1:
		print("56# test MR __mod__ MR[1] % R[1] success")
	else:
		print("56# test MR __mod__ MR[1] % R[1] fail.MR[2]=%s"%MR[2])
	MR[1] = 123456
	MR[2] = 7
	MR[3] = MR[1] % MR[2]
	if MR[3] == 4:
		print("57# test MR __mod__ MR[1] % MR[2] success")
	else:
		print("57# test MR __mod__ MR[1] % MR[2] fail.MR[3]=%s"%MR[3])
	
	MR[1] = 123456
	MR[2] = MR[1] % 78.9012   #54.523199999995484
	if MR[2] == 54:
		print("58# test MR __mod__ MR[1] % float success")
	else:
		print("58# test MR __mod__ MR[1] % float fail.MR[2]=%s"%MR[2])
	MR[1] = 123456
	MR[2] = MR[1] % 7
	if MR[2] == 4:
		print("59# test MR __mod__ MR[1] % int success")
	else:
		print("59# test MR __mod__ MR[1] % int fail.MR[2]=%s"%MR[2])

def test_MR_pow():
	MR[1] = 12
	R[1] = 3.45
	MR[2] = MR[1] ** R[1] #5286.585967626344
	if MR[2] == 5286:
		print("60# test MR __pow__ MR[1]**R[1] success")
	else:
		print("60# test MR __pow__ MR[1]**R[1] fail.MR[2]=%s"%MR[2])
	MR[1] = 12
	MR[2] = 3
	MR[3] = MR[1] ** MR[2]
	if MR[3] == 1728:
		print("61# test MR __pow__ MR[1]**MR[2] success")
	else:
		print("61# test MR __pow__ MR[1]**MR[2] fail.MR[3]=%s"%MR[3])
	
	MR[1] = 12
	MR[2] = MR[1] ** 3.45  
	if MR[2] == 5286:
		print("62# test MR __pow__ MR[1]**float success")
	else:
		print("62# test MR __pow__ MR[1]**float fail.MR[2]=%s"%MR[2])
	MR[1] = 12
	MR[2] = MR[1] ** 3
	if MR[2] == 1728:
		print("63# test MR __pow__ MR[1]**int success")
	else:
		print("63# test MR __pow__ MR[1]**int fail.MR[2]=%s"%MR[2])

def test_R_pow():
	R[1] = 2.234
	R[2] = 8.98
	R[3] = R[1] ** R[2] #1363.850777569892
	if R[3] == 1363.850777569892:
		print("64# test R __pow__ R[1]**R[2] success")
	else:
		print("64# test R __pow__ R[1]**R[2] fail.R[3]=%s"%R[3])
	R[1] = 12.34567
	MR[1] = 3
	R[2] = R[1] ** MR[1]
	if R[2] == 1881.6723022905624:
		print("65# test R __pow__ R[1]**MR[1] success")
	else:
		print("65# test R __pow__ R[1]**MR[1] fail.R[2]=%s"%R[2])
	
	R[1] = 2.234
	R[2] = R[1] ** 8.98
	if R[2] == 1363.850777569892:
		print("66# test R __pow__ R[1]**float success")
	else:
		print("66# test R __pow__ R[1]**float fail.R[2]=%s"%R[2])
	R[1] = 12.34567
	R[2] = R[1] ** 3
	if R[2] == 1881.6723022905624:
		print("67# test R __pow__ R[1]**int success")
	else:
		print("67# test R __pow__ R[1]**int fail.R[2]=%s"%R[2])

#小于
def test_R_lt():
	R[1] = 12.34567
	R[2] = 88.901234
	if R[1] < R[2]:
		print("68# test R __lt__ R[1] < R[2] success")
	else:
		print("68# test R __lt__ R[1] < R[2] fail")
	R[1] = 99.9	
	MR[1] = 100
	if R[1] < MR[1]:
		print("69# test R __lt__ R[1] < MR[1] success")
	else:
		print("69# test R __lt__ R[1] < MR[1] fail")
	R[1] = 99.8	
	if R[1] < 99.9:
		print("70# test R __lt__ R[1] < float success")
	else:
		print("70# test R __lt__ R[1] < float fail")
	R[1] = 99.9	
	if R[1] < 100:
		print("71# test R __lt__ R[1] < int success")
	else:
		print("71# test R __lt__ R[1] < int fail")

def test_R_le():
	R[1] = 12.34567
	R[2] = 88.901234
	if R[1] <= R[2]:
		print("72# test R __le__ R[1] <= R[2] success")
	else:
		print("72# test R __le__ R[1] <= R[2] fail")
	R[1] = 99.9	
	MR[1] = 100
	if R[1] <= MR[1]:
		print("73# test R __le__ R[1] <= MR[1] success")
	else:
		print("73# test R __le__ R[1] <= MR[1] fail")
	R[1] = 99.8	
	if R[1] <= 99.9:
		print("74# test R __le__ R[1] <= float success")
	else:
		print("74# test R __le__ R[1] <= float fail")
	R[1] = 99.9	
	if R[1] <= 100:
		print("75# test R __le__ R[1] <= int success")
	else:
		print("75# test R __le__ R[1] <= int fail")

def test_MR_lt():
	MR[1] = 99
	MR[2] = 100
	if MR[1] < MR[2]:
		print("76# test MR __lt__ MR[1] < MR[2] success")
	else:
		print("76# test MR __lt__ MR[1] < MR[2] fail")
	MR[1] = 99	
	R[1] = 99.1
	if MR[1] < R[1]:
		print("77# test MR __lt__ MR[1] < R[1] success")
	else:
		print("77# test MR __lt__ MR[1] < R[1] fail")
	MR[1] = 99	
	if MR[1] < 99.1:
		print("78# test MR __lt__ MR[1] < float success")
	else:
		print("78# test MR __lt__ MR[1] < float fail")
	MR[1] = 99	
	if MR[1] < 100:
		print("79# test MR __lt__ MR[1] < int success")
	else:
		print("79# test MR __lt__ MR[1] < int fail")

def test_MR_le():
	MR[1] = 99
	MR[2] = 100
	if MR[1] <= MR[2]:
		print("80# test MR __le__ MR[1] <= MR[2] success")
	else:
		print("80# test MR __le__ MR[1] <= MR[2] fail")
	MR[1] = 99	
	R[1] = 99.1
	if MR[1] <= R[1]:
		print("81# test MR __le__ MR[1] <= R[1] success")
	else:
		print("81# test MR __le__ MR[1] <= R[1] fail")
	MR[1] = 99	
	if MR[1] <= 99.1:
		print("82# test MR __le__ MR[1] <= float success")
	else:
		print("82# test MR __le__ MR[1] <= float fail")
	MR[1] = 99
	if MR[1] <= 100:
		print("83# test MR __le__ MR[1] <= int success")
	else:
		print("83# test MR __le__ MR[1] <= int fail")

def test_R_gt():
	R[1] = 99.9999
	R[2] = 99.9998
	if R[1] > R[2]:
		print("84# test R __gt__ R[1] > R[2] success")
	else:
		print("84# test R __gt__ R[1] > R[2] fail")
	R[1] = 99.9	
	MR[1] = 99
	if R[1] > MR[1]:
		print("85# test R __gt__ R[1] > MR[1] success")
	else:
		print("85# test R __gt__ R[1] > MR[1] fail")
	R[1] = 99.9	
	if R[1] > 99.8:
		print("86# test R __gt__ R[1] > float success")
	else:
		print("86# test R __gt__ R[1] > float fail")
	R[1] = 99.9	
	if R[1] > 99:
		print("87# test R __gt__ R[1] > int success")
	else:
		print("87# test R __gt__ R[1] > int fail")

def test_R_ge():
	R[1] = 99.9999
	R[2] = 99.9998
	if R[1] >= R[2]:
		print("88# test R __ge__ R[1] >= R[2] success")
	else:
		print("88# test R __ge__ R[1] >= R[2] fail")
	R[1] = 99.9	
	MR[1] = 99
	if R[1] >= MR[1]:
		print("89# test R __ge__ R[1] >= MR[1] success")
	else:
		print("89# test R __ge__ R[1] >= MR[1] fail")
	R[1] = 99.9	
	if R[1] >= 99.8:
		print("90# test R __ge__ R[1] >= float success")
	else:
		print("90# test R __ge__ R[1] >= float fail")
	R[1] = 99.9	
	if R[1] >= 99:
		print("91# test R __ge__ R[1] >= int success")
	else:
		print("91# test R __ge__ R[1] >= int fail")

def test_MR_gt():
	MR[1] = 100
	MR[2] = 99
	if MR[1] > MR[2]:
		print("92# test MR __gt__ MR[1] > MR[2] success")
	else:
		print("92# test MR __gt__ MR[1] > MR[2] fail")
	MR[1] = 100	
	R[1] = 99.9
	if MR[1] > R[1]:
		print("93# test MR __gt__ MR[1] > R[1] success")
	else:
		print("93# test MR __gt__ MR[1] > R[1] fail")
	MR[1] = 100	
	if MR[1] > 99.9:
		print("94# test MR __gt__ MR[1] > float success")
	else:
		print("94# test MR __gt__ MR[1] > float fail")
	MR[1] = 100	
	if MR[1] > 99:
		print("95# test MR __gt__ MR[1] > int success")
	else:
		print("95# test MR __gt__ MR[1] > int fail")

def test_MR_ge():
	MR[1] = 100
	MR[2] = 99
	if MR[1] >= MR[2]:
		print("96# test MR __gt__ MR[1] >= MR[2] success")
	else:
		print("96# test MR __gt__ MR[1] >= MR[2] fail")
	MR[1] = 100	
	R[1] = 99.9
	if MR[1] >= R[1]:
		print("97# test MR __gt__ MR[1] >= R[1] success")
	else:
		print("97# test MR __gt__ MR[1] >= R[1] fail")
	MR[1] = 100	
	if MR[1] >= 99.9:
		print("98# test MR __gt__ MR[1] >= float success")
	else:
		print("98# test MR __gt__ MR[1] >= float fail")
	MR[1] = 100	
	if MR[1] >= 99:
		print("99# test MR __gt__ MR[1] >= int success")
	else:
		print("99# test MR __gt__ MR[1] >= int fail")


def test_R_ne():
	R[1] = 99.9999
	R[2] = 99.9998
	if R[1] != R[2]:
		print("100# test R __ne__ R[1] != R[2] success")
	else:
		print("100# test R __ne__ R[1] != R[2] fail")
	R[1] = 99.9	
	MR[1] = 99
	if R[1] != MR[1]:
		print("101# test R __ne__ R[1] != MR[1] success")
	else:
		print("101# test R __ne__ R[1] != MR[1] fail")
	R[1] = 99.9	
	if R[1] != 99.8:
		print("102# test R __ne__ R[1] != float success")
	else:
		print("102# test R __ne__ R[1] != float fail")
	R[1] = 99.9	
	if R[1] != 99:
		print("103# test R __ne__ R[1] != int success")
	else:
		print("103# test R __ne__ R[1] != int fail")
def test_MR_ne():
	MR[1] = 99
	MR[2] = 88
	if MR[1] != MR[2]:
		print("104# test MR __ne__ MR[1] != MR[2] success")
	else:
		print("104# test MR __ne__ MR[1] != MR[2] fail")
	MR[1] = 100	
	R[1] = 99.9
	if MR[1] != R[1]:
		print("105# test MR __ne__ MR[1] != R[1] success")
	else:
		print("105# test MR __ne__ MR[1] != R[1] fail")
	MR[1] = 100	
	if MR[1] != 99.9:
		print("106# test MR __ne__ MR[1] != float success")
	else:
		print("106# test MR __ne__ MR[1] != float fail")
	MR[1] = 100	
	if MR[1] != 99:
		print("107# test MR __ne__ MR[1] != int success")
	else:
		print("107# test MR __ne__ MR[1] != int fail")

def test_cascad_access(): #寄存器间级联访问测试
	R[1]  = 123.456
	R[2] = 1.0
	if R[R[2]] == 123.456:
		print("108# test R[R[]] success") 
	else:
		print("108# test R[R[]] fail.")
	MR[1] = 2
	R[2] = 234.567
	if R[MR[1]] == 234.567:
		print("109# test R[MR[]] success") 
	else:
		print("109# test R[MR[]] fail.")
	SR[1] = "abc2def3fggh"
	if R[SR[1]] == 234.567:
		print("110# test R[SR[]] success") 
	else:
		print("110# test R[SR[]] fail.")
	MR[3] = 23456
	R[1] = 3.0
	if MR[R[1]] == 23456:
		print("111# test MR[R[]] success") 
	else:
		print("111# test MR[R[]] fail.")
	MR[1] = 3
	if MR[MR[1]] == 23456:
		print("112# test MR[MR[]] success") 
	else:
		print("112# test MR[MR[]] fail.")
	SR[1] = "qwe3sd4"
	if MR[SR[1]] == 23456:
		print("113# test MR[SR[]] success") 
	else:
		print("113# test MR[SR[]] fail.")
	SR[3] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890"
	R[1] = 3.0
	if SR[R[1]] == "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890":
		print("114# test SR[R[]] success") 
	else:
		print("114# test SR[R[]] fail.")
	MR[1] = 3
	if SR[MR[1]] == "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890":
		print("115# test SR[MR[]] success") 
	else:
		print("115# test SR[MR[]] fail.")
	SR[1] = "sdfahfja3.0adfa"
	if SR[SR[1]] == "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890":
		print("116# test SR[SR[]] success") 
	else:
		print("116# test SR[SR[]] fail.")

	SR[3] = "stringIndex"
	if SR["fhsdjfsbd3.0wgeb"] == "stringIndex":
		print("117# test SR[string] success") 
	else:
		print("117# test SR[string] fail.")
	SR[3] = "qwertyuiopasdfghjklzxcvbnm"
	MR[2] = 3
	R[2] = 2.0
	SR[1] = "abc2.0sjfa88"
	if SR[MR[R[SR[1]]]] == "qwertyuiopasdfghjklzxcvbnm":
		print("118# test SR[MR[R[SR[]]]] success") 
	else:
		print("118# test SR[MR[R[SR[]]]] fail.")
def test_cascad_asignment(): #级联赋值测试
	MR[1] = 2
	R[MR[1]] = 99.99
	if R[2] == 99.99:
		print("119# test set R[MR[]] succes");
	else:
		print("119# test set R[MR[]] fail");
	R[1] = 2.34
	R[R[1]] = 123.456
	if R[2] == 123.456:
		print("120# test set R[R[]] succes");
	else:
		print("120# test set R[R[]] fail");
	SR[1] = "sdfhsebf2.34sg"
	R[SR[1]] = 423.456
	if R[2] == 423.456:
		print("121# test set R[SR[]] succes");
	else:
		print("121# test set R[SR[]] fail");

	MR[1] = 2
	MR[MR[1]] = 999
	if MR[2] == 999:
		print("122# test set MR[MR[]] succes");
	else:
		print("122# test set MR[MR[]] fail");
	R[1] = 2.34
	MR[R[1]] = 789
	if MR[2] == 789:
		print("123# test set MR[R[]] succes");
	else:
		print("123# test set MR[R[]] fail");
	SR[1] = "sdfhsebf2.34sg"
	MR[SR[1]] = 423
	if MR[2] == 423:
		print("124# test set MR[SR[]] succes");
	else:
		print("124# test set MR[SR[]] fail");

	MR[1] = 2
	SR[MR[1]] = "hello world"
	if SR[2] == "hello world":
		print("125# test set SR[MR[]] succes");
	else:
		print("125# test set SR[MR[]] fail");
	MR[1] = 2
	SR[MR["abc1.23fdsbgs"]] = "qazwsx"
	if SR[2] == "qazwsx":
		print("126# test set SR[MR[str]] succes");
	else:
		print("126# test set SR[MR[str]] fail");
	MR[1] = 2
	SR[3] = "abc1.23fdsbgs"
	SR[MR[SR[3]]] = "zxcvbnm"
	if SR[2] == "zxcvbnm":
		print("127# test set SR[MR[SR[]]] succes");
	else:
		print("127# test set SR[MR[SR[]]] fail");

	MR[1] = 2
	SR[3] = "abc1.23fdsbgs"
	R[MR[SR[3]]] = 123.789
	if R[2] == 123.789:
		print("128# test set R[MR[SR[]]] succes");
	else:
		print("128# test set R[MR[SR[]]] fail");

def testPR():
    pr = Posture(1,2,3,4,5,1,2,3,4,5,6,7,8,9,1,2,3,4,5,6,7,8,9)
    reg.SetPR(1,pr)
    d = reg.GetPR(1)
    print(d)
def test_set_get_PR():
	pr = Posture(45,2,3,4,5,1,2,3,4,5,6,7,8,9,1,2,3,4,5,6,7,8,9)
	reg.SetPR(1,pr)
	print("129# test set PR[1]=%s"%PR[1])
	PR[2] = PR[1]
	print("129# test PR[2]=PR[1]  PR[2]=%s"%PR[2])
	MR[1] = 2
	R[1] = 2.0
	SR[1] = "abc2.3sahfaj"
	print("130# test  PR[MR[1]]=%s"%PR[MR[1]])
	print("131# test  PR[R[1]]=%s"%PR[R[1]])
	print("132# test  PR[SR[1]]=%s"%PR[SR[1]])
	print("133# test  PR[float]=%s"%PR[2.34])
	print("134# test  PR[string]=%s"%PR["qwer2.23"])


print("======register basic calculation test log======")

test_set_SR()
test_set_MR()
test_set_R()
test_MR_R()
test_R_MR()
test_SR_MR()
test_SR_R()
test_R_SR()
test_MR_SR_float()
test_MR_SR_int()
test_MR_add()
test_R_add()
test_SR_add()
test_R_sub()
test_MR_sub()
test_R_mul()
test_MR_mul()
test_SR_mul()
test_R_div()
test_MR_div()
test_R_mod()
test_MR_mod()
test_MR_pow()
test_R_pow()
test_R_lt()
test_R_le()
test_MR_lt()
test_MR_le()
test_R_gt()
test_R_ge()
test_MR_gt()
test_MR_ge()
test_R_ne()
test_MR_ne()
test_cascad_access()
test_cascad_asignment()
#testPR()
test_set_get_PR()

