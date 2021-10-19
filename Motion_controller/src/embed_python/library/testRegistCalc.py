"""
文档说明:  本文档编写了一些对寄存器赋值及基本运算的测试代码
肖志才测试通过
2021/10/19 
"""

from registers import *

def test_set_SR():
	SR[0] = 'abcdefg'
	SR[1] = SR[0]
	if SR[1] == 'abcdefg':
		print("1# test SR[1] = SR[0] success")
	else:
		print("1# test SR[1]=SR[0] fail. SR[1]=%s"%SR[1])

def test_set_MR():
	MR[0] = 99.9
	if MR[0] == 99:
		print("2_1# test MR[0]=float success")
	else:
		print("2_1# test MR[0]=float fail. MR[0]=%s"%MR[0])
	MR[0] = 100
	if MR[0] == 100:
		print("2_2# test MR[0]=int success")
	else:
		print("2_2# test MR[0]=int fail. MR[0]=%s"%MR[0])
	R[0] = 99.9
	MR[0] = R[0]
	if MR[0] == 99:
		print("2_3# test MR[0] = R[0] success")
	else:
		print("2_3# test MR[0] = R[0] fail. MR[0]=%s"%MR[0])
	MR[1] = 65535
	MR[0] = MR[1]
	if MR[0] == 65535:
		print("2_4# test MR[0] = MR[1] success")
	else:
		print("2_4# test MR[0] = MR[1] fail. MR[1]=%s"%MR[0])
	SR[0] = "abc123fgerjb234fdgdb45db"
	MR[0] = SR[0]
	if MR[0] == 123:
		print("2_4# test MR[0]=SR[0] success")
	else:
		print("2_4# test MR[0]=SR[0] fail. MR[1]=%s"%MR[0])

	SR[0] = "abc123fgerjb234fdgdb45db"
	MR[1] = SR[0]
	if MR[1] == 123:
		print("2_5# test MR[1]=SR[0] <int> success")
	else:
		print("2_5# test MR[1]=SR[0] <int> fail. MR[1]=%s"%MR[1])
	SR[0] = "abc-123.456fgerjb234dsvb78dsdgs"
	MR[1] = SR[0]
	if MR[1] == -123:
		print("2_6# test MR[1]=SR[0] <float> success")
	else:
		print("2_6# test MR[1]=SR[0] <float> fail. MR[1]=%s"%MR[1])
	SR[0] = "fgffdg-2.384e+6fsbsd45675fgf"
	MR[1] = SR[0]
	if MR[1] == -2384000:
		print("2_7# test MR[1]=SR[0] <e> success")
	else:
		print("2_7# test MR[1]=SR[0] <e> fail. MR[1]=%s"%MR[1])
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
	R[0] = 99.9
	if R[0] == 99.9:
		print("3_1# test R[0]=float success")
	else:
		print("3_1# test R[0]=float fail. R[0]=%s"%R[0])
	R[0] = 100
	if R[0] == 100.0:
		print("3_2# test R[0]=int success")
	else:
		print("3_2# test R[0]=int fail. R[0]=%s"%R[0])
	R[0] = 1.00e-04
	R[1] = R[0]
	if R[1] == 0.0001:
		print("3_3# test R[1]=R[0] success")
	else:
		print("3_3# test R[1]=R[0] fail. R[1]=%s"%R[1])
	MR[0] = 100
	R[1] = MR[0]
	if R[1] == 100.0:
		print("3_4# test R[1]=MR[0] success")
	else:
		print("3_4# test R[1]=MR[0] fail. R[1]=%s"%R[1])
	SR[0] = "abc123fgerjb234fdgdb45db"
	R[1] = SR[0]
	if R[1] == 123.0:
		print("3_5# test R[1]=SR[0] <int> success")
	else:
		print("3_5# test R[1]=SR[0] <int> fail. R[1]=%s"%R[1])
	SR[0] = "abc-123.456fgerjb234dsvb78dsdgs"
	R[1] = SR[0]
	if R[1] == -123.456:
		print("3_6# test R[1]=SR[0] <float> success")
	else:
		print("3_6# test R[1]=SR[0] <float> fail. R[1]=%s"%R[1])
	SR[0] = "fgffdg-2.384e+6fsbsd45675fgf"
	R[1] = SR[0]
	if R[1] == -2384000.0:
		print("3_7# test R[1]=SR[0] <e> success")
	else:
		print("3_7# test R[1]=SR[0] <e> fail. R[1]=%s"%R[1])
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
	R[0] = 99.99999
	MR[0] = R[0]
	if MR[0] == 99:
		print("4# test MR[0] = R[0] success")
	else:
		print("4# test MR[0] = R[0] fail MR[0]=%s"%MR[0])

def test_R_MR():
	MR[0] = 12345
	R[0] = MR[0] 
	if R[0] == 12345.0:
		print("5# test R[0] = MR[0] success")
	else:
		print("5# test R[0] = MR[0] fail MR[0]=%s"%R[0])

def test_SR_MR():
	MR[0] = 65535
	SR[0] = MR[0]
	if SR[0] == '65535': 
		print("6# test SR[0] = MR[0] success")
	else:
		print("6# test SR[0] = MR[0] fail  SR[0]=%s"%SR[0])

def test_SR_R():
	R[0] = 1234.5678
	SR[0] = R[0]
	if SR[0] == '1234.5678': 
		print("7# test SR[0] = R[0] success")
	else:
		print("7# test SR[0] = R[0] fail  SR[0]=%s"%SR[0])

def test_R_SR():
	SR[0] = '65535.987654321'
	R[0] = SR[0]
	if R[0]- 65535.987654321 < 0.00000001:
		print("8# test R[0] = SR[0] success")
	else:
		print("8# test R[0] = SR[0] fail R[0]=%s"%R[0])

def test_MR_SR_float():
	SR[0] = '65535.987654321'
	MR[0] = SR[0]
	if MR[0] == 65535:
		print("9# test MR[0] = (float)SR[0] success")
	else:
		print("9# test MR[0] = (float)SR[0] fail MR[0]=%s"%R[0])
def test_MR_SR_int():
	SR[0] = '65535'
	MR[0] = SR[0]
	if MR[0] == 65535:
		print("10# test MR[0] = (int)SR[0] success")
	else:
		print("10# test MR[0] = (int)SR[0] fail MR[0]=%s"%R[0])

def test_MR_add():
	MR[0]=1
	MR[1]=99
	MR[0]=MR[0]+MR[1]+1
	if MR[0] == 101:
		print("11# test MR __add__ test success")
	else:
		print("11# test MR __add__ test fail")
	MR[0]=1
	MR[0]+=2
	if MR[0] == 3:
		print("12# test MR __iadd__ int test success")
	else:
		print("12# test MR __iadd__ int test fail")
	MR[0]=1
	MR[0]+=2.123
	if MR[0] == 3:
		print("13# test MR __iadd__ float test success")
	else:
		print("13# test MR __iadd__ float test fail")

def test_R_add():
	R[0]=1.1
	R[1]=99.2
	R[0]=R[0]+R[1]+1.2345
	if R[0] == 101.5345:
		print("14# test R __add__ test success")
	else:
		print("14# test R __add__ test fail")
	R[0]=1.123
	R[0]+=2
	if R[0] == 3.123:
		print("15# test R __iadd__ int test success")
	else:
		print("15# test R __iadd__ int test fail")
	R[0]=1.1234
	R[0]+=2.4321
	if R[0]-3.5555 < 0.00000001:
		print("16# test R __iadd__ float test success")
	else:
		print("16# test R __iadd__ float test fail.R[0]=%s"%R[0])
	
def test_SR_add():
	SR[0] = "qwe"
	SR[1] = "rty"
	SR[0] = SR[0]+SR[1]
	if SR[0] == "qwerty":
		print("17# test SR __add__ test success")
	else:
		print("17# test SR __add__ test fail. SR[0]=%s"%SR[0])
	SR[0] = "qwe"
	SR[0] += "rty"
	if SR[0] == "qwerty":
		print("18# test SR __iadd__ test success")
	else:
		print("18# test SR __iadd__ test fail.SR[0]=====================>%s"%SR[0])

def test_R_sub():
	R[0] = 1.234
	R[1] = 0.123
	R[1] = R[0]-R[1]
	if R[1] == 1.111:
		print("19# test R __sub__ R[0]-R[1]  success")
	else:
		print("19# test R __sub__ R[0]-R[1]  fail. R[1]=%s"%R[1])
	R[0] = R[0] - 1.105
	if R[0] - 0.129 < 0.00000001:
		print("20# test R __sub__ R[0]-float  success")
	else:
		print("20# test R __sub__ R[0]-float  fail. R[0]=%s"%R[0])
	R[0] = 100.234	
	R[0] -= 1
	if R[0] == 99.234:
		print("21# test R __isub__ R[0] success")
	else:
		print("21# test R __isub__ R[0] fail. R[0]=%s"%R[0])

def test_MR_sub():
	MR[0] = 65535
	MR[1] = 53190
	MR[1] = MR[0]-MR[1]
	if MR[1] == 12345:
		print("22# test MR __sub__ MR[0]-MR[1]  success")
	else:
		print("22# test MR __sub__ MR[0]-MR[1]  fail. MR[1]=%s"%MR[1])
	MR[0] = 54321	
	MR[0] = MR[0] - 12345
	if MR[0] == 41976:
		print("23# test MR __sub__ MR[0]-int  success")
	else:
		print("23# test MR __sub__ MR[0]-int  fail. MR[0]=%s"%MR[0])
	MR[0] = 1000	
	MR[0] = MR[0] - 456.123 
	if MR[0] == 543:
		print("24# test MR __sub__ MR[0]-float  success")
	else:
		print("24# test MR __sub__ MR[0]-float  fail. MR[0]=%s"%MR[0])
	MR[0] = 1000	
	MR[0]-=1
	if MR[0] == 999:
		print("25# test MR __isub__ MR[0] success")
	else:
		print("25# test MR __isub__ MR[0] fail. MR[0]=%s"%MR[0])

def test_R_mul():
	R[0] = 1.234
	R[1] = 5.678
	R[1] = R[0]*R[1]
	if R[1] == 7.006652:
		print("26# test R __mul__ R[0]*R[1]  success")
	else:
		print("26# test R __mul__ R[0]*R[1]  fail. R[1]=%s"%R[1])
	R[0] = 5.4321	
	R[0] = R[0] * 1.2345
	if R[0] == 6.70592745:
		print("27# test R __mul__ R[0]*float  success")
	else:
		print("27# test R __mul__ R[0]*float  fail. R[0]=%s"%R[0])
	R[0] = 5.4321	
	R[0] = R[0] * 1234
	if R[0] == 6703.2114:
		print("28# test R __mul__ R[0]*int  success")
	else:
		print("28# test R __mul__ R[0]*int  fail. R[0]=%s"%R[0])
	R[0] = 5.4321
	MR[0] = 1234	
	R[0] = R[0] * MR[0]
	if R[0] == 6703.2114:
		print("29# test R __mul__ R[0]*MR[0]  success")
	else:
		print("29# test R __mul__ R[0]*MR[0]  fail. R[0]=%s"%R[0])

def test_MR_mul():
	MR[0] = 12345
	MR[1] = 67890
	MR[1] = MR[0]*MR[1]
	if MR[1] == 838102050:
		print("30# test MR __mul__ MR[0]*MR[1]  success")
	else:
		print("30# test MR __mul__ MR[0]*MR[1]  fail. MR[1]=%s"%MR[1])
	MR[0] = 54321	
	MR[0] = MR[0] * 12345
	if MR[0] == 670592745:
		print("31# test MR __mul__ MR[0]*int  success")
	else:
		print("31# test MR __mul__ MR[0]*int  fail. MR[0]=%s"%MR[0])
	MR[0] = 45678	
	MR[0] = MR[0] * 456.123 #20834786.394
	if MR[0] == 20834786:
		print("32# test MR __mul__ MR[0]*float  success")
	else:
		print("32# test MR __mul__ MR[0]*float  fail. MR[0]=%s"%MR[0])
	MR[0] = 45678	
	R[0] = 456.123
	MR[0] = MR[0] * R[0] #20834786.394
	if MR[0] == 20834786:
		print("33# test MR __mul__ MR[0]*R[0]  success")
	else:
		print("33# test MR __mul__ MR[0]*R[0]  fail. MR[0]=%s"%MR[0])

def test_SR_mul():
	SR[1] = "abc"
	SR[1] = SR[1]*3
	if SR[1] == "abcabcabc":
		print("34# test SR __mul__ SR[1]*3  success")
	else:
		print("34# test SR __mul__ SR[1]*3  fail. SR[1]=%s"%SR[1])
	SR[1] = "xyz"
	MR[1] = 5
	SR[1] = SR[1]*MR[1]
	if SR[1] == "xyzxyzxyzxyzxyz":
		print("35# test SR __mul__ SR[1]*MR[1]  success")
	else:
		print("35# test SR __mul__ SR[1]*MR[1]  fail. SR[1]=%s"%SR[1])

def test_R_div():
	#------------------true-div----------------
	R[0] = 123.456789
	R[0] = R[0]/34567
	if R[0] == 0.0035715216536002545:
		print("36# test R __truediv__ R[0]/int success")
	else:
		print("36# test R __truediv__ R[0]/int false.R[0]=%s"%R[0])
	
	R[0] = 123.456789
	R[0] = R[0]/567.123489
	if R[0] == 0.21768942989416545:
		print("37# test R __truediv__ R[0]/float success")
	else:
		print("37# test R __truediv__ R[0]/float false.R[0]=%s"%R[0])
	
	R[0] = 123.456789
	R[1] = 567.123489
	R[0] = R[0]/R[1]
	if R[0] == 0.21768942989416545:
		print("38# test R __truediv__ R[0]/R[1] success")
	else:
		print("38# test R __truediv__ R[0]/R[1] false.R[0]=%s"%R[0])
	
	R[0] = 567.123489
	MR[0] = 34567
	R[0] = R[0]/MR[0]
	if R[0] == 0.01640650010125264:
		print("39# test R __truediv__ R[0]/MR[0] success")
	else:
		print("39# test R __truediv__ R[0]/MR[0] fail. R[0]=%s"%R[0])
	#----------------------floordiv
	R[0] = 987654.321
	R[0] = R[0]//255
	if R[0] == 3873.0:
		print("40# test R __floordiv__ R[0]//int success")
	else:
		print("40# test R __floordiv__ R[0]//int false.R[0]=%s"%R[0])
	
	R[0] = 987654.321
	R[0] = R[0]//255.789
	if R[0] == 3861.0:
		print("41# test R __floordiv__ R[0]//float success")
	else:
		print("41# test R __floordiv__ R[0]//float false.R[0]=%s"%R[0])
	
	R[0] = 987654.321
	R[1] = 255.789
	R[0] = R[0]//R[1]
	if R[0] ==3861.0:
		print("42# test R __floordiv__ R[0]//R[1] success")
	else:
		print("42# test R __floordiv__ R[0]//R[1] false.R[0]=%s"%R[0])
	
	R[0] = 987654.321
	MR[0] = 255
	R[0] = R[0]//MR[0]
	if R[0] == 3873.0:
		print("43# test R __floordiv__ R[0]//MR[0] success")
	else:
		print("43# test R __floordiv__ R[0]//MR[0] fail. R[0]=%s"%R[0])

def test_MR_div():
	#------------------true-div----------------
	MR[0] = 123456789
	MR[0] = MR[0]/34567
	if MR[0] == 3571:
		print("44# test MR __truediv__ MR[0]/int success")
	else:
		print("44# test MR __truediv__ MR[0]/int false.MR[0]=%s"%MR[0])
	
	MR[0] = 123456789
	MR[0] = MR[0]/345.678 #357143.899814278
	if MR[0] == 357143:
		print("45# test MR __truediv__ MR[0]/float success")
	else:
		print("45# test MR __truediv__ MR[0]/float false.MR[0]=%s"%MR[0])
	
	MR[0] = 123456789
	MR[1] = 34567
	MR[0] = MR[0]/MR[1]
	if MR[0] == 3571:
		print("46# test MR __truediv__ MR[0]/MR[1] success")
	else:
		print("46# test MR __truediv__ MR[0]/MR[1] false.MR[0]=%s"%MR[0])
	
	MR[0] = 987654321
	R[0] = 67.1234589
	MR[0] = MR[0]/R[0]  #14713996.22703293
	if MR[0] == 14713996:
		print("47# test MR __truediv__ MR[0]/R[0] success")
	else:
		print("47# test MR __truediv__ MR[0]/R[0] fail. MR[0]=%s"%MR[0])
	#----------------------floordiv
	MR[0] = 123456789
	MR[0] = MR[0]//34567
	if MR[0] == 3571:
		print("48# test MR __floordiv__ MR[0]//int success")
	else:
		print("48# test MR __floordiv__ MR[0]//int false.MR[0]=%s"%MR[0])
	
	MR[0] = 123456789
	MR[0] = MR[0]//345.678 #357143.899814278
	if MR[0] == 357143:
		print("49# test MR __floordiv__ MR[0]//float success")
	else:
		print("49# test MR __floordiv__ MR[0]//float false.MR[0]=%s"%MR[0])
	
	MR[0] = 123456789
	MR[1] = 34567
	MR[0] = MR[0]//MR[1]
	if MR[0] == 3571:
		print("50# test MR __floordiv__ MR[0]//MR[1] success")
	else:
		print("50# test MR __floordiv__ MR[0]//MR[1] false.MR[0]=%s"%MR[0])
	
	MR[0] = 987654321
	R[0] = 67.1234589
	MR[0] = MR[0]//R[0]  #14713996.22703293
	if MR[0] == 14713996:
		print("51# test MR __floordiv__ MR[0]//R[0] success")
	else:
		print("51# test MR __floordiv__ MR[0]//R[0] fail. MR[0]=%s"%MR[0])

def test_R_mod():
	R[0] = 123.456
	R[1] = 7.89
	R[0] = R[0] % R[1]
	if R[0] == 5.106000000000008:
		print("52# test R __mod__ R[0] % R[1] success")
	else:
		print("52# test R __mod__ R[0] % R[1] fail.R[0]=%s"%R[0])
	R[0] = 123.456
	MR[1] = 7
	R[0] = R[0] % MR[1]
	if R[0] == 4.456000000000003:
		print("53# test R __mod__ R[0] % MR[1] success")
	else:
		print("53# test R __mod__ R[0] % MR[1] fail.R[0]=%s"%R[0])
	
	R[0] = 123.456
	R[0] = R[0] % 7.89
	if R[0] == 5.106000000000008:
		print("54# test R __mod__ R[0] % float success")
	else:
		print("54# test R __mod__ R[0] % float fail.R[0]=%s"%R[0])
	R[0] = 123.456
	R[0] = R[0] % 7
	if R[0] == 4.456000000000003:
		print("55# test R __mod__ R[0] % int success")
	else:
		print("55# test R __mod__ R[0] % int fail.R[0]=%s"%R[0])

def test_MR_mod():
	MR[0] = 123456
	R[1] = 7.89
	MR[0] = MR[0] % R[1] #1.170000000005003
	if MR[0] == 1:
		print("56# test MR __mod__ MR[0] % R[1] success")
	else:
		print("56# test MR __mod__ MR[0] % R[1] fail.MR[0]=%s"%MR[0])
	MR[0] = 123456
	MR[1] = 7
	MR[0] = MR[0] % MR[1]
	if MR[0] == 4:
		print("57# test MR __mod__ MR[0] % MR[1] success")
	else:
		print("57# test MR __mod__ MR[0] % MR[1] fail.MR[0]=%s"%MR[0])
	
	MR[0] = 123456
	MR[0] = MR[0] % 78.9012   #54.523199999995484
	if MR[0] == 54:
		print("58# test MR __mod__ MR[0] % float success")
	else:
		print("58# test MR __mod__ MR[0] % float fail.MR[0]=%s"%MR[0])
	MR[0] = 123456
	MR[0] = MR[0] % 7
	if MR[0] == 4:
		print("59# test MR __mod__ MR[0] % int success")
	else:
		print("59# test MR __mod__ MR[0] % int fail.MR[0]=%s"%MR[0])

def test_MR_pow():
	MR[0] = 12
	R[1] = 3.45
	MR[0] = MR[0] ** R[1] #5286.585967626344
	if MR[0] == 5286:
		print("60# test MR __pow__ MR[0]**R[1] success")
	else:
		print("60# test MR __pow__ MR[0]**R[1] fail.MR[0]=%s"%MR[0])
	MR[0] = 12
	MR[1] = 3
	MR[0] = MR[0] ** MR[1]
	if MR[0] == 1728:
		print("61# test MR __pow__ MR[0]**MR[1] success")
	else:
		print("61# test MR __pow__ MR[0]**MR[1] fail.MR[0]=%s"%MR[0])
	
	MR[0] = 12
	MR[0] = MR[0] ** 3.45  
	if MR[0] == 5286:
		print("62# test MR __pow__ MR[0]**float success")
	else:
		print("62# test MR __pow__ MR[0]**float fail.MR[0]=%s"%MR[0])
	MR[0] = 12
	MR[0] = MR[0] ** 3
	if MR[0] == 1728:
		print("63# test MR __pow__ MR[0]**int success")
	else:
		print("63# test MR __pow__ MR[0]**int fail.MR[0]=%s"%MR[0])

def test_R_pow():
	R[0] = 12.34567
	R[1] = 8.901234
	R[0] = R[0] ** R[1] #5197894247.998701
	if R[0] == 5197894247.998701:
		print("64# test R __pow__ R[0]**R[1] success")
	else:
		print("64# test R __pow__ R[0]**R[1] fail.R[0]=%s"%R[0])
	R[0] = 12.34567
	MR[1] = 3
	R[0] = R[0] ** MR[1]
	if R[0] == 1881.6723022905624:
		print("65# test R __pow__ R[0]**MR[1] success")
	else:
		print("65# test R __pow__ R[0]**MR[1] fail.R[0]=%s"%R[0])
	
	R[0] = 12.34567
	R[0] = R[0] ** 8.901234
	if R[0] == 5197894247.998701:
		print("66# test R __pow__ R[0]**float success")
	else:
		print("66# test R __pow__ R[0]**float fail.R[0]=%s"%R[0])
	R[0] = 12.34567
	R[0] = R[0] ** 3
	if R[0] == 1881.6723022905624:
		print("67# test R __pow__ R[0]**int success")
	else:
		print("67# test R __pow__ R[0]**int fail.R[0]=%s"%R[0])

#小于
def test_R_lt():
	R[0] = 12.34567
	R[1] = 88.901234
	if R[0] < R[1]:
		print("68# test R __lt__ R[0] < R[1] success")
	else:
		print("68# test R __lt__ R[0] < R[1] fail")
	R[0] = 99.9	
	MR[1] = 100
	if R[0] < MR[1]:
		print("69# test R __lt__ R[0] < MR[1] success")
	else:
		print("69# test R __lt__ R[0] < MR[1] fail")
	R[0] = 99.8	
	if R[0] < 99.9:
		print("70# test R __lt__ R[0] < float success")
	else:
		print("70# test R __lt__ R[0] < float fail")
	R[0] = 99.9	
	if R[0] < 100:
		print("71# test R __lt__ R[0] < int success")
	else:
		print("71# test R __lt__ R[0] < int fail")

def test_R_le():
	R[0] = 12.34567
	R[1] = 88.901234
	if R[0] <= R[1]:
		print("72# test R __le__ R[0] <= R[1] success")
	else:
		print("72# test R __le__ R[0] <= R[1] fail")
	R[0] = 99.9	
	MR[1] = 100
	if R[0] <= MR[1]:
		print("73# test R __le__ R[0] <= MR[1] success")
	else:
		print("73# test R __le__ R[0] <= MR[1] fail")
	R[0] = 99.8	
	if R[0] <= 99.9:
		print("74# test R __le__ R[0] <= float success")
	else:
		print("74# test R __le__ R[0] <= float fail")
	R[0] = 99.9	
	if R[0] <= 100:
		print("75# test R __le__ R[0] <= int success")
	else:
		print("75# test R __le__ R[0] <= int fail")

def test_MR_lt():
	MR[0] = 99
	MR[1] = 100
	if MR[0] < MR[1]:
		print("76# test MR __lt__ MR[0] < MR[1] success")
	else:
		print("76# test MR __lt__ MR[0] < MR[1] fail")
	MR[0] = 99	
	R[1] = 99.1
	if R[0] < MR[1]:
		print("77# test MR __lt__ MR[0] < R[1] success")
	else:
		print("77# test MR __lt__ MR[0] < R[1] fail")
	MR[0] = 99	
	if MR[0] < 99.1:
		print("78# test MR __lt__ R[0] < float success")
	else:
		print("78# test MR __lt__ R[0] < float fail")
	MR[0] = 99	
	if MR[0] < 100:
		print("79# test MR __lt__ R[0] < int success")
	else:
		print("79# test MR __lt__ R[0] < int fail")

def test_MR_le():
	MR[0] = 99
	MR[1] = 100
	if MR[0] <= MR[1]:
		print("80# test MR __le__ MR[0] <= MR[1] success")
	else:
		print("80# test MR __le__ MR[0] <= MR[1] fail")
	MR[0] = 99	
	R[1] = 99.1
	if R[0] <= MR[1]:
		print("81# test MR __le__ MR[0] <= R[1] success")
	else:
		print("81# test MR __le__ MR[0] <= R[1] fail")
	MR[0] = 99	
	if MR[0] <= 99.1:
		print("82# test MR __le__ R[0] <= float success")
	else:
		print("82# test MR __le__ R[0] <= float fail")
	MR[0] = 99
	if MR[0] <= 100:
		print("83# test MR __le__ R[0] <= int success")
	else:
		print("83# test MR __le__ R[0] <= int fail")

def test_R_gt():
	R[0] = 99.9999
	R[1] = 99.9998
	if R[0] > R[1]:
		print("84# test R __gt__ R[0] > R[1] success")
	else:
		print("84# test R __gt__ R[0] > R[1] fail")
	R[0] = 99.9	
	MR[1] = 99
	if R[0] > MR[1]:
		print("85# test R __gt__ R[0] > MR[1] success")
	else:
		print("85# test R __gt__ R[0] > MR[1] fail")
	R[0] = 99.9	
	if R[0] > 99.8:
		print("86# test R __gt__ R[0] > float success")
	else:
		print("86# test R __gt__ R[0] > float fail")
	R[0] = 99.9	
	if R[0] > 99:
		print("87# test R __gt__ R[0] > int success")
	else:
		print("87# test R __gt__ R[0] > int fail")

def test_R_ge():
	R[0] = 99.9999
	R[1] = 99.9998
	if R[0] >= R[1]:
		print("88# test R __ge__ R[0] >= R[1] success")
	else:
		print("88# test R __ge__ R[0] >= R[1] fail")
	R[0] = 99.9	
	MR[1] = 99
	if R[0] >= MR[1]:
		print("89# test R __ge__ R[0] >= MR[1] success")
	else:
		print("89# test R __ge__ R[0] >= MR[1] fail")
	R[0] = 99.9	
	if R[0] >= 99.8:
		print("90# test R __ge__ R[0] >= float success")
	else:
		print("90# test R __ge__ R[0] >= float fail")
	R[0] = 99.9	
	if R[0] >= 99:
		print("91# test R __ge__ R[0] >= int success")
	else:
		print("91# test R __ge__ R[0] >= int fail")

def test_MR_gt():
	MR[0] = 100
	MR[1] = 99
	if MR[0] > MR[1]:
		print("92# test MR __gt__ MR[0] > MR[1] success")
	else:
		print("92# test MR __gt__ MR[0] > MR[1] fail")
	MR[0] = 100	
	R[1] = 99.9
	if MR[0] > R[1]:
		print("93# test MR __gt__ MR[0] > R[1] success")
	else:
		print("93# test MR __gt__ MR[0] > R[1] fail")
	MR[0] = 100	
	if MR[0] > 99.9:
		print("94# test MR __gt__ MR[0] > float success")
	else:
		print("94# test MR __gt__ MR[0] > float fail")
	MR[0] = 100	
	if MR[0] > 99:
		print("95# test MR __gt__ MR[0] > int success")
	else:
		print("95# test MR __gt__ MR[0] > int fail")

def test_MR_ge():
	MR[0] = 100
	MR[1] = 99
	if MR[0] >= MR[1]:
		print("96# test MR __gt__ MR[0] >= MR[1] success")
	else:
		print("96# test MR __gt__ MR[0] >= MR[1] fail")
	MR[0] = 100	
	R[1] = 99.9
	if MR[0] >= R[1]:
		print("97# test MR __gt__ MR[0] >= R[1] success")
	else:
		print("97# test MR __gt__ MR[0] >= R[1] fail")
	MR[0] = 100	
	if MR[0] >= 99.9:
		print("98# test MR __gt__ MR[0] >= float success")
	else:
		print("98# test MR __gt__ MR[0] >= float fail")
	MR[0] = 100	
	if MR[0] >= 99:
		print("99# test MR __gt__ MR[0] >= int success")
	else:
		print("99# test MR __gt__ MR[0] >= int fail")


def test_R_ne():
	R[0] = 99.9999
	R[1] = 99.9998
	if R[0] != R[1]:
		print("100# test R __ne__ R[0] != R[1] success")
	else:
		print("100# test R __ne__ R[0] != R[1] fail")
	R[0] = 99.9	
	MR[1] = 99
	if R[0] != MR[1]:
		print("101# test R __ne__ R[0] != MR[1] success")
	else:
		print("101# test R __ne__ R[0] != MR[1] fail")
	R[0] = 99.9	
	if R[0] != 99.8:
		print("102# test R __ne__ R[0] != float success")
	else:
		print("102# test R __ne__ R[0] != float fail")
	R[0] = 99.9	
	if R[0] != 99:
		print("103# test R __ne__ R[0] != int success")
	else:
		print("103# test R __ne__ R[0] != int fail")
def test_MR_ne():
	MR[0] = 99
	MR[1] = 88
	if MR[0] != MR[1]:
		print("104# test MR __ne__ MR[0] != MR[1] success")
	else:
		print("104# test MR __ne__ MR[0] != MR[1] fail")
	MR[0] = 100	
	R[1] = 99.9
	if MR[0] != R[1]:
		print("105# test MR __ne__ MR[0] != R[1] success")
	else:
		print("105# test MR __ne__ MR[0] != R[1] fail")
	MR[0] = 100	
	if MR[0] != 99.9:
		print("106# test MR __ne__ MR[0] != float success")
	else:
		print("106# test MR __ne__ MR[0] != float fail")
	MR[0] = 100	
	if MR[0] != 99:
		print("107# test MR __ne__ MR[0] != int success")
	else:
		print("107# test MR __ne__ MR[0] != int fail")




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




