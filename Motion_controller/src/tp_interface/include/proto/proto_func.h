/**
 * @file proto_func.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2017-10-11
 */

#ifndef PROTO_FUNC_H_
#define PROTO_FUNC_H_

#include "proto_parse.h"

typedef struct _ProtoFunctions
{
    void (ProtoParse::*setMsg)(const uint8_t *in_buf, int in_len, void *out_buf);
    void (ProtoParse::*getMsg)(const uint8_t *in_buf, int in_len, void *out_buf);
}ProtoFunctions;


std::map<int, ProtoFunctions> g_proto_funcs_mp = 
	/*
{							
	{	32965,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	31619,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	48981,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	40629,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	36933,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	7076,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	1284,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	77427,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	44979,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	53909,	{	&ProtoParse::decToolFrame,	&ProtoParse::encToolFrame	}	},
	{	24293,	{	&ProtoParse::decUserFrame,	&ProtoParse::encUserFrame	}	},
	{	80242,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	94084,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	45710,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	33367,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	8837,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	99488,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	4276,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	9523,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	9524,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	79844,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	75299,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	97967,	{	&ProtoParse::decDefault,	&ProtoParse::encIOInfo	}	},
	{	15923,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	43859,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	53925,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	50980,	{	&ProtoParse::decSoftConstraint,	&ProtoParse::encSoftConstraint	}	},
	{	32184,	{	&ProtoParse::decDefault,	&ProtoParse::encDHParameters	}	},
	{	57668,	{	&ProtoParse::decDefault,	&ProtoParse::encHardConstraint	}	},
	{	77076,	{	&ProtoParse::decManualCmd,	&ProtoParse::encDefault	}	},
	{	17993,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	11892,	{	&ProtoParse::decTeachTarget,	&ProtoParse::encDefault	}	},
	{	94965,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	}, 
};	*/

{							
	{	32965,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	31619,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	48981,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	40629,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	69397,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	50277,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	36933,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	7076,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	1284,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	77427,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	44979,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	80242,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	94084,	{	&ProtoParse::decDefault,	&ProtoParse::encString	}	},
	{	45710,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	33367,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	8837,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	99488,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	4276,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	9523,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	9524,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	79844,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	75299,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	97967,	{	&ProtoParse::decDefault,	&ProtoParse::encIOInfo	}	},
	{	15923,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	43859,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	53925,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	50980,	{	&ProtoParse::decSoftConstraint,	&ProtoParse::encSoftConstraint	}	},
	{	35940,	{	&ProtoParse::decSoftConstraint,	&ProtoParse::encSoftConstraint	}	},
	{	32184,	{	&ProtoParse::decDefault,	&ProtoParse::encDHParameters	}	},
	{	57668,	{	&ProtoParse::decDefault,	&ProtoParse::encHardConstraint	}	},
	{	77076,	{	&ProtoParse::decManualCmd,	&ProtoParse::encDefault	}	},
	{	17993,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	11892,	{	&ProtoParse::decTeachTarget,	&ProtoParse::encDefault	}	},
	{	94965,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	10478,	{	&ProtoParse::decDefault,	&ProtoParse::encVersionInfo	}	},
	{	87442,	{	&ProtoParse::decDefault,	&ProtoParse::encDefault	}	},
	{	53909,	{	&ProtoParse::decFrame,	&ProtoParse::encFrame	}	},
	{	24293,	{	&ProtoParse::decFrame,	&ProtoParse::encFrame	}	},
	{	49429,	{	&ProtoParse::decActivateFrame,	&ProtoParse::encActivateFrame	}	},
	{	67525,	{	&ProtoParse::decActivateFrame,	&ProtoParse::encActivateFrame	}	},
	{	65444,	{	&ProtoParse::decFrameIDList,	&ProtoParse::encFrameIDList	}	},
	{	37812,	{	&ProtoParse::decFrameIDList,	&ProtoParse::encFrameIDList	}	},
	{	34069,	{	&ProtoParse::decRegister,	&ProtoParse::encRegister	}	},
	{	32882,	{	&ProtoParse::decRegister,	&ProtoParse::encRegister	}	},
	{	4855,	{	&ProtoParse::decDefault,	&ProtoParse::encString	}	},
	{	22462,	{	&ProtoParse::decDefault,	&ProtoParse::encGlobalAcc	}	},

};


	

	


#endif //#ifndef PROTO_FUNC_H_

