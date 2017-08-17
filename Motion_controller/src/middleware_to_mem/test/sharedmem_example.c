
#include "structure_mem/struct_feedback_joint_states.h"
#include "middleware_to_sharedmem/middleware_to_sharedmem.h"

main(int argc, char** argv)
{
    int handle = openMem(MEM_CORE);
    //Writing the feedback joint states
    FeedbackJointState fbjs1;
    for(int i = 0;i < JOINT_NUM; ++i)
    {
        fbjs1.position[i] = i;
        fbjs1.velocity[i] = i * 0.1;
        fbjs1.effort[i] = i * 0.01;
    }
    fbjs1.state = STATE_READY;
    int write_result = readWriteSharedMem(handle, &fbjs1, "FeedbackJointState", MEM_WRITE);
    
    //Reading the feedback joint states
    FeedbackJointState fbjs2;
    int read_result = readWriteSharedMem(handle, &fbjs2, "FeedbackJointState", MEM_READ);
    if (read_result == true)
    {
        //use fbjs2;
        for(i=0;i<JOINT_NUM; i++){printf("fbjs.position[%d] = %f \n", i, fbjs2.position[i]);}
        for(i=0;i<JOINT_NUM; i++){printf("fbjs.velocity[%d] = %f \n", i, fbjs2.velocity[i]);}
        for(i=0;i<JOINT_NUM; i++){printf("fbjs.effort[%d] = %f \n", i, fbjs2.effort[i]);}
        printf(" fbjs.state = %d\n\n", fbjs2.state);
    }


   
    return 0;
}


