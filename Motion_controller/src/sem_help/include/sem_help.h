#ifndef SEM_HELP_H
#define SEM_HELP_H

#include <semaphore.h>
#include <string>

namespace base_space
{
class SemHelp
{
private:
    sem_t sem_;
    bool is_taken_;
public:
    SemHelp();
    /* 
       The pshared argument indicates whether this semaphore is to be shared between the threads of a process, or between processes.

       If  pshared  has the value 0, then the semaphore is shared between the threads of a process, 
       and should be located at some address that is visible to all threads (e.g., a global variable, or a variable allocated dynamically on the heap).

       If pshared is nonzero, then the semaphore is shared between processes, and should be located in a region of 
       shared memory (see shm_open(3), mmap(2), and shmget(2)).  (Since a child created  by
       fork(2)  inherits  its  parent's  memory  mappings,  it  can  also  access the semaphore.)  
       Any process that can access the shared memory region can operate on the semaphore using sem_post(3),sem_wait(3), etc.

       The value argument specifies the initial value for the semaphore.
    */
    SemHelp(int pshared, int value);
    ~SemHelp();

public:
    int take(void);
    /*timeout with millisecond*/
    int take(int timeout);
    int give(void);
    /*returns the sem_ is been taken or not*/
    bool isTaken(void);
    
    /* 
       call sem_getvalue places the current value of the semaphore pointed to sem into the integer pointed to by [val].
       If  one or more processes or threads are blocked waiting to lock the semaphore with sem_wait(3), 
       POSIX.1-2001 permits two possibilities for the value returned in sval: either 0 is returned; or
       a negative number whose absolute value is the count of the number of processes and threads currently blocked in sem_wait(3).
    */
    int getValue(int *val);
};
}



#endif