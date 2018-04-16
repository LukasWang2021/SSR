/**
 * @file launch_code_mgr.h
 * @brief 
 * @author Lujiaming
 * @version 1.0.0
 * @date 2018-04-12
 */
#ifndef LAUNCH_CODE_MGR_H_
#define LAUNCH_CODE_MGR_H_

#include <atomic>
#include <string>
#include <pb_encode.h>
#include "motionSL.pb.h"
#include "base_types.pb.h"
#include "common.h"
#include "interpreter_common.h"
#include "error_code.h"

#include <vector>
#include <string>
#include <map>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

class LaunchCodeMgr
{
  public:	 
    LaunchCodeMgr();
    ~LaunchCodeMgr();
    U64 initial();
    
    /**
     * @brief: get Program by Code
     *
     * @param iLaunchCode: Launch Code
     *
     * @return: Program Name
     */
    std::string getProgramByCode(int iLaunchCode);


    /**
     * @brief: get all Launch Code again
     *
     *
     * @return: 0 if success 
     */
    U64 updateAll();


  private:
  	int readFileList(char *basePath);
	int parseProgramPropFile(char *filename, char * progName);
	int parseLaunchCode(char * data);
	int printLaunchCodeList();
	
    std::map<int, std::string>  launchCodeList; 
};

#endif
