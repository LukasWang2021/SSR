
#include <iostream>
#include <fstream>
#include <ctime>
#include <unistd.h>
#include <sys/time.h>


#include <parameter_manager/parameter_manager_param_builder.h>
#include <parameter_manager/parameter_manager_param_group.h>
#include <log_manager/log_manager_logger.h>


using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::map;



int main(int argc, char **argv)
{
	cout << "hello world" << endl;
    
    fst_log::Logger log;
	int loop = 0;
    int i = 0;
    struct timeval time_now, time_past;

    while (i < 100) {
        i++;
        loop = 0;
        log.info("i=%d", i);
	    
        while (loop < 10) {
		    loop++;
            log.info("loop=%d", loop);
    		
            fst_parameter::ParamValue pv;
	        fst_parameter::ParamBuilder builder;
    		char buf[128];
	    	snprintf(buf, sizeof(buf), "/root/install/share/configuration/test%d.yaml", loop);
            log.info(buf);
            
            gettimeofday(&time_past, NULL);
    		std::ifstream in(buf);
            gettimeofday(&time_now, NULL);
            if (time_now.tv_sec - time_past.tv_sec > 1) {
                log.error("  open file time overflow");
                log.error("  time before open:%d.%6d", time_past.tv_sec, time_past.tv_usec);
                log.error("  time  after open:%d.%6d", time_now.tv_sec,  time_now.tv_usec);
                return 0;
            }
            else {
                log.info("  open file success");
            }

            gettimeofday(&time_past, NULL);
		    string yaml_str((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
            gettimeofday(&time_now, NULL);
            if (time_now.tv_sec - time_past.tv_sec > 1) {
                log.error("  read file time overflow");
                log.error("  time before read:%d.%6d", time_past.tv_sec, time_past.tv_usec);
                log.error("  time  after read:%d.%6d", time_now.tv_sec,  time_now.tv_usec);
                return 0;
            }
            else {
                log.info("  read file success");
            }

            gettimeofday(&time_past, NULL);
            in.close();
            gettimeofday(&time_now, NULL);
            if (time_now.tv_sec - time_past.tv_sec > 1) {
                log.error("  close file time overflow");
                log.error("  time before close:%d.%6d", time_past.tv_sec, time_past.tv_usec);
                log.error("  time  after close:%d.%6d", time_now.tv_sec,  time_now.tv_usec);
                return 0;
            }
            else {
                log.info("  close file success");
            }

    		//cout << "string :" << endl << yaml_str << endl;
	    	//cout << "Build param_value from YAML" << endl;
            try {
    		    builder.buildParamFromString(yaml_str, pv);
	    	}
            catch (fst_parameter::ParamException &e) {
                log.error("parse failed.");
                log.error(e.getMessage().c_str());
                return 0;
		    	// cout << pv << endl;
    		}

    		string dump;
            try {
		        builder.dumpParamToString(pv, dump);
                dump = dump + "\r#END";
                // cout << dump << endl;
	    	}
            catch (fst_parameter::ParamException &e) {
                log.error("dump failed.");
                log.error(e.getMessage().c_str());
	    		// cout << pv << endl;
		    }

            gettimeofday(&time_past, NULL);
            std::ofstream out(buf);
            gettimeofday(&time_now, NULL);
            if (time_now.tv_sec - time_past.tv_sec > 1) {
                log.error("  open file time overflow");
                log.error("  time before open:%d.%6d", time_past.tv_sec, time_past.tv_usec);
                log.error("  time  after open:%d.%6d", time_now.tv_sec,  time_now.tv_usec);
                return 0;
            }
            else {
                log.info("  open file success");
            }

            gettimeofday(&time_past, NULL);
		    out << dump;
            gettimeofday(&time_now, NULL);
            if (time_now.tv_sec - time_past.tv_sec > 1) {
                log.error("  write file time overflow");
                log.error("  time before write:%d.%6d", time_past.tv_sec, time_past.tv_usec);
                log.error("  time  after write:%d.%6d", time_now.tv_sec,  time_now.tv_usec);
                return 0;
            }
            else {
                log.info("  write file success");
            }

            gettimeofday(&time_past, NULL);
            in.close();
            gettimeofday(&time_now, NULL);
            if (time_now.tv_sec - time_past.tv_sec > 1) {
                log.error("  close file time overflow");
                log.error("  time before close:%d.%6d", time_past.tv_sec, time_past.tv_usec);
                log.error("  time  after close:%d.%6d", time_now.tv_sec,  time_now.tv_usec);
                return 0;
            }
            else {
                log.info("  close file success");
            }

            usleep(2 * 1000);
	    }

        usleep(50 * 1000);
    }

	return 0;
}


