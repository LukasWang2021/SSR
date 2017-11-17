
#include <iostream>
#include <fstream>
#include <ctime>
#include <unistd.h>
#include <sys/time.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>


#include <parameter_manager/parameter_manager_param_builder.h>
#include <parameter_manager/parameter_manager_param_group.h>
#include <log_manager/log_manager_logger.h>

//#define C_STYLE

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::map;

bool g_running = true;
fst_log::Logger log;

static void sigintHandle(int num)
{
    log.warn("Interrupt request catched.");
    g_running = false;
    usleep(500 * 1000);
}

int main(int argc, char **argv)
{
    signal(SIGINT, sigintHandle);

    log.initLogger("test");
    //log.setDisplayLevel(MSG_LEVEL_ERROR);
    //

#ifdef C_STYLE
    log.info("C style test:");
#else
    log.info("fstream style");
#endif

    log.info("start test?");
    getchar();
    
	int loop = 0;
    int i = 0;
    struct timeval time_now, time_past;

    while (i < 30000 * 65536 && g_running) {
        i++;
        loop = 0;
        log.info("test loop: %d", i);

        while (loop < 10 && g_running) {
		    loop++;
    		
            bool quit = false;
            fst_parameter::ParamValue pv;
	        fst_parameter::ParamBuilder builder;
    		char buf[128];
	    	snprintf(buf, sizeof(buf), "/root/install/share/configuration/test%d.yaml", loop);
            log.info(buf);
            
            gettimeofday(&time_past, NULL);
#ifdef C_STYLE
            FILE *in = fopen(buf, "r");
#else
    		std::ifstream in(buf);
#endif
            gettimeofday(&time_now, NULL);
            if (time_now.tv_sec - time_past.tv_sec > 1 || time_now.tv_usec - time_past.tv_usec > 750000) {
                log.error("  open file time overflow");
                log.error("  time before open:%d.%6d", time_past.tv_sec, time_past.tv_usec);
                log.error("  time  after open:%d.%6d", time_now.tv_sec,  time_now.tv_usec);
                quit = true;
            }
            else {
#ifdef C_STYLE
                log.info("  open file success C style");
#else
                log.info("  open file success ifstream");
#endif
            }

            gettimeofday(&time_past, NULL);
#ifdef C_STYLE
            char str[64 * 1024];
            memset(str, 0, sizeof(str));
            fread(str, sizeof(char), 64 * 1024, in);
            string yaml_str = str;
#else
		    string yaml_str((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
#endif
            gettimeofday(&time_now, NULL);
            if (time_now.tv_sec - time_past.tv_sec > 1 || time_now.tv_usec - time_past.tv_usec > 750000) {
                log.error("  read file time overflow");
                log.error("  time before read:%d.%6d", time_past.tv_sec, time_past.tv_usec);
                log.error("  time  after read:%d.%6d", time_now.tv_sec,  time_now.tv_usec);
                quit = true;
            }
            else {
                log.info("  read file success");
            }

            gettimeofday(&time_past, NULL);
#ifdef C_STYLE
            fclose(in);
#else
            in.close();
#endif
            gettimeofday(&time_now, NULL);
            if (time_now.tv_sec - time_past.tv_sec > 1 || time_now.tv_usec - time_past.tv_usec > 750000) {
                log.error("  close file time overflow");
                log.error("  time before close:%d.%6d", time_past.tv_sec, time_past.tv_usec);
                log.error("  time  after close:%d.%6d", time_now.tv_sec,  time_now.tv_usec);
                quit = true;
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
                log.error(e.getMessage());
                quit = true;
		    	// cout << pv << endl;
    		}

    		string dump;
            try {
		        builder.dumpParamToString(pv, dump);
                dump = dump + "\n#END";
                // cout << dump << endl;
	    	}
            catch (fst_parameter::ParamException &e) {
                log.error("dump failed.");
                log.error(e.getMessage());
                quit = true;
	    		// cout << pv << endl;
		    }

            gettimeofday(&time_past, NULL);
#ifdef C_STYLE
            FILE *out = fopen(buf, "w");
#else
            std::ofstream out(buf);
#endif
            gettimeofday(&time_now, NULL);
            if (time_now.tv_sec - time_past.tv_sec > 1 || time_now.tv_usec - time_past.tv_usec > 750000) {
                log.error("  open file time overflow");
                log.error("  time before open:%d.%6d", time_past.tv_sec, time_past.tv_usec);
                log.error("  time  after open:%d.%6d", time_now.tv_sec,  time_now.tv_usec);
                quit = true;
            }
            else {
                log.info("  open file success");
            }

            gettimeofday(&time_past, NULL);
#ifdef C_STYLE
		    fwrite(dump.c_str(), sizeof(char), dump.length(), out);
            //fflush(out);
#else
		    out << dump;
            //out.flush();
#endif
            gettimeofday(&time_now, NULL);
            if (time_now.tv_sec - time_past.tv_sec > 1 || time_now.tv_usec - time_past.tv_usec > 750000) {
                log.error("  write file time overflow");
                log.error("  time before write:%d.%6d", time_past.tv_sec, time_past.tv_usec);
                log.error("  time  after write:%d.%6d", time_now.tv_sec,  time_now.tv_usec);
                quit = true;
            }
            else {
                log.info("  write file success");
            }

            gettimeofday(&time_past, NULL);
#ifdef C_STYLE
		    fclose(out);
#else
            in.close();
#endif
            gettimeofday(&time_now, NULL);
            if (time_now.tv_sec - time_past.tv_sec > 1 || time_now.tv_usec - time_past.tv_usec > 750000) {
                log.error("  close file time overflow");
                log.error("  time before close:%d.%6d", time_past.tv_sec, time_past.tv_usec);
                log.error("  time  after close:%d.%6d", time_now.tv_sec,  time_now.tv_usec);
                quit = true;
            }
            else {
                log.info("  close file success");
            }

            if (quit)   return 0;
            else        usleep(2 * 1000);
	    }

        usleep(50 * 1000);
    }

	return 0;
}


