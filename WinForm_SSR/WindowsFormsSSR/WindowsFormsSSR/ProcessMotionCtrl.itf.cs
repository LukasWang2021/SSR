using System;
using System.Runtime.InteropServices;

namespace ProcessMotion
{
    public partial class ProcessMotionCtrl
    {
        /// <summary>Start connection. Should be called at the beginning.</summary>
        /// <param name="server_ip">The IP address of the controller</param>
        /// <returns>error_code</returns>
        public static UInt64 StartConnection(String server_ip)
        {
            UInt64 ret = 0;
            ret = c_initRpc(server_ip);
            if (ret != 0) return ret;
            ret = c_initSub(server_ip);
            if (ret != 0) return ret;
            ret = c_initEvent(server_ip);
            if (ret != 0) return ret;
            c_deleteTopic();
            ret = c_addTopic();
            if (ret != 0) return ret;
            return ret;
        }

        /// <summary>Stop connection. Should be called when exiting.</summary>
        /// <returns>error_code</returns>
        public static UInt64 ExitConnection()
        {
            UInt64 ret = 0;
            ret = c_deleteTopic();
            if (ret != 0) return ret;
            ret = c_exitEvent();
            if (ret != 0) return ret;
            ret = c_exitSub();
            if (ret != 0) return ret;

            return ret;
        }


    }
}
