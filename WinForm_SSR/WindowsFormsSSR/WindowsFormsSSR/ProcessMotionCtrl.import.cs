using System;
using System.Runtime.InteropServices;

namespace ProcessMotion
{
    public partial class ProcessMotionCtrl
    { 
        private const string dllPath = @"..\Depends\dll\SmartControl_SSR.dll";//"SmartControl.dll";

        /// <summary>Logon RPC connection. Should be called at the beginning.</summary>
        /// <param name="server_ip">The IP address of the controller</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_initRpc", CallingConvention = CallingConvention.Cdecl)]
        private extern static UInt64 c_initRpc(String server_ip);

        /// <summary>Logon Subscriber connection. Should be called at the beginning.</summary>
        /// <param name="server_ip">The IP address of the controller</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_initSub", CallingConvention = CallingConvention.Cdecl)]
        private extern static UInt64 c_initSub(String server_ip);

        /// <summary>Logon Event connection. Should be called at the beginning.</summary>
        /// <param name="server_ip">The IP address of the controller</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_initEvent", CallingConvention = CallingConvention.Cdecl)]
        private extern static UInt64 c_initEvent(String server_ip);

        /// <summary>Logout Subscriber connection.</summary>
        /// <param name="server_ip">The IP address of the controller</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_exitSub", CallingConvention = CallingConvention.Cdecl)]
        private extern static UInt64 c_exitSub();

        /// <summary>Logout Event connection.</summary>
        /// <param name="server_ip">The IP address of the controller</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_exitEvent", CallingConvention = CallingConvention.Cdecl)]
        private extern static UInt64 c_exitEvent();

        /// <summary>Subscribe the topic</summary>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_addTopic", CallingConvention = CallingConvention.Cdecl)]
        private extern static UInt64 c_addTopic();

        /// <summary>Unsubscribe the topics</summary>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_deleteTopic", CallingConvention = CallingConvention.Cdecl)]
        private extern static UInt64 c_deleteTopic();

        //axis API

        /// <summary>Control the power stage(On or Off).</summary>
        /// <param name="axis_id"> The ID of the axis</param>
        /// <param name="enable">1:power on. 0:power off.</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_axisPower", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_axisPower(Int32 axis_id, Int32 enable);

        /// <summary>Reset all internal axis-related errors.</summary>
        /// <param name="axis_id"> The ID of the axis</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_axisReset", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_axisReset(Int32 axis_id);

        /// <summary>Stop the axis motion.</summary>
        /// <param name="axis_id"> The ID of the axis</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_axisStop", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_axisStop(Int32 axis_id);

        /// <summary>Set the offset position of the axis</summary>
        /// <param name="axis_id"> The ID of the axis</param>
        /// <param name="position">The desired position value of the axis</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_axisSetPosition", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_axisSetPosition(Int32 axis_id, double position);

        /// <summary>Command a controlled motion to a specified absolute position.</summary>
        /// <param name="axis_id"> The ID of the axis</param>
        /// <param name="position">The absolute position input</param>
        /// <param name="velocity">The value of the maximum velocity</param>
        /// <param name="acc">The value of the acceleration(always positive)</param>
        /// <param name="dec">The value of the deceleration(always positive)</param>
        /// <param name="jerk">The value of the jerk(always positive)</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_axisMoveAbsolute", CallingConvention = CallingConvention.Cdecl)]
        public static extern UInt64 c_axisMoveAbsolute(Int32 axis_id, double position, double velocity, double acc, double dec, double jerk);

        /// <summary>Command a controlled motion to a specified relative position.</summary>
        /// <param name="axis_id"> The ID of the axis</param>
        /// <param name="position">The relative position input</param>
        /// <param name="velocity">The value of the maximum velocity</param>
        /// <param name="acc">The value of the acceleration(always positive)</param>
        /// <param name="dec">The value of the deceleration(always positive)</param>
        /// <param name="jerk">The value of the jerk(always positive)</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_axisMoveRelative", CallingConvention = CallingConvention.Cdecl)]
        public static extern UInt64 c_axisMoveRelative(Int32 axis_id, double position, double velocity, double acc, double dec, double jerk);

        /// <summary>Ask the controller for state.</summary>
        /// <param name="axis_id"> The ID of the axis</param>
        /// <param name="status">The state of the axis.</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_axisReadAxisStatus", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_axisReadAxisStatus(Int32 axis_id, ref Int32 status);

        /// <summary>Ask the controller for position.</summary>
        /// <param name="axis_id"> The ID of the axis</param>
        /// <param name="pos">The position of the axis.</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_axisReadActualPosition", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_axisReadActualPosition(Int32 axis_id, ref double pos);

        /// <summary>Reset the encoder error.</summary>
        /// <param name="axis_id"> The ID of the axis</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_axisResetEncoder", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_axisResetEncoder(Int32 axis_id);

        //group API

        /// <summary>Reset group-related errors.</summary>
        /// <param name="group_id"> The ID of the group</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_groupReset", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_groupReset(Int32 group_id);

        /// <summary>Power on.</summary>
        /// <param name="group_id"> The ID of the group</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_groupEnable", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_groupEnable(Int32 group_id);

        /// <summary>Power off.</summary>
        /// <param name="group_id"> The ID of the group</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_groupDisable", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_groupDisable(Int32 group_id);

        /// <summary>Get the controller error.</summary>
        /// <param name="group_id"> The ID of the group</param>
        /// <param name="error">The error of the group.</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_groupReadError", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_groupReadError(Int32 group_id, ref double error);

        /// <summary>Ask the controller for state.</summary>
        /// <param name="group_id"> The ID of the group</param>
        /// <param name="status">The state of the group.</param>
        /// <param name="in_pos">Reach the position or not.</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_groupReadStatus", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_groupReadStatus(Int32 group_id, ref Int32 status, ref Int32 in_pos);

        /// <summary>Reset the group encoder error.</summary>
        /// <param name="group_id"> The ID of the group</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_groupResetAllEncoder", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_groupResetAllEncoder(Int32 group_id);


        //motion_control API

        /// <summary>Set the velocity ratio.</summary>
        /// <param name="group_id">The ID of the group</param>
        /// <param name="ratio">The ratio of the velocity.</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_mcSetGlobalVelRatio", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_mcSetGlobalVelRatio(Int32 ratio);

        /// <summary>Set the accelaration ratio.</summary>
        /// <param name="group_id">The ID of the group</param>
        /// <param name="ratio">The ratio of the acceleration.</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_mcSetGlobalAccRatio", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_mcSetGlobalAccRatio(Int32 ratio);

        /// <summary>Do manual step move.</summary>
        /// <param name="axis_id">The ID of the axis</param>
        /// <param name="direction">The direction of the movement.0-stand, 1-increase, 2-decrease</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_mcDoStepManualMove", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_mcDoStepManualMove(Int32 axis_id, Int32 direction);

        /// <summary>Do manual continuous move.</summary>
        /// <param name="axis_id">The ID of the axis</param>
        /// <param name="direction">The direction of the movement.0-stand, 1-increase, 2-decrease</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_mcDoContinuousManualMove", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_mcDoContinuousManualMove(Int32 axis_id, Int32 direction);

        /// <summary>Do manual continuous move to standstill.</summary>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_mcDoContinuousManualToStandstill", CallingConvention = CallingConvention.Cdecl)]
        public extern static void c_mcDoContinuousManualToStandstill();

        /// <summary>Do manual move to cartesian position.</summary>
        /// <param name="group_id">The ID of the axis</param>
        /// <param name="x">The x position</param>
        /// <param name="y">The y position</param>
        /// <param name="z">The z position</param>
        /// <param name="a">The a position</param>
        /// <param name="b">The b position</param>
        /// <param name="c">The c position</param>
        /// <param name="arm_left_right">The posture arm.1: front arm, -1: back arm.</param>
        /// <param name="elbow_up_down">The posture eblow.1: elbow above wrist, -1: elbow below wrist.</param>
        /// <param name="wrist_down_up">The posture wrist.1: wrist face down, -1: wrist face up.</param>
        /// <param name="uf_id">The user frame id.</param>
        /// <param name="tf_id">The tool frame id.</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_mcDoGotoCartesianMove", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_mcDoGotoCartesianMove(Int32 group_index, double x, double y, double z, double a, double b, double c,
    Int32 arm_left_right, Int32 elbow_up_down, Int32 wrist_down_up, Int32 uf_id, Int32 tf_id);

        /// <summary>Do manual move to joint positions.</summary>
        /// <param name="group_id">The ID of the axis</param>
        /// <param name="j1">The x position</param>
        /// <param name="j2">The y position</param>
        /// <param name="j3">The z position</param>
        /// <param name="j4">The a position</param>
        /// <param name="j5">The b position</param>
        /// <param name="j6">The c position</param>
        /// <param name="j7">The posture arm.1: front arm, -1: back arm.</param>
        /// <param name="j8">The posture eblow.1: elbow above wrist, -1: elbow below wrist.</param>
        /// <param name="j9">The posture wrist.1: wrist face down, -1: wrist face up.</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_mcDoGotoJointMove", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_mcDoGotoJointMove(Int32 group_index, double j1, double j2, double j3, double j4, double j5, double j6, double j7, double j8, double j9);

        /// <summary>Set the coordinate system.</summary>
        /// <param name="coord_type">"Joint/Base/World/User/Tool"</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_mcSetCoordinate", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_mcSetCoordinate(string coord_type);

        /// <summary>Ignore the lost offset.</summary>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_mcIgnoreLostZeroError", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_mcIgnoreLostZeroError();

        /// <summary>Set the zero offset.</summary>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_mcSetAllZeroPointOffsets", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_mcSetAllZeroPointOffsets();

        /// <summary>Set the step value for the movement.</summary>
        /// <param name="joint_step">The step value of the joint movement</param>
        /// <param name="cartesian_step">The step value of the cartesian movement</param>
        /// <param name="orientation_step">The step value of the orientation movement</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_mcSetStep", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_mcSetStep(double joint_step, double cartesian_step, double orientation_step);


        //feedback 

        /// <summary>Get all the axes feedback.</summary>
        /// <param name="array_size">The lengh of the array</param>
        /// <param name="isr">The Interrupt Service Routine of servo</param>
        /// <param name="state">The status of the axes</param>
        /// <param name="position">The positions of the axes</param>
        /// <param name="velocity">The velocity of the axes</param>
        /// <param name="torque">The torque of the axes</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_getTopicAxisFeedback", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_getTopicAxisFeedback(UInt32 array_size, UInt32[] isr, UInt32[] state, double[] position, double[] velocity, double[] torque);


        /// <summary>Get the error code from Controller.</summary>
        /// <param name="error">The error code buffer. The size = 8.</param>
        /// <param name="time_stamp">time_stamp The ISR of the error. The size = 8</param>
        /// <param name="size_ptr">The number of the error codes in the buffer</param>
        /// <returns>error_code</returns>
        [DllImport(dllPath, EntryPoint = "c_getEventErrorList", CallingConvention = CallingConvention.Cdecl)]
        public extern static UInt64 c_getEventErrorList(UInt64[] error, UInt64[] time_stamp, ref UInt32 size_ptr);




    }
}
