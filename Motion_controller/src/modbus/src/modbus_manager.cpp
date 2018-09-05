#include "modbus_manager.h"
//#include "common_file_path.h"
#include "common_log.h"
#include <string>

#include <boost/date_time/posix_time/posix_time.hpp>

using namespace fst_modbus;

ModbusManager::ModbusManager():
    //file_path_(COMPONENT_PARAM_FILE_DIR),
    log_level_(fst_log::MSG_LEVEL_ERROR)
{
    //file_path_ += "tp_comm.yaml";
}
	
//------------------------------------------------------------
// Function:	init
// Summary: Initialize 
// In:		None
// Out: 	None
// Return:	U64 -> error codes.
//------------------------------------------------------------
U64 ModbusManager::init(int fake)
{
	LocalIP local_ip_;
	int start_mode = getStartMode();
	int result = 0;
	switch (fake)
	{
		case 0:
			break;
		case 1:
			break;
		default:
			break;
	}
	
	if(START_MODE_SERVER == start_mode)
	{
		startThread();
		usleep(10);
		tcp_client = new fst_modbus::ModbusTCPClient(local_ip_.get(), getConfigPort());
   		tcp_client->setDebug(true);
	}
	else 
	{
		tcp_client = new fst_modbus::ModbusTCPClient(getConfigIP(), getConfigPort());
   		tcp_client->setDebug(true);
	}
	return SUCCESS;

}

//------------------------------------------------------------
// Function:	getDevicesNum
// Summary: get the number of devices
// In:		None
// Out: 	None
// Return:	int -> the total number of io devices.
//------------------------------------------------------------
int ModbusManager::getDevicesNum(void)
{
	int num = 0;
	return num;
}

//------------------------------------------------------------
// Function:	getDeviceInfo
// Summary: get the information of each device.
// In:		index -> the sequence number of the device.
// Out: 	info  -> the information of each device.
// Return:	U64   -> error codes.
//------------------------------------------------------------
U64 ModbusManager::getDeviceInfo(unsigned int index, fst_io_manager::IODeviceInfo &info)
{
	return SUCCESS;
}

//------------------------------------------------------------
// Function:	getModuleValue
// Summary: get the status value of one port on one device.
// In:		id		   -> the parameter id according to the parameter path.
//			port_type  -> IO_INPUT or IO_output.
//			port_seq   -> the sequence number of the ports.
// Out: 	port_value -> the port status.
// Return:	U64 	   -> error codes.
//------------------------------------------------------------
U64 ModbusManager::getModuleValue(unsigned int id, int port_type, unsigned int port_seq, unsigned char &port_value)
{
    uint8_t* tab_rq_bits = new uint8_t[10];
    fst_modbus::ModbusStatus read_status;
	// read_status.addr = getaddrefromseq(port_seq);
    read_status.dest = tab_rq_bits;
    if (!tcp_client->readCoils(read_status))
        printf("Test Modbus Client : Failed write and read coils: nb = %d\n", 10);
    else
        printf("Test Modbus Client : Success write and read coils.\n");
	port_value = tab_rq_bits[0];
	delete[] tab_rq_bits;
	return SUCCESS;
}

//------------------------------------------------------------
// Function:	getModuleValues
// Summary: get the status values of all ports on one device
// In:		id	-> the parameter id according to the parameter path.
//			len -> the size of the memory to store port values.
// Out: 	ptr -> the address of the memoryto store port values.
//			num -> the total number of the ports.
// Return:	U64 -> error codes.
//------------------------------------------------------------
U64 ModbusManager::getModuleValues(unsigned int id, int len, unsigned char *ptr, int &num)
{
    uint8_t* tab_rq_bits = new uint8_t[num];
    fst_modbus::ModbusStatus read_status;
    read_status.dest = tab_rq_bits;
    if (!tcp_client->readCoils(read_status))
        printf("Test Modbus Client : Failed write and read coils: nb = %d\n", num);
    else
        printf("Test Modbus Client : Success write and read coils.\n");
	memcpy(ptr, tab_rq_bits, num);
	delete[] tab_rq_bits;
	return SUCCESS;
}

//------------------------------------------------------------
// Function:	setModuleValue
// Summary: set the status value of one port on one device.
// In:		id		   -> the parameter id according to the parameter path.
//			port_seq   -> the sequence number of the port.
//			port_value -> the status value of the port.
// Out: 	None.
// Return:	U64 	   -> error codes.
//------------------------------------------------------------
U64 ModbusManager::setModuleValue(unsigned int id, unsigned int port_seq, unsigned char port_value)
{	   
	return SUCCESS;
}

//------------------------------------------------------------
// Function:	getThreadError
// Summary: get the error status of the io thread.
// In:		None.
// Out: 	None.
// Return:	U64 -> error codes.
//------------------------------------------------------------
U64 ModbusManager::getIOError(void)
{
	return SUCCESS;
}


//------------------------------------------------------------
// Function:    startThread
// Summary: start a thread.
// In:      None.
// Out:     None.
// Return:  None.
//------------------------------------------------------------
void ModbusManager::startThread(void)
{
    io_thread = boost::thread(boost::bind(&ModbusManager::runThread, this));
    //io_thread.timed_join(boost::posix_time::milliseconds(100)); // check the thread running or not.
}

//------------------------------------------------------------
// Function:    runThread
// Summary: main function of io thread.
// In:      None.
// Out:     None.
// Return:  None.
//------------------------------------------------------------
void ModbusManager::runThread(void)
{
    try
    {
	    tcp_server = new fst_modbus::ModbusTCPServer(getConfigPort());

	    if (!tcp_server->init(1))
	    {
	        printf("Modbus: Failed to init tcp server!\n");
	        return ;
	    }

	    tcp_server->setDebug(true);
		
        while (true)
        {
	        uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
	        if (0 <= tcp_server->receive(query))
	        {
	            tcp_server->reply(query, MODBUS_TCP_MAX_ADU_LENGTH);
	        }
	        else
	        {
	            printf("Modbus : Connection closed.\n");
	            tcp_server->close(); //close ctx
	            tcp_server->accept(); //accept next frames from client
	        }
            // set interruption point.
            boost::this_thread::sleep(boost::posix_time::microseconds(LOOP_CYCLE));
        }
    }
    catch (boost::thread_interrupted &)
    {
        std::cout<<"~Stop IO Thread Safely.~"<<std::endl;
    }
}


