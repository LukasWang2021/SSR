#include "tcp_client.h"

int main(void)
{
    int start_addr = 0;
    int end_addr = 99;
    int nb = end_addr - start_addr + 1; // number of addr
    uint8_t* tab_rq_bits = new uint8_t[nb];
    uint8_t* tab_rp_bits = new uint8_t[nb];
    uint16_t* tab_rq_registers = new uint16_t[nb];
    uint16_t* tab_rp_reqisters = new uint16_t[nb];
    uint16_t* tab_rw_rq_registers = new uint16_t[nb];

    fst_modbus::ModbusTCPClient tcp_client(1025);

    if (!tcp_client.init())
    {
        printf("Modbus Client : Failed init.\n");
        return -1;
    }

    tcp_client.setDebug(true);

    int nb_loop = 0; // number of loop
    int nb_fail = 0; // number of fail
    int total_loop = 1; //total of loop

    while(nb_loop++ < total_loop)
    {
        for (int i = 0; i != nb; ++i)
        {
            tab_rq_registers[i] = (uint16_t)(65535.0 * rand()/(RAND_MAX + 1.0));
            tab_rw_rq_registers[i] =~ tab_rq_registers[i];
            tab_rq_bits[i] = (uint8_t)(65535.0 * rand()/(RAND_MAX + 1.0));
        }

        for (int addr = start_addr; addr != nb; ++addr)
        {
            // test write and read single coil
            if (!tcp_client.writeAndReadSingleCoil(addr, tab_rq_bits[0], tab_rp_bits[0]))
                printf("Test Modbus Client : Failed write and read single coil.\n");
            else
                printf("Test Modbus Client : Success write and read single coil.\n");

            // test write and read coils
            fst_modbus::ModbusStatus write_status;
            write_status.addr = addr;
            write_status.nb = nb;
            write_status.dest = tab_rq_bits;
            fst_modbus::ModbusStatus read_status;
            read_status.dest = tab_rp_bits;
            if (!tcp_client.writeAndReadCoils(write_status, read_status))
                printf("Test Modbus Client : Failed write and read coils: nb = %d\n", nb);
            else
                printf("Test Modbus Client : Success write and read coils.\n");

            // test write and read single holding reg
            if (!tcp_client.writeAndReadSingleHoldingReg(addr, tab_rq_registers[0], tab_rp_reqisters[0]))
                printf("Test Modbus Client : Failed write and read single holding reg: nb = %d\n", nb);
            else
                printf("Test Modbus Client : Success write and read single holding reg.\n");

            // test write and read holding regs
            fst_modbus::ModbusRegisters write_regs;
            write_regs.addr = addr;
            write_regs.nb = nb;
            write_regs.dest = tab_rq_registers;
            fst_modbus::ModbusRegisters read_regs;
            tab_rp_reqisters = read_regs.dest;
            if (!tcp_client.writeAndReadHoldingRegs(write_regs, read_regs))
                printf("Test Modbus Client : Failed write and read holding regs: nb = %d\n", nb);
            else
                printf("Test Modbus Client : Success write and read holding regs.\n");

            //test read discrete inputs
            fst_modbus::ModbusStatus input_status;
            input_status.addr = addr;
            input_status.nb = nb;
            input_status.dest = tab_rp_bits;
            if (!tcp_client.readDiscreteInputs(input_status))
                printf("Test Modbus Client : Failed read discrete inputs: nb = %d\n", nb);
            else
                printf("Test Modbus Client : Success read discrete inputs.\n");

            //test read input regs
            fst_modbus::ModbusRegisters read_input_regs;
            read_input_regs.addr = addr;
            read_input_regs.nb = nb;
            read_input_regs.dest = tab_rp_reqisters;
            if (!tcp_client.readInputRegs(read_input_regs))
                printf("Test Modbus Client : Failed read input regs: nb = %d\n", nb);
            else
                printf("Test Modbus Client : Success read input regs.\n");
        }
    }

    printf("Modbus Client : Test :\n");
    nb_fail = tcp_client.getFailedOperationNumber();
    if(nb_fail) printf("%d Fails\n", nb_fail);
    else printf("Success!\n");

    delete tab_rq_bits;
    delete tab_rp_bits;
    delete tab_rq_registers;
    delete tab_rp_reqisters;
    delete tab_rw_rq_registers;

    printf("Modbus : Quit the loop\n");
    return 0;
}
