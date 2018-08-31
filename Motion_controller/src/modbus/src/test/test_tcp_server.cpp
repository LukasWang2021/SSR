#include "tcp_server.h"

int main(void)
{

    fst_modbus::ModbusTCPServer tcp_server(1025);

    if (!tcp_server.init(1))
    {
        printf("Modbus: Failed to init tcp server!\n");
        return -1;
    }

    tcp_server.setDebug(true);

    if (!tcp_server.mapping_new(500, 500, 500, 500))
    {
        printf("Modbus : Failed mapping!\n");
        return -1;
    }

    for(;;)
    {
        uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
        if (0 <= tcp_server.receive(query))
        {
            tcp_server.reply(query, MODBUS_TCP_MAX_ADU_LENGTH);
        }
        else
        {
            printf("Modbus : Connection closed.\n");
            tcp_server.close(); //close ctx
            tcp_server.accept(); //accept next frames from client
        }
    }

    printf("Modbus : Quit the loop\n");
    return 0;
}