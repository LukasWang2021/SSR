/**********************************************
File: test.c
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
Instruction: test the service functions
Author: Feng.Wu 10-oct-2016
Modifier:
**********************************************/

#ifndef TEST_C_
#define TEST_C_

#include <iostream>
#include <string.h>
#include "comm_interface/comm_interface.h"
#include "struct_to_mem/struct_service_request.h"
#include "struct_to_mem/struct_service_response.h"
#include "nanomsg/nn.h"
#include "nanomsg/reqrep.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <sys/time.h>
#include <netinet/in.h>  /* For htonl and ntohl */
#include <sys/mman.h>

int main(int argc, char *argv[]) {

  if (!fork()) { /* child */
      char rec_buff[128];
      fst_comm_interface::CommInterface comm;
//      comm.init();
      int fd = comm.createChannel(REQ, "client");
      
      if (fd == -1)
      {
          printf("Error when client createChannel.\n");
          return -1;
      }
      for (int i = 0; i < 5; ++i)
      {
//          char send_buff[128] = "From Client---hello world";
//          sprintf(send_buff, "%s+%d",send_buff, i);
//          int send = comm.send(fd, send_buff, 128);

          std::string str = "----world---opr---sssssss--";
          char *send_buff = const_cast<char*> (str.c_str());
//          char *send_buff = "From Client-hello world---";
          int send = comm.send(fd, send_buff, strlen(send_buff));

          if (send == -1)
          {
              printf("Error when client send.\n");
              return -1;
          }
          
          usleep(10000);
/*          int rc = comm.recv(fd, rec_buff, 128);
          if (rc == -1)
          {
              printf("Error when client recv.\n");
              return -1;
          }

          printf("client recv msg: %s \n", rec_buff);
*/      }
//      comm.close(fd);
      sleep(1);
      printf("client  done\n");
      return 0;
          
  } else { /* parent */
      char send_buff[128] = "from server-hello world";
      char rec_buff[128];
      memset(rec_buff, 0, sizeof(rec_buff));
//      char *rec_buff = NULL; // will be release in the method.
      fst_comm_interface::CommInterface comm;
      int fd = comm.createChannel(REP, "client");
      if (fd == -1)
      {
          printf("Error when client createChannel.\n");
          return -1;
      }
      for (int i = 0; i <5; ++i)
      {
          usleep(10000);
          int rc = comm.recv(fd, rec_buff, 128);
          if (rc == -1)
          {
              printf("Error when server recv.\n");
//              return -1;
          }
          printf("server%d recv msg: %s , size is \n", rc, rec_buff);
          

/*
          sprintf(send_buff, "%s+%d",send_buff, i);
          int send = comm.send(fd, send_buff, 128);
          if (send == -1)
          {
              printf("Error when client send.\n");
              return -1;
          }
*/
      }
//      comm.close (fd);
  }
  sleep(1);
  printf("server done\n");
  return 0;
}

#endif

