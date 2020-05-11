#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
#include <vector>

using namespace std;

class TCPClient {
 private:
  int sock;
  std::string address;
  int port;
  struct sockaddr_in server;

 public:
  TCPClient();
  bool setup(string address, int port);
  bool send_msg(string data);
  string receive_msg(int size = 4096);
  string read();
};
