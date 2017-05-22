#include <stdio.h>
#include <time.h>
#include "E101.h"
	

int connect_to_server(char server_addr[15], int port);
int send_to_server(char message[24]);
int receive_from_server(char message[24]);

int main (){
	
	init(); 
		char pass[24];
		connect_to_server("130.195.6.196", 1024);
		send_to_server("Please");
		receive_from_server(pass);
		send_to_server(pass);

return 0;
}
