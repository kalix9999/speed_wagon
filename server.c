#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <ctype.h>

#define PORT 9000
#define BUF_SIZE 1024


void error_handling(char* message);


int main(){
	int serv_sock, clnt_sock;
	struct sockaddr_in serv_addr,clnt_addr;
	socklen_t clnt_addr_size;
	char message[BUF_SIZE];
	int str_len,i;

	serv_sock = socket(PF_INET,SOCK_STREAM,0);
	if(serv_sock == -1) error_handling("socket() error!");

	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(PORT);
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);

	if( bind(serv_sock,(struct sockaddr *)&serv_addr, sizeof(serv_addr)) == -1)
		 error_handling("bind() error");
	if(listen(serv_sock,5) == -1 ) error_handling("listen() error");
	
	printf("--- server on (Port: %d) ---\n", PORT);

	clnt_addr_size = sizeof(clnt_addr);
	
	clnt_sock = accept(serv_sock,(struct sockaddr*)&clnt_addr, &clnt_addr_size);
	if (clnt_sock == -1) error_handling("accept() error");

	printf("guest in! IP:%s\n", inet_ntoa(clnt_addr.sin_addr));

	while((str_len = read(clnt_sock,message, BUF_SIZE)) != 0)
	{
		for(i = 0; i<str_len; i++){
		message[i] = toupper(message[i]);
		}
		
		write(clnt_sock,message,str_len);
	}

	printf("guest out! server shut down!\n");

	close(clnt_sock);
    close(serv_sock);

	return 0;
}
void error_handling(char *message)
{
    fputs(message, stderr);
    fputc('\n', stderr);
    exit(1);
}
