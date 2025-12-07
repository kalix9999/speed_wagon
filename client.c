#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define PORT 9000
#define BUF_SIZE 1024

void error_handling(char *message);

int main(int argc, char *argv[])
{
	int sock;
    struct sockaddr_in serv_addr;
    char message[BUF_SIZE];
    int str_len;

	char *server_ip = "127.0.0.1";

	if(argc == 2) server_ip = argv[1];
	sock = socket(PF_INET, SOCK_STREAM, 0);
    if (sock == -1) error_handling("socket() error");

	memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr(server_ip);
    serv_addr.sin_port = htons(PORT);

	if(connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) == -1) error_handling("connect() error!");
	printf("서버 연결 성공! 소문자 문장을 입력하세요 (Q to quit):\n");

	while(1)
	{

	
	if (fgets(message, BUF_SIZE, stdin) == NULL)
        break;


	if(!strcmp(message,"q\n") || !strcmp(message,"Q\n")) break;

	
	write(sock,message,strlen(message));
	str_len = read(sock, message, BUF_SIZE - 1);
    message[str_len] = 0; // 문자열 끝 처리

	printf("Server: %s", message);

	}
	
	
	close(sock);
    return 0;
}

void error_handling(char *message)
{
    fputs(message, stderr);
    fputc('\n', stderr);
    exit(1);
}
