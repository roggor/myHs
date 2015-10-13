/*
 * cli.cpp
 *
 *  Created on: Sep 29, 2015
 *      Author: maciek
 */
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>

#include "globalConfig.h"
#include "cli.h"
#include "protocol.h"

/* TODO:
 * zmienic te bufory przy kadej komendzie, przy najmniej na jakis jeden globalny
 *
 */

/*
 * extern C to dla kompilatora g++, bo poniewaz w g++ moga byc przeciazone funkcje
 * tzn w zaleznosci od parametrow ta funkcja bedzie sie troche inaczej nazywac w .o (objdump)
 * i wtedy nie jest znajdowana w bibliotece
 */
extern "C"
{
#include <libcli.h>
}

int usartComRxPrintStats(struct cli_def *cli, char *command, char *argv[], int argc)
{
	//TODO: change it somehow
	char buf[1000];
	unsigned int ret;
	ret=protoGetGlobalStats(buf);
    cli_print(cli, "%s\nSize: %u out of %lu\n", buf, ret, sizeof(buf));
    return CLI_OK;
}

int stanStats(struct cli_def *cli, char *command, char *argv[], int argc)
{
	//TODO: change it somehow
	char buf[1000];
	unsigned int ret, k;

	if (argc < 1)
	{
		for(k=0; k<(MAX_STAN+1); k++)
		{
			ret=protoGetPerStanStats(buf, k);
			cli_print(cli, "%s\nSize: %u out of %lu\n", buf, ret, sizeof(buf));
		}
	}
	else if ((atoi(argv[0]) < (MAX_STAN+1)) && (atoi(argv[0])>=0))
	{
		ret=protoGetPerStanStats(buf, atoi(argv[0]));
		cli_print(cli, "%s\nSize: %u out of %lu\n", buf, ret, sizeof(buf));
	}
	else
	{
		cli_print(cli, "Invalid argument\n");
	}

	return CLI_OK;
}

void InitCli(void)
{
	struct sockaddr_in servaddr;
	struct cli_command *stats;
	struct cli_def *cli;
	int on = 1, x, s;

	cli = cli_init();

	// Set the hostname (shown in the the prompt)
	cli_set_hostname(cli, "myHs");
	// Set the greeting
	cli_set_banner(cli, "Welcome to myHs.");

	// Set up a few simple one-level commands
	stats=cli_register_command(cli, NULL, "prot", NULL, PRIVILEGE_UNPRIVILEGED, MODE_EXEC, "help prot");
	cli_register_command(cli, stats, "globStats", usartComRxPrintStats, PRIVILEGE_UNPRIVILEGED, MODE_EXEC, "help globStats");
	cli_register_command(cli, stats, "stanStats", stanStats, PRIVILEGE_UNPRIVILEGED, MODE_EXEC, "help stanStats");

	// Create a socket
	s = socket(AF_INET, SOCK_STREAM, 0);
	setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

	// Listen on port 12345
	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	servaddr.sin_port = htons(12345);
	bind(s, (struct sockaddr *)&servaddr, sizeof(servaddr));

	// Wait for a connection
	listen(s, 2);

	while ((x = accept(s, NULL, 0)))
	{
		// Pass the connection off to libcli
		cli_loop(cli, x);
		close(x);
	}

	// Free data structures
	cli_done(cli);
}
