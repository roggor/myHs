//============================================================================
// Name        : myHs.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
#include "globalConfig.h"
#include "protocol.h"
#include "stanowiska.h"
#include "cli.h"
/*
 * TODO:
 * -sprawdzic tego selecta bo jak jeden wyswietlacz nie bedzie odpowiadal to inne tez beda wolno pytane
 * -mozna robic reada np po max 2 bajty i wtedy ten timeout mozna zmniejszyc
 */

int main() {

	stanInit();
	protoInit();
	InitCli();

	return 0;
}

