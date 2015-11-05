//============================================================================
// Name        : myHs.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
#include "cli.h"
#include "configGlobal.h"
#include "protoCol.h"
#include "protoDev.h"
#include "protoDevR05.h"
#include "protoDevR06.h"
#include "myjnia.h"


int main() {
	devArrayInit();
	protoInit();
	InitCli();

	return 0;
}

