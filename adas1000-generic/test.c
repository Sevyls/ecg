
#include "ADAS1000.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main() {
	printf("Main() Start\n");
	unsigned char result = ADAS1000_Init(ADAS1000_2KHZ_FRAME_RATE);
	unsigned long val = 1ul;
	
	if (result == 1) {
		printf("Result = 1: initialization was successful\n");
	} else {
		printf("Result = 0: initialization was not successful\n");
		return 1;
	}

	// Example 1

	// Write 1
	unsigned long cmrefctl = ADAS1000_CMREFCTL_LACM | ADAS1000_CMREFCTL_LLCM | ADAS1000_CMREFCTL_RACM;
	printf("0x%lx\n", cmrefctl);
	ADAS1000_SetRegisterValue(ADAS1000_CMREFCTL, cmrefctl);
	
	// Write 2
	unsigned long frmctl = (1ul << 18) | (1ul << 17) | (1ul << 16) | (1ul << 15)  ;
	printf("0x%lx\n", frmctl);
	ADAS1000_SetRegisterValue(ADAS1000_FRMCTL, frmctl);

	// Write 3
 	unsigned long ecgctl = ADAS1000_ECGCTL_LAEN | ADAS1000_ECGCTL_LLEN | ADAS1000_ECGCTL_RAEN | ADAS1000_ECGCTL_VREFBUF | ADAS1000_ECGCTL_PWREN;
	printf("0x%08lx\n", ecgctl);
	ADAS1000_SetRegisterValue(ADAS1000_ECGCTL, ecgctl);
	ADAS1000_SetRegisterValue(ADAS1000_NOP, 0ul);
	
	while(1) {
		ADAS1000_GetRegisterValue(ADAS1000_FRAMES, &val);
		if ((val & (1ul << 31)) == (1ul << 31)) {
			printf("FRAME HEADER: 0x%08lx\n", val);
			exit(0);
		} else {
			printf("FRAME DATA:   0x%08lx\n", val);
		}	 
	}

	printf("Main() End\n");
}
