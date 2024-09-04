/*
 * registers.c
 *
 *	Library for the EiceDRIVER APD 2ED4820-EM 48 V smart high-side MOSFET gate driver.
 *	Created by Rene Santeler @ MCI EAL 2022 (based on sample code from schwarzg created on 4 Mar 2020)
 */


#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <DRIVER_2ED4820EM/Driver_2ED4820.h>
#include <DRIVER_2ED4820EM/registers.h>
#include <DRIVER_2ED4820EM/target.h>
#include <stdlib.h> // includes strtol

//****************************************************************************
// global buffers for register data
//****************************************************************************
static uint8_t g_reg_data[11];         // cache for 2ED4820 register data

//****************************************************************************
// global variables for register and bit names
//****************************************************************************
const char reg_name[11][7]={
        "STDIAG",
        "CHDIAG",
        "DIAG",
        "MOSCTR",
        "FCLEAN",
        "VDSTH",
        "FLTBLK",
        "CSAOCT",
        "VBRES",
        "RESETS",
        "SPARE"
};

//
// [11 registers][8 bit data][10 char. max]
// bit names are shortened vs. specification
// for space reasons and to make it easier to read
//
const char bit_name[11][8][10]={
        { //STDIAG
                "VBATOV",    // LSB : VBAT undervoltage failure (not latched)
                "VBATUV",    // VBAT overvoltage failure (not latched)
                "VDDUV",     // VDD undervoltage propagated in the first command and indicates an under voltage event
                "TSD",       // Chip in Overtemperature (latched)
                "OTWARN",    // Temperature warning
                "MEM",       // Error in the memory, trimming not possible (flag cannot be cleaned)
                "VCPRDY",    // Charge pump is ready
                "FAIL"       // MSB : Main Failure indication, 1 when there is a failure
        },
        { //CHDIAG
                "VSOVA",     // LSB : Source overvoltage - Channel_A
                "VDSOVA",    // Drain to Source overvoltage - Channel_A (latched)
                "VGSUVA",    // Gate-Source undervoltage - Channel_A (latched)
                "VSOVB",     // Source overvoltage - Channel_B
                "VDSOVB",    // Drain to Source overvoltage - Channel_B (latched)
                "VGSUVB",    // Gate-Source undervoltage - Channel_B (latched)
                "OC",        // Overcurrent failure (latched)
                "VCPUV"      // MSB : Charge pump undervoltage
        },
        { //DIAG
                "SST_EN",    // LSB : SAFESTATEN
                "ADD_NA",    // Address not available
                "LOG_CP",    // Loss of charge pump ground
                "LOG_D",     // Loss of digital ground
                "LOG_A",     // Loss of analog ground
                "D5",        // Not assigned
                "D6",        // Not assigned
                "D7"         // Not assigned
        },
        { //MOS_CHS_CTRL
                "CHA_ON",    // LSB : Switch on channel A
                "VDSCLA",    // Clear VDS flag Channel A
                "VGSCLA",    // Clear VGS flag Channel A
                "CHB_ON",    // Switch on channel B
                "VDSCLB",    // Clear VDS flag Channel B
                "VGSCLB",    // Clear VGS flag Channel B
                "ITRPCL",    // Clear ITRIP flag
                "XCTRL"      // MSB : Channel cross control activation
        },
        {  // FAILURE_CLEAN
                "VBUVCL",    // LSB : Clear VBAT undervoltage failure
                "VBOVCL",    // Clear VBAT overvoltage failure
                "VDDUCL",    // Clear Vdd undervoltage failure
                "TSD_CL",     // Clear overtemperature failure
                "INT_CL",     // Clear interrupt
                "SST_CL",     // Safe State clear
                "D6",        // Not assigned
                "CPUVCL"     // MSB : VCP under voltage clear
        },
        { // VDSTHA_B
                "VDS_A0",     // LSB : Drain-Source overvoltage threshold Channel_A
                "VDS_A1",     // Drain-Source overvoltage threshold Channel_A
                "VDS_A2",     // Drain-Source overvoltage threshold Channel_A
                "SST_A",    // VDS Channel A safe state: ON when 0, OFF when 1 (default)
                "VDS_B0",     // Drain-Source overvoltage threshold Channel_B
                "VDS_B1",     // Drain-Source overvoltage threshold Channel_B
                "VDS_B2",     // Drain-Source overvoltage threshold Channel_B
                "SST_B"     // MSB : VDS Channel B safe state: ON when 0, OFF when 1 (default)
        },
        { // MOSFLTBLKA_B
                "FLT_A0",     // LSB : MOS voltage filter Channel_A
                "FLT_A1",     // MOS voltage filter Channel_A
                "BLK_A0",     // MOS voltage blank time Channel_A
                "BLK_A1",     // MOS voltage blank time Channel_A
                "FLT_B0",     // MOS voltage filter Channel_B
                "FLT_B1",     // MOS voltage filter Channel_B
                "BLK_B0",     // MOS voltage blank time Channel_B
                "BLK_B1"      // MSB : MOS voltage blank time Channel_B
        },
        { // CSAG_OCTH
                "CSAG0",     // LSB : Current sense amplifier gain GDIFF
                "CSAG1",     // Current sense amplifier gain GDIFF
                "CSAG2",     // Current sense amplifier gain GDIFF
                "OCTH0",     // Over current detection thresholds
                "OCTH1",     // Over current detection thresholds
                "HSS",       // Control signal to select the CSA configuration LSS or HSS(or bidirectional)
                "COUT",      // Configures opamp output stage vs load cap
                "D7"         // MSB : Not assigned
        },
        { // VBATOVOVRST
                "OVRT0",     // LSB : VBAT overvoltage auto-restart time
                "OVRT1",     // VBAT overvoltage auto-restart time
                "UVRT0",     // VBAT undervoltage auto-restart time
                "UVRT1",     // VBAT undervoltage auto-restart time
                "D4",        // Not assigned
                "D5",        // Not assigned
                "D6",        // Not assigned
                "D7"         // Not assigned
        },
        {
                "SRES0",     // LSB : Software Reset 0
                "FRES0",     // Reset Fails registers 0
                "GENINT",    // Generate interrupt signal
                "D3",        // Not assigned
                "SRES1",     // Software Reset 1
                "FRES1",     // Reset Fails registers 1
                "D6",        // Not assigned
                "D7"         // Not assigned
        },
        {
                "SPARE0",    // LSB : Spare register bit 0
                "SPARE1",    // Spare register bit 1
                "SPARE2",    // Spare register bit 2
                "SPARE3",    // Spare register bit 3
                "SPARE4",    // Spare register bit 4
                "SPARE5",    // Spare register bit 5
                "SPARE6",    // Spare register bit 6
                "SPARE7"     // MSB : Spare register bit 7
        }

};



//****************************************************************************
// xtoi
//
// Returns int value for hexadecimal string
//
// Copyright (c) Leigh Brasington 2012.  http://www.leighb.com/xtoi.htm
//****************************************************************************
int xtoi(char *hexstring)
{
    int i = 0;
    char c = 0;

    // get rid of '0x'
    if ((*hexstring == '0') && (*(hexstring+1) == 'x'))
        hexstring += 2;

    while(*hexstring)
    {
        c = toupper((unsigned char)*hexstring++);
        if ((c < '0') || (c > 'F') || ((c > '9') && (c < 'A')))
            break;
        c -= '0';
        if (c > 9)
            c -= 7;
        i = (i << 4) + c;
    }
    return i;
}


//****************************************************************************
// getRegisterName
//****************************************************************************
const char* getRegisterName(uint8_t reg_num)
{
    if(reg_num >= 0 && reg_num <= 10)
    {
        return reg_name[reg_num];
    }
    else
    {
        return "[ERROR]: Invalid Register Number";
    }
}



//****************************************************************************
// setRegisterCacheData
//****************************************************************************
uint8_t setRegisterCacheData(uint8_t reg_num, uint8_t reg_data)
{
    if(reg_num >= 0 && reg_num <= 10)
    {
        g_reg_data[reg_num] = reg_data;
        return 1;
    }
    else
    {
        return 0;
    }
}

//****************************************************************************
// getRegisterCacheData
//****************************************************************************
uint8_t getRegisterCacheData(uint8_t reg_num)
{
    if(reg_num >= 0 && reg_num <= 10)
    {
        return g_reg_data[reg_num];
    }
    else
    {
        return 0;
    }
}


//****************************************************************************
// getBitName
//****************************************************************************
const char* getBitName(uint8_t reg_num, uint8_t bit_num)
{

    if(reg_num >= 0 && reg_num <= 10)
    {
        if(bit_num >= 0 && bit_num <= 7)
        {
            return bit_name[reg_num][bit_num];
        }
        else
        {
            return "[ERROR]: Invalid bit number";
        }
    }
    else
    {
        return "[ERROR]: Invalid register number";
    }
}


//****************************************************************************
// setBits
//
// sets the bits in given register and writes register
// (the used register must be read to cache before this!)
//
// bitIsLatched - if 1 data is written back to cache
//****************************************************************************
void setBits(uint8_t reg_num, uint8_t bitmask, uint8_t bitIsLatched)
{
    uint8_t data;

    if(isEnabled()){
    	data = g_reg_data[reg_num]; // get current register data
    	data |= bitmask;            // set bits

    	spiWrite(reg_num, data);    // write new register data

    	// if bit stays after set (latched) also write to cache (prevent overwrite if no read is performed between function calls)
		if(bitIsLatched)
			setRegisterCacheData(reg_num, data);
    }
    else {
		//DEBUGPRINTF("[ERROR]: Driver is disabled");
		//XMC_DEBUG("[ERROR]: Driver is disabled");
	}

    //DEBUGPRINTF("SetBits:   reg %d, bitmask %d, data %d (EN=%d)\r\n", reg_num, bitmask, data, isEnabled());
}

//****************************************************************************
// clearBits
//
// clears the bits in given register and writes register
// (the used register must be read to cache before this!)
//
// bitIsLatched - if 1 data is written back to cache
//****************************************************************************
void clearBits(uint8_t reg_num, uint8_t bitmask, uint8_t bitIsLatched)
{
    uint8_t data;

    if(isEnabled()){
		data = g_reg_data[reg_num]; // get current register data
		data &= ~bitmask;           // clear bits

     	spiWrite(reg_num, data);    // write new register data

     	// if bit stays after set (latched) also write to cache (prevent overwrite if no read is performed between function calls)
     	if(bitIsLatched)
     		setRegisterCacheData(reg_num, data);
    }
	else {
		//DEBUGPRINTF("[ERROR]: Driver is disabled");
		//XMC_DEBUG("[ERROR]: Driver is disabled");
	}

    //DEBUGPRINTF("ClearBits: reg %d, bitmask %d, data %d\r\n", reg_num, bitmask, data);
}


//****************************************************************************
// readRegister
//****************************************************************************
void readRegister(uint8_t reg_num)
{
	uint8_t rx_data; // received data

    // print only if driver is enabled (ENABLE pin high)
    if(isEnabled())
    {
		if(reg_num >= 0 && reg_num <= 10) {
			spiRead(reg_num, &rx_data);
			spiRead(reg_num, &rx_data);

			//XMC_DEBUG("register number %2d has data %2d. \n",reg_num,rx_data);
		}
		else {
			//DEBUGPRINTF("[ERROR]: Invalid register number");
			//XMC_DEBUG("[ERROR]: Invalid register number");
		}
    }
    else {
        //DEBUGPRINTF("[ERROR]: Can't read register - driver is disabled");
        //XMC_DEBUG("[ERROR]: Can't read register - driver is disabled");
    }

}

//****************************************************************************
// readAllRegisters
//
// reads all registers and stores data in global variables
//****************************************************************************
void readAllRegisters(void)
{
    uint8_t reg_num; // register number
    uint8_t rx_data; // received data

    //uint8_t reg_datax;


    // read only if driver is enabled (ENABLE pin high)
    if(isEnabled())
    {
        for(reg_num = 0; reg_num < 11; reg_num++)
        {
            spiRead(reg_num, &rx_data);
            //spiRead(reg_num, &rx_data);

            //skip first response - rx data is response to previous frame
            if (reg_num > 0)
            {

                g_reg_data[reg_num-1] = rx_data; // store data in global variable
            }
        }

        //repeat last frame to get data for last register (Reg. #10)
        spiRead(10, &rx_data);
        g_reg_data[10] = rx_data;

       // reg_datax = g_reg_data[0];

        //XMC_DEBUG("\nreg cache data:  %2d \n", reg_datax);

    }
}

//****************************************************************************
// writeRegister
//****************************************************************************
void writeRegister(char *adr, char *dat)
{
    uint8_t reg_num;    // register number
    uint8_t reg_data;       // register data

    // write only if driver is enabled (ENABLE pin high)
    if(isEnabled())
    {
        reg_num = atoi(adr); // convert string to integer

    	//reg_num = *adr;
    	//reg_data = *dat;

        if(reg_num >= 0 && reg_num <= 10)
        {
            // convert data string from hex format to uint8_t
            if (strlen(dat)<=2) // expecting up to 2 hex characters
            {
                reg_data = xtoi(dat);
                spiWrite(reg_num, reg_data);
            }
            else
            {
               // DEBUGPRINTF("\r\n"); // new line
               // DEBUGPRINTF("[ERROR]: expecting max. 2 hex characters (e.g. '2A')");
            }
        }
        else
        {
           // DEBUGPRINTF("\r\n"); // new line
            //DEBUGPRINTF("[ERROR]: Invalid register number");
        }

    }
    else
    {
        //DEBUGPRINTF("\r\n"); // new line
        //DEBUGPRINTF("[ERROR]: Can't write register - driver is disabled");
    }
}


//****************************************************************************
// printRegister
//****************************************************************************
void printRegister(uint8_t reg_num)
{
    int bit_num;
    //const char *reg_name;
    //const char *bit_name;

    if(reg_num >= 0 && reg_num <= 10)
    {
        //reg_name = getRegisterName(reg_num);



        for(bit_num = 7; bit_num >= 0; bit_num--)
        {
           //bit_name = getBitName(reg_num, (uint8_t)bit_num);
           if((g_reg_data[reg_num]) & (1 << bit_num)) // if bit is set (high)
            {

            }
           else // if bit is not set (low)
           {

           }
        }
    }
    else
    {
        //DEBUGPRINTF("\r\n"); // new line
        //DEBUGPRINTF("[ERROR]: Invalid register number");
        //XMC_DEBUG("\r\n");
        //XMC_DEBUG("[ERROR]: Invalid register number");
    }
}



