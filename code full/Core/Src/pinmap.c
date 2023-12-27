#include "pinmap.h"


float read_res_fpcb(uint16_t *adc, uint32_t scale_value, uint8_t time_test)
{
	  float res;
		switch (scale_value)
		{
			case SCALE_20R :
				FET_20R_ON;
			
				FET_200R_OFF;
				FET_2K_OFF;
				FET_20K_OFF;
				FET_200K_OFF;
				FET_2M_OFF;
			break;
			
			case SCALE_200R :
				FET_200R_ON;
			
				FET_20R_OFF;
				FET_2K_OFF;
				FET_20K_OFF;
				FET_200K_OFF;
				FET_2M_OFF;
			break;
			
			case SCALE_2K :
				FET_2K_ON;
			
				FET_20R_OFF;
				FET_200R_OFF;
				FET_20K_OFF;
				FET_200K_OFF;
				FET_2M_OFF;
			break;
			
			case SCALE_20K :
				FET_20K_ON;
			
				FET_20R_OFF;
				FET_200R_OFF;
				FET_2K_OFF;
				FET_200K_OFF;
				FET_2M_OFF;
			break;
			
			case SCALE_200K :
				FET_200K_ON;
			
				FET_20R_OFF;
				FET_200R_OFF;
				FET_20K_OFF;
				FET_2K_OFF;
				FET_2M_OFF;
			break;
			
			case SCALE_2M :
				FET_2M_ON;
			
				FET_20R_OFF;
				FET_200R_OFF;
				FET_20K_OFF;
				FET_200K_OFF;
				FET_2K_OFF;
			break;
		}
		for (uint8_t i = 0; i < time_test; i ++)
		{
			res += (scale_value * ((3.3* *adc)/4096.0)) / (3.3 - ((3.3* *adc)/4096.0));
		}	
		float res_average = res/time_test;	
    return res_average;
}

void GND_INIT()
{
	CON11_CON12_SPEC;
	CON13_CON14_GND;
}