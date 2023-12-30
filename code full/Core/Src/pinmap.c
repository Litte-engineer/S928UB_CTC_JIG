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

/************ DFF_MAIN ****************/
void DFF_MAIN_1	()	{test_con(U1, HC74_PIN1, HIGH);}
void DFF_MAIN_2	()	{test_con(U1, HC74_PIN2, HIGH);}
void DFF_MAIN_3	()	{test_con(U1, HC74_PIN3, HIGH);}
void DFF_MAIN_4	()	{test_con(U1, HC74_PIN4, HIGH);}
void DFF_MAIN_5	()	{test_con(U1, HC74_PIN5, HIGH);}
void DFF_MAIN_6	()	{test_con(U1, HC74_PIN6, HIGH);}
void DFF_MAIN_7	()	{test_con(U1, HC74_PIN7, HIGH);}
void DFF_MAIN_8	()	{test_con(U1, HC74_PIN8, HIGH);}
void DFF_MAIN_9	()	{test_con(U2, HC74_PIN1, HIGH);}
void DFF_MAIN_10	()	{test_con(U2, HC74_PIN2, HIGH);}
void DFF_MAIN_11	()	{test_con(U2, HC74_PIN3, HIGH);}
void DFF_MAIN_12	()	{test_con(U2, HC74_PIN4, HIGH);}
void DFF_MAIN_13	()	{test_con(U2, HC74_PIN5, HIGH);}
void DFF_MAIN_14	()	{test_con(U2, HC74_PIN6, HIGH);}
void DFF_MAIN_15	()	{test_con(U2, HC74_PIN7, HIGH);}
void DFF_MAIN_16	()	{test_con(U2, HC74_PIN8, HIGH);}
void DFF_MAIN_17	()	{test_con(U3, HC74_PIN1, HIGH);}
void DFF_MAIN_18	()	{test_con(U3, HC74_PIN2, HIGH);}
void DFF_MAIN_19	()	{test_con(U3, HC74_PIN3, HIGH);}
void DFF_MAIN_20	()	{test_con(U3, HC74_PIN4, HIGH);}
void DFF_MAIN_21	()	{test_con(U3, HC74_PIN5, HIGH);}
void DFF_MAIN_22	()	{test_con(U3, HC74_PIN6, HIGH);}
void DFF_MAIN_23	()	{test_con(U3, HC74_PIN7, HIGH);}
void DFF_MAIN_24	()	{test_con(U3, HC74_PIN8, HIGH);}
void DFF_MAIN_25	()	{test_con(U4, HC74_PIN1, HIGH);}
void DFF_MAIN_26	()	{test_con(U4, HC74_PIN2, HIGH);}
void DFF_MAIN_27	()	{test_con(U4, HC74_PIN3, HIGH);}
void DFF_MAIN_28	()	{test_con(U4, HC74_PIN4, HIGH);}
void DFF_MAIN_29	()	{test_con(U4, HC74_PIN5, HIGH);}
void DFF_MAIN_30	()	{test_con(U4, HC74_PIN6, HIGH);}
void DFF_MAIN_31	()	{test_con(U4, HC74_PIN7, HIGH);}
void DFF_MAIN_32	()	{test_con(U4, HC74_PIN8, HIGH);}
void DFF_MAIN_33	()	{test_con(U5, HC74_PIN1, HIGH);}
void DFF_MAIN_34	()	{test_con(U5, HC74_PIN2, HIGH);}
void DFF_MAIN_35	()	{test_con(U5, HC74_PIN3, HIGH);}
void DFF_MAIN_36	()	{test_con(U5, HC74_PIN4, HIGH);}
void DFF_MAIN_37	()	{test_con(U5, HC74_PIN5, HIGH);}
void DFF_MAIN_38	()	{test_con(U5, HC74_PIN6, HIGH);}
void DFF_MAIN_39	()	{test_con(U5, HC74_PIN7, HIGH);}
void DFF_MAIN_40	()	{test_con(U5, HC74_PIN8, HIGH);}

void DFF_MAIN_41	()	{test_con(U5, HC74_PIN1, HIGH);}
void DFF_MAIN_42	()	{test_con(U5, HC74_PIN2, HIGH);}
void DFF_MAIN_43	()	{test_con(U5, HC74_PIN3, HIGH);}
void DFF_MAIN_44	()	{test_con(U5, HC74_PIN4, HIGH);}
void DFF_MAIN_45	()	{test_con(U5, HC74_PIN5, HIGH);}
void DFF_MAIN_46	()	{test_con(U5, HC74_PIN6, HIGH);}
void DFF_MAIN_47	()	{test_con(U5, HC74_PIN7, HIGH);}
void DFF_MAIN_48	()	{test_con(U5, HC74_PIN8, HIGH);}
void DFF_MAIN_49	()	{test_con(U6, HC74_PIN1, HIGH);}
void DFF_MAIN_50	()	{test_con(U6, HC74_PIN2, HIGH);}
void DFF_MAIN_51	()	{test_con(U6, HC74_PIN3, HIGH);}
void DFF_MAIN_52	()	{test_con(U6, HC74_PIN4, HIGH);}
void DFF_MAIN_53	()	{test_con(U6, HC74_PIN5, HIGH);}
void DFF_MAIN_54	()	{test_con(U6, HC74_PIN6, HIGH);}
void DFF_MAIN_55	()	{test_con(U6, HC74_PIN7, HIGH);}
void DFF_MAIN_56	()	{test_con(U6, HC74_PIN8, HIGH);}
void DFF_MAIN_57	()	{test_con(U7, HC74_PIN1, HIGH);}
void DFF_MAIN_58	()	{test_con(U7, HC74_PIN2, HIGH);}
void DFF_MAIN_59	()	{test_con(U7, HC74_PIN3, HIGH);}
void DFF_MAIN_60	()	{test_con(U7, HC74_PIN4, HIGH);}

/******** MAIN GND  **********************************/
void 	GND_MAIN_1	()	{test_con(U1, HC74_PIN1, LOW);}
void 	GND_MAIN_2	()	{test_con(U1, HC74_PIN2, LOW);}
void 	GND_MAIN_3	()	{test_con(U1, HC74_PIN3, LOW);}
void 	GND_MAIN_4	()	{test_con(U1, HC74_PIN4, LOW);}
void 	GND_MAIN_5	()	{test_con(U1, HC74_PIN5, LOW);}
void 	GND_MAIN_6	()	{test_con(U1, HC74_PIN6, LOW);}
void 	GND_MAIN_7	()	{test_con(U1, HC74_PIN7, LOW);}
void 	GND_MAIN_8	()	{test_con(U1, HC74_PIN8, LOW);}
void 	GND_MAIN_9	()	{test_con(U2, HC74_PIN1, LOW);}
void 	GND_MAIN_10	()	{test_con(U2, HC74_PIN2, LOW);}
void 	GND_MAIN_11	()	{test_con(U2, HC74_PIN3, LOW);}
void 	GND_MAIN_12	()	{test_con(U2, HC74_PIN4, LOW);}
void 	GND_MAIN_13	()	{test_con(U2, HC74_PIN5, LOW);}
void 	GND_MAIN_14	()	{test_con(U2, HC74_PIN6, LOW);}
void 	GND_MAIN_15	()	{test_con(U2, HC74_PIN7, LOW);}
void 	GND_MAIN_16	()	{test_con(U2, HC74_PIN8, LOW);}
void 	GND_MAIN_17	()	{test_con(U3, HC74_PIN1, LOW);}
void 	GND_MAIN_18	()	{test_con(U3, HC74_PIN2, LOW);}
void 	GND_MAIN_19	()	{test_con(U3, HC74_PIN3, LOW);}
void 	GND_MAIN_20	()	{test_con(U3, HC74_PIN4, LOW);}
void 	GND_MAIN_21	()	{test_con(U3, HC74_PIN5, LOW);}
void 	GND_MAIN_22	()	{test_con(U3, HC74_PIN6, LOW);}
void 	GND_MAIN_23	()	{test_con(U3, HC74_PIN7, LOW);}
void 	GND_MAIN_24	()	{test_con(U3, HC74_PIN8, LOW);}
void 	GND_MAIN_25	()	{test_con(U4, HC74_PIN1, LOW);}
void 	GND_MAIN_26	()	{test_con(U4, HC74_PIN1, LOW);}
void 	GND_MAIN_27	()	{test_con(U4, HC74_PIN2, LOW);}
void 	GND_MAIN_28	()	{test_con(U4, HC74_PIN3, LOW);}
void 	GND_MAIN_29	()	{test_con(U4, HC74_PIN4, LOW);}
void 	GND_MAIN_30	()	{test_con(U4, HC74_PIN5, LOW);}
void 	GND_MAIN_31	()	{test_con(U4, HC74_PIN6, LOW);}
void 	GND_MAIN_32	()	{test_con(U4, HC74_PIN7, LOW);}
void 	GND_MAIN_33	()	{test_con(U5, HC74_PIN1, LOW);}
void 	GND_MAIN_34	()	{test_con(U5, HC74_PIN2, LOW);}
void 	GND_MAIN_35	()	{test_con(U5, HC74_PIN3, LOW);}
void 	GND_MAIN_36	()	{test_con(U5, HC74_PIN4, LOW);}
void 	GND_MAIN_37	()	{test_con(U5, HC74_PIN5, LOW);}
void 	GND_MAIN_38	()	{test_con(U5, HC74_PIN6, LOW);}
void 	GND_MAIN_39	()	{test_con(U5, HC74_PIN7, LOW);}
void 	GND_MAIN_40	()	{test_con(U5, HC74_PIN8, LOW);}
void 	GND_MAIN_41	()	{test_con(U6, HC74_PIN1, LOW);}
void 	GND_MAIN_42	()	{test_con(U6, HC74_PIN2, LOW);}
void 	GND_MAIN_43	()	{test_con(U6, HC74_PIN3, LOW);}
void 	GND_MAIN_44	()	{test_con(U6, HC74_PIN4, LOW);}
void 	GND_MAIN_45	()	{test_con(U6, HC74_PIN5, LOW);}
void 	GND_MAIN_46	()	{test_con(U6, HC74_PIN6, LOW);}
void 	GND_MAIN_47	()	{test_con(U6, HC74_PIN7, LOW);}
void 	GND_MAIN_48	()	{test_con(U6, HC74_PIN8, LOW);}
void 	GND_MAIN_49	()	{test_con(U7, HC74_PIN1, LOW);}
void 	GND_MAIN_50	()	{test_con(U7, HC74_PIN2, LOW);}
void 	GND_MAIN_51	()	{test_con(U7, HC74_PIN3, LOW);}
void 	GND_MAIN_52	()	{test_con(U7, HC74_PIN4, LOW);}
void 	GND_MAIN_53	()	{test_con(U7, HC74_PIN5, LOW);}
void 	GND_MAIN_54	()	{test_con(U7, HC74_PIN6, LOW);}
void 	GND_MAIN_55	()	{test_con(U7, HC74_PIN7, LOW);}
void 	GND_MAIN_56	()	{test_con(U7, HC74_PIN8, LOW);}
void 	GND_MAIN_57	()	{test_con(U8, HC74_PIN1, LOW);}
void 	GND_MAIN_58	()	{test_con(U8, HC74_PIN2, LOW);}
void 	GND_MAIN_59	()	{test_con(U8, HC74_PIN3, LOW);}
void 	GND_MAIN_60	()	{test_con(U8, HC74_PIN4, LOW);}

/************ DFF_SUB ********************************/
void 	DFF_SUB_1	()	{test_con(U12, HC74_PIN1, HIGH);}
void 	DFF_SUB_2	()	{test_con(U12, HC74_PIN2, HIGH);}
void 	DFF_SUB_3	()	{test_con(U12, HC74_PIN3, HIGH);}
void 	DFF_SUB_4	()	{test_con(U12, HC74_PIN4, HIGH);}
void 	DFF_SUB_5	()	{test_con(U12, HC74_PIN5, HIGH);}
void 	DFF_SUB_6	()	{test_con(U12, HC74_PIN6, HIGH);}
void 	DFF_SUB_7	()	{test_con(U12, HC74_PIN7, HIGH);}
void 	DFF_SUB_8	()	{test_con(U12, HC74_PIN8, HIGH);}
void 	DFF_SUB_9	()	{test_con(U9, HC74_PIN1, HIGH);}
void 	DFF_SUB_10	()	{test_con(U9, HC74_PIN2, HIGH);}
void 	DFF_SUB_11	()	{test_con(U9, HC74_PIN3, HIGH);}
void 	DFF_SUB_12	()	{test_con(U9, HC74_PIN4, HIGH);}
void 	DFF_SUB_13	()	{test_con(U9, HC74_PIN5, HIGH);}
void 	DFF_SUB_14	()	{test_con(U9, HC74_PIN6, HIGH);}
void 	DFF_SUB_15	()	{test_con(U9, HC74_PIN7, HIGH);}
void 	DFF_SUB_16	()	{test_con(U9, HC74_PIN8, HIGH);}
void 	DFF_SUB_17	()	{test_con(U14, HC74_PIN1, HIGH);}
void 	DFF_SUB_18	()	{test_con(U14, HC74_PIN2, HIGH);}
void 	DFF_SUB_19	()	{test_con(U14, HC74_PIN3, HIGH);}
void 	DFF_SUB_20	()	{test_con(U14, HC74_PIN4, HIGH);}
void 	DFF_SUB_21	()	{test_con(U14, HC74_PIN5, HIGH);}
void 	DFF_SUB_22	()	{test_con(U14, HC74_PIN6, HIGH);}
void 	DFF_SUB_23	()	{test_con(U14, HC74_PIN7, HIGH);}
void 	DFF_SUB_24	()	{test_con(U14, HC74_PIN8, HIGH);}
void 	DFF_SUB_25	()	{test_con(U15, HC74_PIN1, HIGH);}
void 	DFF_SUB_26	()	{test_con(U15, HC74_PIN2, HIGH);}
void 	DFF_SUB_27	()	{test_con(U15, HC74_PIN3, HIGH);}
void 	DFF_SUB_28	()	{test_con(U15, HC74_PIN4, HIGH);}
void 	DFF_SUB_29	()	{test_con(U15, HC74_PIN5, HIGH);}
void 	DFF_SUB_30	()	{test_con(U15, HC74_PIN6, HIGH);}
void 	DFF_SUB_31	()	{test_con(U15, HC74_PIN7, HIGH);}
void 	DFF_SUB_32	()	{test_con(U15, HC74_PIN8, HIGH);}
void 	DFF_SUB_33	()	{test_con(U16, HC74_PIN1, HIGH);}
void 	DFF_SUB_34	()	{test_con(U16, HC74_PIN2, HIGH);}
void 	DFF_SUB_35	()	{test_con(U16, HC74_PIN3, HIGH);}
void 	DFF_SUB_36	()	{test_con(U16, HC74_PIN4, HIGH);}
void 	DFF_SUB_37	()	{test_con(U16, HC74_PIN5, HIGH);}
void 	DFF_SUB_38	()	{test_con(U16, HC74_PIN6, HIGH);}
void 	DFF_SUB_39	()	{test_con(U16, HC74_PIN7, HIGH);}
void 	DFF_SUB_40	()	{test_con(U16, HC74_PIN8, HIGH);}

void 	DFF_SUB_41	()	{test_con(U13, HC74_PIN1, HIGH);}
void 	DFF_SUB_42	()	{test_con(U13, HC74_PIN2, HIGH);}
void 	DFF_SUB_43	()	{test_con(U13, HC74_PIN3, HIGH);}
void 	DFF_SUB_44	()	{test_con(U13, HC74_PIN4, HIGH);}
void 	DFF_SUB_45	()	{test_con(U13, HC74_PIN5, HIGH);}
void 	DFF_SUB_46	()	{test_con(U13, HC74_PIN6, HIGH);}
void 	DFF_SUB_47	()	{test_con(U13, HC74_PIN7, HIGH);}
void 	DFF_SUB_48	()	{test_con(U13, HC74_PIN8, HIGH);}
void 	DFF_SUB_49	()	{test_con(U18, HC74_PIN1, HIGH);}
void 	DFF_SUB_50	()	{test_con(U18, HC74_PIN2, HIGH);}
void 	DFF_SUB_51	()	{test_con(U18, HC74_PIN3, HIGH);}
void 	DFF_SUB_52	()	{test_con(U18, HC74_PIN4, HIGH);}
void 	DFF_SUB_53	()	{test_con(U18, HC74_PIN5, HIGH);}
void 	DFF_SUB_54	()	{test_con(U18, HC74_PIN6, HIGH);}
void 	DFF_SUB_55	()	{test_con(U18, HC74_PIN7, HIGH);}
void 	DFF_SUB_56	()	{test_con(U18, HC74_PIN8, HIGH);}
void 	DFF_SUB_57	()	{test_con(U19, HC74_PIN1, HIGH);}
void 	DFF_SUB_58	()	{test_con(U19, HC74_PIN2, HIGH);}
void 	DFF_SUB_59	()	{test_con(U19, HC74_PIN3, HIGH);}
void 	DFF_SUB_60	()	{test_con(U19, HC74_PIN4, HIGH);}


/************ GND_SUB ********************************/
void 	GND_SUB_1	()	{test_con(U12, HC74_PIN1, LOW);}
void 	GND_SUB_2	()	{test_con(U12, HC74_PIN2, LOW);}
void 	GND_SUB_3	()	{test_con(U12, HC74_PIN3, LOW);}
void 	GND_SUB_4	()	{test_con(U12, HC74_PIN4, LOW);}
void 	GND_SUB_5	()	{test_con(U12, HC74_PIN5, LOW);}
void 	GND_SUB_6	()	{test_con(U12, HC74_PIN6, LOW);}
void 	GND_SUB_7	()	{test_con(U12, HC74_PIN7, LOW);}
void 	GND_SUB_8	()	{test_con(U12, HC74_PIN8, LOW);}
void 	GND_SUB_9	()	{test_con(U9, HC74_PIN1,   LOW);}
void 	GND_SUB_10	()	{test_con(U9, HC74_PIN2, LOW);}
void 	GND_SUB_11	()	{test_con(U9, HC74_PIN3, LOW);}
void 	GND_SUB_12	()	{test_con(U9, HC74_PIN4, LOW);}
void 	GND_SUB_13	()	{test_con(U9, HC74_PIN5, LOW);}
void 	GND_SUB_14	()	{test_con(U9, HC74_PIN6, LOW);}
void 	GND_SUB_15	()	{test_con(U9, HC74_PIN7, LOW);}
void 	GND_SUB_16	()	{test_con(U9, HC74_PIN8, LOW);}
void 	GND_SUB_17	()	{test_con(U14, HC74_PIN1, LOW);}
void 	GND_SUB_18	()	{test_con(U14, HC74_PIN2, LOW);}
void 	GND_SUB_19	()	{test_con(U14, HC74_PIN3, LOW);}
void 	GND_SUB_20	()	{test_con(U14, HC74_PIN4, LOW);}
void 	GND_SUB_21	()	{test_con(U14, HC74_PIN5, LOW);}
void 	GND_SUB_22	()	{test_con(U14, HC74_PIN6, LOW);}
void 	GND_SUB_23	()	{test_con(U14, HC74_PIN7, LOW);}
void 	GND_SUB_24	()	{test_con(U14, HC74_PIN8, LOW);}
void 	GND_SUB_25	()	{test_con(U15, HC74_PIN1, LOW);}
void 	GND_SUB_26	()	{test_con(U15, HC74_PIN2, LOW);}
void 	GND_SUB_27	()	{test_con(U15, HC74_PIN3, LOW);}
void 	GND_SUB_28	()	{test_con(U15, HC74_PIN4, LOW);}
void 	GND_SUB_29	()	{test_con(U15, HC74_PIN5, LOW);}
void 	GND_SUB_30	()	{test_con(U15, HC74_PIN6, LOW);}
void 	GND_SUB_31	()	{test_con(U15, HC74_PIN7, LOW);}
void 	GND_SUB_32	()	{test_con(U15, HC74_PIN8, LOW);}
void 	GND_SUB_33	()	{test_con(U16, HC74_PIN1, LOW);}
void 	GND_SUB_34	()	{test_con(U16, HC74_PIN2, LOW);}
void 	GND_SUB_35	()	{test_con(U16, HC74_PIN3, LOW);}
void 	GND_SUB_36	()	{test_con(U16, HC74_PIN4, LOW);}
void 	GND_SUB_37	()	{test_con(U16, HC74_PIN5, LOW);}
void 	GND_SUB_38	()	{test_con(U16, HC74_PIN6, LOW);}
void 	GND_SUB_39	()	{test_con(U16, HC74_PIN7, LOW);}
void 	GND_SUB_40	()	{test_con(U16, HC74_PIN8, LOW);}
void 	GND_SUB_41	()	{test_con(U13, HC74_PIN1, LOW);}
void 	GND_SUB_42	()	{test_con(U13, HC74_PIN2, LOW);}
void 	GND_SUB_43	()	{test_con(U13, HC74_PIN3, LOW);}
void 	GND_SUB_44	()	{test_con(U13, HC74_PIN4, LOW);}
void 	GND_SUB_45	()	{test_con(U13, HC74_PIN5, LOW);}
void 	GND_SUB_46	()	{test_con(U13, HC74_PIN6, LOW);}
void 	GND_SUB_47	()	{test_con(U13, HC74_PIN7, LOW);}
void 	GND_SUB_48	()	{test_con(U13, HC74_PIN8, LOW);}
void 	GND_SUB_49	()	{test_con(U18, HC74_PIN1, LOW);}
void 	GND_SUB_50	()	{test_con(U18, HC74_PIN2, LOW);}
void 	GND_SUB_51	()	{test_con(U18, HC74_PIN3, LOW);}
void 	GND_SUB_52	()	{test_con(U18, HC74_PIN4, LOW);}
void 	GND_SUB_53	()	{test_con(U18, HC74_PIN5, LOW);}
void 	GND_SUB_54	()	{test_con(U18, HC74_PIN6, LOW);}
void 	GND_SUB_55	()	{test_con(U18, HC74_PIN7, LOW);}
void 	GND_SUB_56	()	{test_con(U18, HC74_PIN8, LOW);}
void 	GND_SUB_57	()	{test_con(U19, HC74_PIN1, LOW);}
void 	GND_SUB_58	()	{test_con(U19, HC74_PIN2, LOW);}
void 	GND_SUB_59	()	{test_con(U19, HC74_PIN3, LOW);}
void 	GND_SUB_60	()	{test_con(U19, HC74_PIN4, LOW);}


