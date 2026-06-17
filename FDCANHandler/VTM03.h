#ifndef VTM03_H
#define VTM03_H

#include "string.h"
#include "usart.h"
#include "stdbool.h"
#include "Double_Buffer.h"
#include "FDCANHandler.h"
#include "crc.h"

#define VTM_Overtime 30

typedef struct 
{
 	int16_t rc_ch0;   			 /*ÓÒÒ¡¸ËXÖá*/
 	int16_t rc_ch1;   			 /*ÓÒÒ¡¸ËYÖá*/
 	int16_t rc_ch2;   			 /*×óÒ¡¸ËXÖá*/
 	int16_t rc_ch3;   			 /*×óÒ¡¸ËYÖá*/
 	uint8_t rc_s1;    			 /*µ²Î»ÇĞ»»¿ª¹Ø*/
	bool		rc_stop;	 			 /*ÔİÍ£°´¼ü*/
	bool		rc_left_key;		 /*×ó²à°´¼ü*/
	bool		rc_right_key;		 /*ÓÒ²à°´¼ü*/
	bool		rc_trigger;			 /*°â»ú¼ü*/
	int16_t     rc_pause_button;
	int16_t     mouse_x;				 /*Êó±êxÖáËÙ¶È*/
	int16_t     mouse_y;				 /*Êó±êyÖáËÙ¶È*/
	int16_t     mouse_z;				 /*Êó±ê¹öÂÖËÙ¶È*/
	bool		mouse_left;
	bool		mouse_right;
	bool		mouse_mid;
	bool		kb_w;
	bool		kb_s;
	bool		kb_a;
	bool		kb_d;
	bool		kb_shift;
	bool		kb_ctrl;
	bool		kb_q;
	bool		kb_e;
	bool		kb_r;
	bool		kb_f;
	bool		kb_g;
	bool		kb_z;
	bool		kb_x;
	bool     kb_c;
	bool		kb_v;
	bool		kb_b;
}VTM_Data_t;

typedef struct 
{
	bool last_rc_stop_status;
	bool last_rc_left_key_status;
	bool last_rc_right_key_status;
	bool last_rc_trigger_status;
	bool last_mouse_left_status;
	bool last_mouse_right_status;
	bool last_mouse_mid_status;
	bool last_kb_w_status;
	bool last_kb_s_status;
	bool last_kb_a_status;
	bool last_kb_d_status;
	bool last_kb_shift_status;
	bool last_kb_ctrl_status;
	bool last_kb_q_status;
	bool last_kb_e_status;
	bool last_kb_r_status;
	bool last_kb_f_status;
	bool last_kb_g_status;
	bool last_kb_z_status;
	bool last_kb_x_status;
	bool last_kb_c_status;
	bool last_kb_v_status;
	bool last_kb_b_status;
}VTM_Data_LastStatus_t;

typedef struct 
{
	bool rc_stop_flag;
	bool rc_left_key_flag;
	bool rc_right_key_flag;
	bool rc_trigger_flag;
	bool mouse_left_flag;
	bool mouse_right_flag;
	bool mouse_mid_flag;
	bool kb_w_flag;
	bool kb_s_flag;
	bool kb_a_flag;
	bool kb_d_flag;
	bool kb_shift_flag;
	bool kb_ctrl_flag;
	bool kb_q_flag;
	bool kb_e_flag;
	bool kb_r_flag;
	bool kb_f_flag;
	bool kb_g_flag;
	bool kb_z_flag;
	bool kb_x_flag;
	bool kb_c_flag;
	bool kb_v_flag;
	bool kb_b_flag;
}VTM_Data_Flag_t;

typedef struct 
{
	int16_t xmag;
	int16_t ymag;
	int16_t wmag;
}Chassis_Mag;

typedef struct
{
	VTM_Data_t data;
	VTM_Data_LastStatus_t data_last_status;
	VTM_Data_Flag_t data_flag;
	Chassis_Mag mag;
}VTM_Handle_t;

bool VTM_Handle_Init(VTM_Handle_t *vt);
bool VTM_Recv_Process(VTM_Handle_t *vt,uint8_t *FDCAN_BUF);

#endif