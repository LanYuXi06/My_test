#include "VTM03.h"
#include "arm_math.h"

#define VTM_Mid_Value	1024

#define Chassis_Mag_Factor 660.0f
#define Shift_Factor 2.0f

static bool VTM_Data_Init(VTM_Data_t *data)
{
	if(data == NULL)return false;
	data->rc_ch0 		  = 0;	
	data->rc_ch1 		  = 0;
	data->rc_ch2 		  = 0;
	data->rc_ch3 		  = 0;
	data->rc_s1 		  = 1;
	data->rc_stop	  	  = false;
	data->rc_left_key  	  = false;
	data->rc_right_key 	  = false;
	data->rc_trigger 	  = false;
	data->rc_pause_button = 0;
	data->mouse_x 		  = 0;
	data->mouse_y 		  = 0;
	data->mouse_z 		  = 0;
	data->mouse_left  	  = false;
	data->mouse_right 	  = false;
	data->mouse_mid 	  = false;
	data->kb_w			  = false;
	data->kb_s			  = false;
	data->kb_a			  = false;
	data->kb_d			  = false;
	data->kb_shift 		  = false;
	data->kb_ctrl 		  = false;
	data->kb_q 			  = false;
	data->kb_e 			  = false;
	data->kb_r 			  = false;
	data->kb_f 			  = false;
	data->kb_g 			  = false;
	data->kb_z 			  = false;
	data->kb_x 			  = false;
	data->kb_c 			  = false;
	data->kb_v 			  = false;
	data->kb_b 			  = false;
	return true;
}

static bool VTM_Data_LastStatus_Init(VTM_Data_LastStatus_t *data_last_status)
{
    if(data_last_status == NULL)return false;
    data_last_status->last_rc_stop_status = false;
    data_last_status->last_rc_left_key_status = false;
    data_last_status->last_rc_right_key_status = false;
    data_last_status->last_rc_trigger_status = false;
    data_last_status->last_mouse_left_status = false;
    data_last_status->last_mouse_right_status = false;
    data_last_status->last_mouse_mid_status = false;
    data_last_status->last_kb_w_status = false;
    data_last_status->last_kb_s_status = false;
    data_last_status->last_kb_a_status = false;
    data_last_status->last_kb_d_status = false;
    data_last_status->last_kb_shift_status = false;
    data_last_status->last_kb_ctrl_status = false;
    data_last_status->last_kb_q_status = false;
    data_last_status->last_kb_e_status = false;
    data_last_status->last_kb_r_status = false;
    data_last_status->last_kb_f_status = false;
    data_last_status->last_kb_g_status = false;
    data_last_status->last_kb_z_status = false;
    data_last_status->last_kb_x_status = false;
    data_last_status->last_kb_c_status = false;
    data_last_status->last_kb_v_status = false;
    data_last_status->last_kb_b_status = false;
    return true;
}

static bool VTM_Data_Flag_Init(VTM_Data_Flag_t *data_flag)
{
    if(data_flag == NULL)return false;
    data_flag->rc_stop_flag = false;
    data_flag->rc_left_key_flag = false;
    data_flag->rc_right_key_flag = false;
    data_flag->rc_trigger_flag = false;
    data_flag->mouse_left_flag = false;
    data_flag->mouse_right_flag = false;
    data_flag->mouse_mid_flag = false;
    data_flag->kb_w_flag = false;
    data_flag->kb_s_flag = false;
    data_flag->kb_a_flag = false;
    data_flag->kb_d_flag = false;
    data_flag->kb_shift_flag = false;
    data_flag->kb_ctrl_flag = false;
    data_flag->kb_q_flag = false;
    data_flag->kb_e_flag = false;
    data_flag->kb_r_flag = false;
    data_flag->kb_f_flag = false;
    data_flag->kb_g_flag = false;
    data_flag->kb_z_flag = false;
    data_flag->kb_x_flag = false;
    data_flag->kb_c_flag = false;
    data_flag->kb_v_flag = false;
    data_flag->kb_b_flag = false;
    return true;
}

static bool Chassis_Mag_Init(Chassis_Mag *mag)
{
    if(!mag) return false;
    mag->wmag = mag->xmag = mag->ymag = 0;
    return true;
}

bool VTM_Handle_Init(VTM_Handle_t *vt)
{
	if(!vt)return false;
	
	if(!VTM_Data_Init(&vt->data) || !VTM_Data_LastStatus_Init(&vt->data_last_status) ||
		 !VTM_Data_Flag_Init(&vt->data_flag) || !Chassis_Mag_Init(&vt->mag)) 
	{	
		return false;
	}else 
	{
		return true;
	}
}

bool VTM_Data_Process(VTM_Data_t *data,uint8_t *FDCAN_BUF)
{
	if(data == NULL || FDCAN_BUF == NULL)return false;

	data->rc_ch0  = (int16_t)(((FDCAN_BUF[0] 	 )|(FDCAN_BUF[1] << 8)) & 0x07FF) - VTM_Mid_Value;
	data->rc_ch1  = (int16_t)(((FDCAN_BUF[1] >> 3 )|(FDCAN_BUF[2] << 5)) & 0x07FF) - VTM_Mid_Value;
	data->rc_ch2  = (int16_t)(((FDCAN_BUF[2] >> 6 )|(FDCAN_BUF[3] << 2) | (FDCAN_BUF[4] << 10)) & 0x07FF) - VTM_Mid_Value;
	data->rc_ch3  = (int16_t)(((FDCAN_BUF[4] >> 1 )|(FDCAN_BUF[5] << 7)) & 0x07FF) - VTM_Mid_Value;
	data->rc_s1   = (FDCAN_BUF[5] >> 4) & 0x03;
	data->rc_stop = (FDCAN_BUF[5] >> 6) & 0x01; 
	data->rc_left_key = (FDCAN_BUF[5] >> 7) & 0x01;
	data->rc_right_key = FDCAN_BUF[6] & 0x01;
	data->rc_pause_button = (int16_t)(((FDCAN_BUF[6] >> 1) | (FDCAN_BUF[7] << 7)) & 0x07FF) - VTM_Mid_Value;
	data->rc_trigger		= (FDCAN_BUF[7] >> 4)&0x01;
	
	data->mouse_x = (int16_t)(FDCAN_BUF[8] | FDCAN_BUF[9] << 8);
	data->mouse_y	= (int16_t)(FDCAN_BUF[10] | FDCAN_BUF[11] << 8);
	data->mouse_z	= (int16_t)(FDCAN_BUF[12] | FDCAN_BUF[13] << 8);
	
	if(data->mouse_x > 9000 || data->mouse_x < -9000) data->mouse_x = 0;
	if(data->mouse_y > 9000 || data->mouse_y < -9000) data->mouse_y = 0;
	if(data->mouse_z > 9000 || data->mouse_z < -9000) data->mouse_z = 0;
	
	data->mouse_left = FDCAN_BUF[14] & 0x03;
	data->mouse_right = (FDCAN_BUF[14] >> 2) & 0x03;
	data->mouse_mid  = (FDCAN_BUF[14] >> 4) & 0x03;
	
	data->kb_w	  = FDCAN_BUF[15] & 0x01;
	data->kb_s	  = (FDCAN_BUF[15] >> 1) & 0x01;
	data->kb_a	  = (FDCAN_BUF[15] >> 2) & 0x01;
	data->kb_d	  = (FDCAN_BUF[15] >> 3) & 0x01;
	data->kb_shift = (FDCAN_BUF[15] >> 4) & 0x01;
	data->kb_ctrl  = (FDCAN_BUF[15] >> 5) & 0x01;
	data->kb_q	  = (FDCAN_BUF[15] >> 6) & 0x01;
	data->kb_e	  = (FDCAN_BUF[15] >> 7) & 0x01;
	data->kb_r	  =  FDCAN_BUF[16] & 0x01;
	data->kb_f	  = (FDCAN_BUF[16] >> 1) & 0x01;
	data->kb_g	  = (FDCAN_BUF[16] >> 2) & 0x01;
	data->kb_z	  = (FDCAN_BUF[16] >> 3) & 0x01;
	data->kb_x	  = (FDCAN_BUF[16] >> 4) & 0x01;
	data->kb_c	  = (FDCAN_BUF[16] >> 5) & 0x01;
	data->kb_v	  = (FDCAN_BUF[16] >> 6) & 0x01;
	data->kb_b	  = (FDCAN_BUF[16] >> 7) & 0x01;
	return true;
}

static inline void flag_process(bool status,bool *last_status,bool *flag)
{
    if(status == true && *last_status == false)
    {
        *flag = !*flag;
        *last_status = true;
    }
    else if(status == false && *last_status == true)
    {
        *last_status = false;
    }else
    {
        return;
    }
}

static bool VTM_Flag_Process(VTM_Handle_t *vt)
{
	if(vt == NULL)return false;
		flag_process(vt->data.rc_stop,&vt->data_last_status.last_rc_stop_status,&vt->data_flag.rc_stop_flag);
    flag_process(vt->data.rc_left_key,&vt->data_last_status.last_rc_left_key_status,&vt->data_flag.rc_left_key_flag);
    flag_process(vt->data.rc_right_key,&vt->data_last_status.last_rc_right_key_status,&vt->data_flag.rc_right_key_flag);
    flag_process(vt->data.rc_trigger,&vt->data_last_status.last_rc_trigger_status,&vt->data_flag.rc_trigger_flag);
    flag_process(vt->data.mouse_left,&vt->data_last_status.last_mouse_left_status,&vt->data_flag.mouse_left_flag);
    flag_process(vt->data.mouse_right,&vt->data_last_status.last_mouse_right_status,&vt->data_flag.mouse_right_flag);
    flag_process(vt->data.mouse_mid,&vt->data_last_status.last_mouse_mid_status,&vt->data_flag.mouse_mid_flag);
    flag_process(vt->data.kb_w,&vt->data_last_status.last_kb_w_status,&vt->data_flag.kb_w_flag);
    flag_process(vt->data.kb_s,&vt->data_last_status.last_kb_s_status,&vt->data_flag.kb_s_flag);
    flag_process(vt->data.kb_a,&vt->data_last_status.last_kb_a_status,&vt->data_flag.kb_a_flag);
    flag_process(vt->data.kb_d,&vt->data_last_status.last_kb_d_status,&vt->data_flag.kb_d_flag);
    flag_process(vt->data.kb_shift,&vt->data_last_status.last_kb_shift_status,&vt->data_flag.kb_shift_flag);
    flag_process(vt->data.kb_ctrl,&vt->data_last_status.last_kb_ctrl_status,&vt->data_flag.kb_ctrl_flag);
    flag_process(vt->data.kb_q,&vt->data_last_status.last_kb_q_status,&vt->data_flag.kb_q_flag);
    flag_process(vt->data.kb_e,&vt->data_last_status.last_kb_e_status,&vt->data_flag.kb_e_flag);
    flag_process(vt->data.kb_r,&vt->data_last_status.last_kb_r_status,&vt->data_flag.kb_r_flag);
    flag_process(vt->data.kb_f,&vt->data_last_status.last_kb_f_status,&vt->data_flag.kb_f_flag);
    flag_process(vt->data.kb_g,&vt->data_last_status.last_kb_g_status,&vt->data_flag.kb_g_flag);
    flag_process(vt->data.kb_z,&vt->data_last_status.last_kb_z_status,&vt->data_flag.kb_z_flag);
    flag_process(vt->data.kb_x,&vt->data_last_status.last_kb_x_status,&vt->data_flag.kb_x_flag);
    flag_process(vt->data.kb_c,&vt->data_last_status.last_kb_c_status,&vt->data_flag.kb_c_flag);
    flag_process(vt->data.kb_v,&vt->data_last_status.last_kb_v_status,&vt->data_flag.kb_v_flag);
    flag_process(vt->data.kb_b,&vt->data_last_status.last_kb_b_status,&vt->data_flag.kb_b_flag);
	return true;
}

static bool Get_Chassis_Mag(VTM_Handle_t *vtm)
{
    if(!vtm) return false;
    int8_t w_state,s_state,a_state,d_state,q_state,e_state;
    
    w_state = vtm->data.kb_w == 1 ? 1 : 0;
    s_state = vtm->data.kb_s == 1 ? -1 : 0;
    a_state = vtm->data.kb_a == 1 ? -1 : 0;
    d_state = vtm->data.kb_d == 1 ? 1 : 0;
    q_state = vtm->data.kb_q == 1 ? -1 : 0;
    e_state = vtm->data.kb_e == 1 ? 1 : 0;

    vtm->mag.ymag = vtm->data.kb_shift == 0 ? (w_state + s_state) * Chassis_Mag_Factor :
                                        (w_state + s_state) * Chassis_Mag_Factor * Shift_Factor;
    
    vtm->mag.xmag = vtm->data.kb_shift == 0 ? (a_state + d_state) * Chassis_Mag_Factor :
                                        (a_state + d_state) * Chassis_Mag_Factor * Shift_Factor;
    
    vtm->mag.wmag = (q_state + e_state) * Chassis_Mag_Factor * Shift_Factor;
//	  vtm->mag.ymag = (w_state + s_state) * Chassis_Mag_Factor;
//		vtm->mag.xmag = (a_state + d_state) * Chassis_Mag_Factor;
//		vtm->mag.wmag = (q_state + e_state) * Chassis_Mag_Factor;
	
    return true;
}


bool VTM_Recv_Process(VTM_Handle_t *vt,uint8_t *FDCAN_BUF)
{
	if(!vt || !FDCAN_BUF) return false;
	if(!VTM_Data_Process(&vt->data,FDCAN_BUF) || !VTM_Flag_Process(vt) || !Get_Chassis_Mag(vt))
	{
		return false;
	}else
	{
		true;
	}
	return true;
}