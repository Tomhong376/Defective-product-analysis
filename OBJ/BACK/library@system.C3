
#include	"../define.h"



byte high_count, high_count_save;
byte check_circle_count, check_disconnect_delay, disconnect_count, check_low_count;
word sleep_delay_count;
word smoking_time_count;
byte vdd_degree;
byte lowload_blink_count, blink_times_count;
byte lowload_delay_count;
byte V_out_degree;
word charge_circle_count;
byte charge_disconnect_count;
word full_count;

byte charge_circle_count_fast;
byte char_disconnect_check_delay;
byte charge_disconnect_count_fast;
byte charge_disconnect_count_fast_ad;
byte vdd_degree_temp;
word vdd_degree_delay;
word VDD_temp;

//#define full_count_protect smoking_time_count//复用内存!!!!!!!!!
eword full_count_protect;
byte smoke_led_delay;


byte r_system_flag;
bit flag_sleep			:r_system_flag.0	
bit flag_mos			:r_system_flag.1
bit flag_smoking		:r_system_flag.2
bit flag_overload		:r_system_flag.3//发生短路
bit flag_full			:r_system_flag.4
bit flag_charge			:r_system_flag.5
bit flag_low			:r_system_flag.6//欠压
bit flag_lock			:r_system_flag.7

byte r_system_flag1;
bit flag_charge_active	:r_system_flag1.0	
bit flag_poweron_test_full:r_system_flag1.1
bit flag_VDD_charging	:r_system_flag1.2	
bit flag_finish_show_vdd:r_system_flag1.3
bit flag_show_vdd		:r_system_flag1.4
bit flag_lowload		:r_system_flag1.5
bit flag_open_mos		:r_system_flag1.6
bit flag_pre_heat		:r_system_flag1.7

byte r_system_flag2;
bit flag_char_disconn_check	:r_system_flag2.0	
bit flag_vdd_degree_down	:r_system_flag2.1	
bit flag_vdd_degree_up		:r_system_flag2.2	
bit flag_charge_first_in	:r_system_flag2.3	
#if(charge_Type == USB_charging)
	#if(USB_charge_check_full_mode == AD_check_full_pin)
bit flag_charge_pin_ad		:r_system_flag2.4	
	#endif
#endif
bit flag_USB_charging		:r_system_flag2.5	
bit flag_Screw_charging		:r_system_flag2.6	


void sleep_deal(void);
void charge_scan(void);
void charge_deal(void);
void charge_reset(void);
void check_low(void);
void check_full(void);
void p_check_out0(void);
void p_check_in(void);
void check_vdd_degree(void);

void smoking_lowload_blink(void);
void reset_lowload_blink(void);

void degree_led_select(void);

void charging_circle(void);
void charging_circle_fast(void);
void reset_charge_data(void);
void check_full_fast(void);
void charge_degree_led_select(void);

void check_full_pin(void);
void vdd_degree_scan(void);

void system_deal(void)
{
	if(!flag_blink_force)
	{
		key_scan_c_5c();//按键扫描
		charge_scan();//充电检测扫描
		vdd_degree_scan();
		///////////////////充电判断，优先处理
//		if(flag_charge)
		if(flag_Screw_charging)
		{
//			charge_deal();
		}
//		else if(flag_5click && (!led_blink_count))
		else if(flag_5click)//闪灯中(低压)允许关机
		{
			led_blink_count = blink_lock*2+1;
			led_blink_half_circle = 50;
//			flag_blink_force = 1;						//!!!!!有这句就是闪烁期间不给锁机
			led_blink_delay = 0;
//			degree_led_select();//调压档位选灯
//			clear_led_select();//调压档位选灯

			if(flag_lock)
			{
				flag_lock = 0;
			}
			else
			{
				flag_lock = 1;
			}
		}
		elseif(flag_3click && (!led_blink_count))
		{
			flag_pre_heat = 0;
			if(!flag_lock)
			{
				V_out_degree ++;
				if(V_out_degree > 2)
				{
					V_out_degree = 0;
				}
//				degree_led_select();//调压档位选灯
				led_blink_count = blink_degree_select*2+1;//调档闪烁
//				led_blink_half_circle = 50;
//				flag_blink_force = 1;						//!!!!!有这句就是闪烁期间不给锁机
				led_blink_delay = 40;	
			}
		}
		elseif(flag_2click || (flag_pre_heat && (!flag_key_press)) )
		{
			if(!flag_lock)
			{
				if(flag_pre_heat)
				{
					smoking_time_count ++;
					if(smoking_time_count >= pre_heat_time_up_data)
					{
						flag_pre_heat = 0;
//						degree_led_select();//调压档位选灯
//						led_blink_count = blink_smoke_time_up*2+1;//超时闪烁
//						led_blink_half_circle = 27;
//						flag_blink_force = 1;						//!!!!!有这句就是闪烁期间不给锁机
						led_r_duty = 0;
						led_g_duty = 0;
						led_b_duty = 0;
					}
				}
				if(flag_2click && (!led_blink_count))
				{
//					degree_led_select();//调压档位选灯
					flag_pre_heat = 1;
					led_duty = 0;
					smoking_time_count = 0;
					led_delay_count = 0;
					led_turn = 0;
				#if(short_reset_deal == reset_keep_Vout_degree)
					enable_mos_working_mack_ram();
				#endif
					mos_duty = mos_duty_max;//*370/420;
					mos_on(); 
				}
			}
		}
		elseif(!led_blink_count)
		{
			///////////////////吸烟判断
			if(flag_key_press)
			{
				flag_pre_heat = 0;
				if(!flag_lock)
				{
//					degree_led_select();//调压档位选灯
//					if(VDD >= VDD_low_data)
					if((flag_low == 0) && (VDD >= VDD_low_data))
					{
						flag_smoking = 1;//正常吸烟
					#if(short_reset_deal == reset_keep_Vout_degree)
						enable_mos_working_mack_ram();
					#endif
						mos_duty = mos_duty_max;//*370/420;//以最小占空比开始
						mos_on(); 
						led_on();	
						led_duty = 0;//led_duty_max/10;
						smoke_led_delay = 102;//充电灯比连击晚生效
						smoking_time_count = 0;//抽烟计时清零
					}
					else
					{
						flag_low = 1;
						led_blink_count = blink_low*2+1;//低压闪烁
//						flag_blink_force = 1;			//!!!!!有这句就是闪烁期间不给锁机
//						led_blink_half_circle = 50;
					}
				}
			}
			///////////////////吸烟过程处理
			elseif(flag_smoking)
			{
				if(flag_key)
				{
					smoking_time_count ++;
					if(smoking_time_count >= smoke_time_up_data)
					{
						led_blink_count = blink_smoke_time_up*2+1;//超时闪烁
//						led_blink_half_circle = 27;
//						flag_blink_force = 1;						//!!!!!有这句就是闪烁期间不给锁机
						flag_smoking = 0;
					}
				}
				else
				{
					flag_smoking = 0;
				}
			
				if(flag_smoking)
				{
					//仍在按按着按键
					led_on();
				}
				else
				{
					//松开按键或超时
					led_off();
				}
			}
		}
	}
	else
	{
		reset_5click();
	}

	flag_key_press = 0;
//	flag_key_release = 0;
	flag_5click = 0;
	flag_3click = 0;
	flag_2click = 0;

	///////////////////判断是否睡眠
	if((!flag_pre_heat) && (!led_blink_count) && (!flag_smoking) && ((!flag_charge) || (flag_charge && flag_full)) && (!flag_key_active) && (!flag_charge_active) && (!led_duty) && (!show_degree_delay))
		//不闪灯 不吸烟 不充电或者充电已经充满 按键状态稳定 充电接口状态稳定
//	if((!led_blink_count) && (!flag_smoking) && (!flag_charge) && (!flag_key_active) && (!flag_charge_active))
		//不闪灯 不吸烟 不充电 按键状态稳定 充电接口状态稳定
//	if((!led_blink_count) && (!led_duty) && (!flag_smoking) && (!flag_key_active) && (!flag_charge_active))
		//不闪灯 灯已经熄灭 不吸烟 按键状态稳定 充电接口状态稳定
//	if((!led_blink_count) && (!led_duty) && (!flag_smoking) && (!flag_key_active))
		//不闪灯 灯已经熄灭 不吸烟 按键状态稳定 
	{
		sleep_delay_count ++;
		if(sleep_delay_count >= sleep_delay_time)
		{
			sleep_delay_count = 0;
			flag_sleep = 1;
		}
	}
	else
	{
		sleep_delay_count = 0; 
	}
}


/*-------------------------------------//
//------------充电扫描处理-------------//
//-------------------------------------*/
void charge_scan(void)
{
	static byte charge_scan_delay;
	static byte Screw_charge_scan_delay;
	if(flag_smoking || flag_pre_heat || smoke_led_delay)
	{
		charge_scan_delay = 0;
		Screw_charge_scan_delay = 0;
		flag_full = 0;
		return;
	}

	flag_charge_active = 0;//默认充电接口状态稳定

	if(flag_charge)
	{
		if(!led_blink_count)
		{
		#if(charge_Type==Screw_charging )   //螺纹头充电
			#if(Screw_charge_off_mode == Check_off_charge_fast)//快速检测螺纹头断开模式
			#elseif(Screw_charge_off_mode == Check_off_charge_slow)
				check_full();
				charging_circle();
			#endif
		#elseif(charge_Type==USB_charging) //USB充电
			#if(USB_charge_check_full_mode == IO_check_full_pin)
				flag_low = 0;
				if(flag_USB_charging)
				{
					check_full_pin();
				}
				else if(flag_Screw_charging)
				{
					check_full_fast();
					charging_circle_fast();
				}
				if(!flag_full)		led_duty = led_duty_max;
				else				led_duty = 0;
			#elseif(USB_charge_check_full_mode == AD_check_full_pin)
				flag_low = 0;
//				led_duty = led_duty_max;
//				clear_led_select();
//				if(flag_full)	flag_led_g = 1;
//				else			flag_led_r = 1;
				if(flag_charge_pin_ad) return;//AD采集未完成，不进行IO电平判断
				flag_charge_pin_ad = 1;//AD采集已完成，重新置1提示进行下一轮AD采集，并允许以下进行是否在充电的检测
			#elseif(USB_charge_check_full_mode == BAT_AD_check_full)
				led_duty = led_duty_max;
				clear_led_select();
				if(flag_full)	flag_led_g = 1;
				else			flag_led_r = 1;
				check_full_fast();
			#endif
		#endif
		}
	}
//#if(charge_Type==Screw_charging )
//	else
//	{
		if((!led_blink_count) && (!flag_Screw_charging) && p_check)//判断螺纹头充电接上
		{
			flag_charge_active = 1;//充电接口有动态
			Screw_charge_scan_delay ++;
			if(Screw_charge_scan_delay >= 100)
			{
				Screw_charge_scan_delay = 0;
				flag_Screw_charging = 1;
//				flag_charge_active = 0;//充电接口状态稳定
//				flag_charge = 1;
//				flag_charge_first_in = 1;
			#if(short_reset_deal == reset_keep_Vout_degree)
				enable_mos_working_mack_ram();
			#endif
//				degree_led_select();//调压档位选灯
//				clear_led_select();
//				flag_led_r = 1;
//				led_blink_count = blink_get_charge*2+1;//闪烁
//				led_blink_delay = 0;	//闪烁，需要清零闪烁计时
//				led_blink_half_circle = 50;	
//				reset_charge_data();
			}
		}
		else
		{
			Screw_charge_scan_delay = 0;
		}
//	}
//#elseif(charge_Type==USB_charging) //USB充电
	#if(USB_charge_check_full_mode == IO_check_full_pin)
//	if( (!led_blink_count) && ( ((!flag_charge) && ((!p_charge) || (!p_full))) || (flag_charge && p_charge && p_full) ) )//充电脚和充满脚都分别接到主控的原理
	if( (!led_blink_count) && ( ((!flag_USB_charging) && ((!p_charge) || (!p_full))) || (flag_USB_charging && p_charge && p_full) ) )//充电脚和充满脚都分别接到主控的原理
	#elseif(USB_charge_check_full_mode == AD_check_full_pin)
	if( (!led_blink_count) && ( ((!flag_charge) && (!p_charge)) || (flag_charge && p_charge)) )//充满脚串接10K电阻后和充电脚并联后接到主控同一个IO的原理
	#elseif(USB_charge_check_full_mode == BAT_AD_check_full)
	if( (!led_blink_count) && ( ((!flag_charge) && (!p_charge)) || (flag_charge && p_charge)) )//仅充电脚接到主控IO口或者充满脚和充电脚直接并联后接到主控同一个IO的原理
	#endif
	{
		flag_charge_active = 1;//充电接口有动态
		charge_scan_delay ++;
		if(charge_scan_delay >= 50)
		{
			charge_scan_delay = 0;
//			flag_charge_active = 0;//充电接口状态稳定
//			if(flag_charge)
			if(flag_USB_charging)
			{
//				flag_charge = 0;
				flag_USB_charging = 0;
//				led_duty = 0;
			}
			else
			{
				flag_USB_charging = 1;
//				flag_charge = 1;
//				flag_charge_first_in = 1;
//				degree_led_select();//调压档位选灯
//				clear_led_select();
//				flag_led_r = 1;
//				led_blink_count = blink_get_charge*2+1;//闪烁
//				led_blink_delay = 0;	//闪烁，需要清零闪烁计时
//				led_blink_half_circle = 50;	
//				reset_charge_data();
			}
		}
	}
	else
	{
		charge_scan_delay = 0;
//		flag_charge_active = 0;//充电接口状态稳定
	}
//	#endif
	
	if(((!flag_charge) && (flag_USB_charging || flag_Screw_charging)) || (flag_charge && (!flag_USB_charging) && (!flag_Screw_charging)))
	{
		if(flag_charge)
		{
			flag_charge = 0;
			led_duty = 0;
		}
		else
		{
			flag_charge = 1;
			flag_charge_first_in = 1;
			reset_charge_data();
		}
	}
}

/*
void check_charge_degree(void)
{
	check_vdd_degree();
	if(flag_charge_first_in)
	{
		flag_charge_first_in = 0;
		vdd_degree = vdd_degree_temp;
//		charge_degree_led_select();
	}
	if(vdd_degree_temp > vdd_degree)
	{
		flag_vdd_degree_down = 0;
		if(flag_vdd_degree_up)
		{
			vdd_degree_delay ++;
			if(vdd_degree_delay >= 600)
			{
				vdd_degree_delay = 0;
				vdd_degree ++;
//				charge_degree_led_select();
			}
		}
		else
		{
			flag_vdd_degree_up = 1;
			vdd_degree_delay = 1;
		}
	}
	else if(vdd_degree_temp < vdd_degree)
	{
		flag_vdd_degree_up = 0;
		if(flag_vdd_degree_down)
		{
			vdd_degree_delay ++;
			if(vdd_degree_delay >= 600)
			{
				vdd_degree_delay = 0;
				vdd_degree --;
//				charge_degree_led_select();
			}
		}
		else
		{
			flag_vdd_degree_down = 1;
			vdd_degree_delay = 1;
		}
	}
	else
	{
		flag_vdd_degree_up = 0;
		flag_vdd_degree_down = 0;
	}
	charge_degree_led_select();
}
*/
/*-------------------------------------//
//-----------充电过程检测循环----------//
//-------------------------------------*/
void charging_circle_fast(void)
{
	charge_circle_count_fast ++;
	if(charge_circle_count_fast >= 4)//乘以5ms
	{
		charge_circle_count_fast = 0;
		mos_duty = 0;
		mos_off();
		p_check_out0();
//		char_disconnect_check_delay = 20;//乘以100us
	}
	else if(!flag_full)
	{		
//		check_charge_degree();
//		led_duty = led_duty_max;
//		led_on();
		mos_duty = mos_duty_max;
		mos_on();
	}
}


void check_full_fast(void)
{
	if(!flag_full)
	{
		if(VDD > VDD_full_data)
		{
			full_count ++;
			if(full_count > 1000)
			{
				flag_full = 1;
			}
		}
		else 
		{
			full_count = 0;
		}

		if(VDD > (VDD_full_data - 8))
		{
			if(VDD_temp < VDD)
			{
				full_count_protect = 0;
				if(VDD_temp == 0)	VDD_temp = VDD;
				else				VDD_temp ++;
			}
			else
			{
				full_count_protect++;
				if(full_count_protect > 60000)
				{
					flag_full =  1;
				}
			}
		}
		else
		{
			full_count_protect = 0;
		}

		if(flag_full)
		{
			full_count = 0;
			full_count_protect = 0;
//		#if(charge_Type==Screw_charging )   //螺纹头充电
//			clear_led_select();
//			flag_led_r = 1;
//			led_blink_count = blink_full*2+1;//闪烁
//			led_blink_delay = 0;	//闪烁，需要清零闪烁计时
//			led_blink_half_circle = 50;	
			mos_duty = 0;
			mos_off();
//		#endif
		}
	}
	else
	{
		full_count = 0;
		full_count_protect = 0;
	}
}


/*-------------------------------------//
//-----------充电过程检测循环----------//
//-------------------------------------* /
void charging_circle(void)
{
	charge_circle_count ++;
	if(charge_circle_count < 380)
	{
		if(!flag_full)
		{
			clear_led_select();
			flag_led_r = 1;
			led_duty = led_duty_max;
			led_on();
			mos_duty = mos_duty_max;
			mos_on();
		}
	}
	elseif(charge_circle_count == 380)
	{
//		led_on();
		mos_duty = 0;
		mos_off();
		p_check_out0();
		charge_disconnect_count = 0;
	}
	elseif(charge_circle_count < 400)
	{
		p_check_io_in();
//		led_on();
		mos_duty = 0;
		mos_off();
		nop;
		nop;
		nop;
		if(!p_check)
		{
			charge_disconnect_count ++;
		}
		p_check_out0();
	}
	else
	{
		p_check_io_in();
		if(flag_full)
		{
			charge_circle_count = 379;
		}
		else
		{
			charge_circle_count = 0;
		}
		if(charge_disconnect_count >= 8)
		{
			mos_duty = 0;
			mos_off();
			flag_charge = 0;
//			degree_led_select();//调压档位选灯
			clear_led_select();
			flag_led_r = 1;
			led_blink_count = blink_off_charge*2+1;//闪烁
			led_blink_delay = 0;	//闪烁，需要清零闪烁计时
//			led_blink_half_circle = 50;	
		}
		else if(!flag_full)
		{
//			degree_led_select();//调压档位选灯
			led_duty = led_duty_max;
			led_on();
			mos_duty = mos_duty_max;
			mos_on();
		}
	}
}
*/

void check_full_pin(void)
{
	if(!flag_full)
	{
//		led_duty = led_duty_max;
//		check_charge_degree();
		if(!p_full)
		{
			full_count ++;
			if(full_count > 100)
			{
				full_count = 0;
				flag_full = 1;
//				degree_led_select();//档位选灯
//				clear_led_select();
//				flag_led_r = 1;
//				led_blink_count = blink_full*2+1;//闪烁
//				led_blink_delay = 0;	//闪烁，需要清零闪烁计时
//				led_blink_half_circle = 40;	
//				flag_led_Level_blink = 0;
			}
		}
		else
		{
			full_count = 0;
		}
	}
	else
	{
/*		if(p_full && (flag_smoking || flag_pre_heat))
		{
			full_count ++;
			if(full_count > 100)
			{
				full_count = 0;
				flag_full = 0;
			}
		}
		else
*/		{
			full_count = 0;
		}
//		full_count = 0;
//		led_duty = 0;//led_duty_max;
//		clear_led_select();
//		flag_led_g = 1;
	}
}


/*-------------------------------------//
//------------充电过程参数复位---------//
//-------------------------------------*/
void reset_charge_data(void)
{
	flag_full = 0;
	full_count = 0;
//	charge_circle_count = 379;
	charge_disconnect_count_fast = 0;
	flag_vdd_degree_up = 0;
	flag_vdd_degree_down = 0;
	full_count_protect = 0;//复用内存!!!
	VDD_temp = 0;
}

/*
void check_full(void)
{
	if(!flag_full)
	{
		if(VDD > VDD_full_data)
		{
			full_count ++;
			if(full_count > 2000)
			{
				flag_full = 1;
			}
		}
		else 
		{
			full_count = 0;
		}

		if(VDD > (VDD_full_data - 6))
		{
			if(VDD_temp < VDD)
			{
				full_count_protect = 0;
				if(VDD_temp == 0)	VDD_temp = VDD;
				else				VDD_temp ++;
			}
			else
			{
				full_count_protect++;
				if(full_count_protect > 65000)
				{
					flag_full =  1;
				}
			}
		}
		else
		{
			full_count_protect = 0;
		}

		if(flag_full)
		{
			full_count = 0;
			full_count_protect = 0;
			flag_full = 1;
			charge_circle_count = 379;
//			degree_led_select();//调压档位选灯
			clear_led_select();
			flag_led_r = 1;
			led_blink_count = blink_full*2+1;//闪烁
			led_blink_delay = 0;	//闪烁，需要清零闪烁计时
//			led_blink_half_circle = 50;	
			mos_duty = 0;
			mos_off();
		}
	}
	else
	{
		full_count = 0;
		full_count_protect = 0;
	}
}
/ *
void check_vdd_degree(void)
{
	vdd_degree_temp = 0;
	if(VDD > vdd_degree01)
	{
		vdd_degree_temp ++;
		if(VDD > vdd_degree12)
		{
			vdd_degree_temp ++;
//			if(VDD > vdd_degree23)
			{
//				vdd_degree_temp ++;
			}
		}
	}
}
void reset_lowload_blink(void)
{
	lowload_blink_count = 0;
	blink_times_count = 0;
}

void smoking_lowload_blink()
{
	lowload_blink_count ++;
	if(lowload_blink_count >= 100)
	{
		lowload_blink_count = 0;
		blink_times_count ++;
	}
	if(blink_times_count.0)
	{
		led_off();
	}
	else
	{
		led_on();
	}
}
*/

void p_check_out0(void)
{
	p_check_c = 1;
	p_check = 0;
}
void p_check_io_in(void)
{
	p_check_c = 0;
	set_check_io;
}
void p_check_ad_in(void)
{
	p_check_c = 0;
	set_check_ad;
}
/*
#if(charge_Type == USB_charging)
	#if(USB_charge_check_full_mode == AD_check_full_pin)
void p_charge_io_in(void)
{
	set_charge_pin_io;
	flag_charge_pin_ad = 0;
}
void p_charge_ad_in(void)
{
	set_charge_pin_ad;
//	flag_charge_pin_ad = 1;	//充电函数每5mS置1一次，只有该标志是1时才允许切换成AD口，读完AD切换回IO口后把标志位清0，
							//充电函数只有等待在标志位为0时才能读取电平，读完标志后又把它置1
}
	#endif
#endif
*/
void mos_on(void)
{
	p_mos = 0;
	p_mos_c = 1;
	p_mos = 0;
	flag_mos = 1;
}

void mos_off(void)
{
	p_mos_c = 0;
	flag_mos = 0;
	flag_mos_ever_off = 1;
}
/*
#define vdd_degree_point1	(vdd_degree01-3)
#define vdd_degree_point2	(vdd_degree12-3)
void degree_led_select(void)
{
	clear_led_select();
	if(!V_out_degree) 			flag_led_r = 1;//第0档
	else if(V_out_degree == 1)	flag_led_g = 1;//第1档
	else 						flag_led_b = 1;//第2档

/*	if(!vdd_degree)
	{
		if(VDD < (vdd_degree_point1+3)) 			{ flag_led_r = 1; }//第0档
		else if(VDD < (vdd_degree_point2+3))		{ flag_led_b = 1; vdd_degree = 1;}//第1档
		else 										{ flag_led_w = 1; vdd_degree = 2;}//第2档
	}
	elseif(vdd_degree == 1)
	{
		if(VDD < (vdd_degree_point1-3)) 			{ flag_led_r = 1; vdd_degree = 0;}//第0档
		else if(VDD < (vdd_degree_point2+3))		{ flag_led_b = 1; }//第1档
		else 										{ flag_led_w = 1; vdd_degree = 2;}//第2档
	}
	else
	{
		if(VDD < (vdd_degree_point1-3)) 			{ flag_led_r = 1; vdd_degree = 0;}//第0档
		else if(VDD < (vdd_degree_point2-3))		{ flag_led_b = 1; vdd_degree = 1;}//第1档
		else 										{ flag_led_w = 1; }//第2档
	}
* /
}
/*
void charge_degree_led_select(void)
{
	//--------------------------档位选择
	clear_led_select();
	switch(vdd_degree)
	{
	case 0:
		flag_led_g = 1;
		break;
	case 1:
		flag_led_b = 1;
		break;
	default:
		flag_led_r = 1;
		break;
	}
}
*/
#define vdd_degree_cutline3		400		
#define vdd_degree_cutline2		382		
#define vdd_degree_cutline1		363		
#define vdd_degree_cutline0		VDD_low_data
void vdd_degree_scan(void)
{
	if(flag_poweron)
	{
		flag_poweron = 0;
		if(VDD > vdd_degree_cutline0) vdd_degree ++;
		if(VDD > vdd_degree_cutline1) vdd_degree ++;
		if(VDD > vdd_degree_cutline2) vdd_degree ++;
		if(VDD > vdd_degree_cutline3) vdd_degree ++;
		return;
	}

	vdd_degree_temp = vdd_degree;
	if(flag_charge)
	{
		switch(vdd_degree_temp)
		{
		case 0:
			if(VDD > vdd_degree_cutline0+15) 		vdd_degree_temp ++;
			break;
		case 1:
			if(VDD > vdd_degree_cutline1+5) 		vdd_degree_temp ++;
			else if(VDD < vdd_degree_cutline0)		vdd_degree_temp --;
			break;
		case 2:
			if(VDD > vdd_degree_cutline2+5) 		vdd_degree_temp ++;
			else if(VDD < vdd_degree_cutline1-2)	vdd_degree_temp --;
			break;
		case 3:
			if(VDD > vdd_degree_cutline3+3) 		vdd_degree_temp ++;
			else if(VDD < vdd_degree_cutline2-2)	vdd_degree_temp --;
			break;
		case 4:
			if(VDD < vdd_degree_cutline3-2)			vdd_degree_temp --;
			break;
		}
	}
	else
	{
		switch(vdd_degree_temp)
		{
		case 0:
			if(VDD > vdd_degree_cutline0+10) 		vdd_degree_temp ++;
			break;
		case 1:
			if(VDD > vdd_degree_cutline1+3) 		vdd_degree_temp ++;
			else if(VDD < vdd_degree_cutline0)		vdd_degree_temp --;
			break;
		case 2:
			if(VDD > vdd_degree_cutline2+3) 		vdd_degree_temp ++;
			else if(VDD < vdd_degree_cutline1-3)	vdd_degree_temp --;
			break;
		case 3:
			if(VDD > vdd_degree_cutline3+3) 		vdd_degree_temp ++;
			else if(VDD < vdd_degree_cutline2-3)	vdd_degree_temp --;
			break;
		case 4:
			if(VDD < vdd_degree_cutline3-3)			vdd_degree_temp --;
			break;
		}
	}
	if(vdd_degree_temp > vdd_degree)
	{
		if(flag_vdd_degree_up)	vdd_degree_delay ++;
		else					vdd_degree_delay = 0;
		flag_vdd_degree_up = 1;
	}
	else if(vdd_degree_temp < vdd_degree)
	{
		if(!flag_vdd_degree_up)	vdd_degree_delay ++;
		else					vdd_degree_delay = 0;
		flag_vdd_degree_up = 0;
	}
	else
	{
		vdd_degree_delay = 0;
		flag_vdd_degree_up = 0;
	}
	if(vdd_degree_delay >= 20) 
	{
		vdd_degree_delay = 0;
		vdd_degree = vdd_degree_temp;
	}
	if(flag_low)	vdd_degree = 0;
}

#if(short_reset_deal == reset_keep_Vout_degree)
/*-------------------------------------//
//------------短保复位闪烁保护---------//
//-------------------------------------*/
void enable_mos_working_mack_ram(void)
{
	mos_working_mark1 = mos_working_key1;
	mos_working_mark2 = mos_working_key2;
	mos_working_mark3 = mos_working_key3;
	degree_bk = V_out_degree;
}
void disable_mos_working_mack_ram(void)
{
	mos_working_mark1 = 0;
	mos_working_mark2 = 0;
	mos_working_mark3 = 0;
	degree_bk = 0;
}
#endif
