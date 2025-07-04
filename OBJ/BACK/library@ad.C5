

#include "../define.h"

#define ad_times			10
#define ad_abandon_times	4

#define V_overload			32//待测定，mos高温时的D和S压降，
#define V_lowload			6

#if(P_out_config == P_out_div_Vout)
#define ad_mos_overload		((420-V_overload)*4095*ad_times*10/12/420)//过载保护AD值
#else
#define ad_mos_overload		((420-V_overload)*4095*ad_times/420)//过载保护AD值
#endif

byte ad_count, load_delay_count, load_low_count;
word ad_value, VDD, VDD_protect, V_out, VDD_smoking;
eword ad_sum, ad_sum_bandgap;
byte ad_turn, charge_remove_delay;

byte ad_charge_turn;

dword ad_cal_dword;


byte r_ad_flag;
bit flag_mos_pre		:r_ad_flag.0;
bit	flag_mos_ever_off	:r_ad_flag.1;


void short_protect_deal(void);//短路保护处理
void short_io_protect(void);//短路保护监测


void ad_get(void)
{
	if(!AD_Start)
	{
		return;
	}

	ad_value = (adcrh << 8 | adcrl);
 	ad_value =  ad_value >> 4;  

	if(ad_count >= ad_abandon_times)
	{
		ad_sum += ad_value;//adcr;
	}

	AD_Start = 1;		//开始

	if((flag_mos && ((!flag_mos_pre) || (flag_mos_ever_off))) || ((!flag_mos) && flag_mos_pre))
	{
		ad_reset();
		flag_mos_ever_off = 0;
		if(flag_mos)
		{
			flag_mos_pre = 1;
		}
		else
		{
			flag_mos_pre = 0;
		}
		return;
	}

	ad_count ++;
	if(ad_count >= ad_times + ad_abandon_times)
	{
		ad_deal();
		ad_count = 0;
		ad_sum = 0;
	}
}



void ad_deal(void)
{
	if(ad_turn == 0)
	{
	   	ad_sum_bandgap = ad_sum;

		if(flag_mos_pre)//采集过程mos状态为打开
		{
			p_check_ad_in();
			$ adcc enable, p_check_ad;
			ad_turn ++;

			math_word0 = ad_sum_bandgap;
		#if(P_out_config == P_out_div_Vout)
			if(flag_pre_heat)				math_word = mos_duty_max*Vout_pre_heat*10/120/12;
			else
			{
				if(!V_out_degree)			math_word = mos_duty_max*Vout_degree0*10/120/12;
				elseif(V_out_degree == 1)	math_word = mos_duty_max*Vout_degree1*10/120/12;
				else						math_word = mos_duty_max*Vout_degree2*10/120/12;//有分压电阻
			}
		#else
			if(flag_pre_heat)				math_word = mos_duty_max*Vout_pre_heat/120;
			else
			{			
				if(!V_out_degree)			math_word = mos_duty_max*Vout_degree0/120;
				elseif(V_out_degree == 1)	math_word = mos_duty_max*Vout_degree1/120;
				else						math_word = mos_duty_max*Vout_degree2/120;//无分压电阻
			}
		#endif
			Math_Word_Mul_Word();
			ad_cal_dword = math_dword;
		}
		else
		{

//		#if(Screw_charge_off_mode == Check_off_charge_fast)
			if(flag_Screw_charging && (!flag_USB_charging))
			{
				p_check_ad_in();
				$ adcc enable, p_check_ad;
				ad_turn ++;
			}
//		#endif

		#if(charge_Type == USB_charging)
			#if(USB_charge_check_full_mode == AD_check_full_pin)
			if(flag_charge_pin_ad)
			{
				if(flag_charge && (!flag_full))
				{
					p_charge_ad_in();
					$ adcc enable, p_charge_ad;
					ad_turn = 2;
				}
				else
				{
					p_charge_io_in();//不需要检测充满信号，标志位直接置0结束AD检测
				}
			}
			#endif
		#endif
			math_eword = ad_sum;
			math_dword = 120*4096*ad_times;//1.2V,AD次数
			Math_Dword_Div_Eword();
			VDD = math_dword;//采集不带载电池电压
		}
	}
	else if(ad_turn == 1)
	{
//	  $ adcc enable, ADC;
	  $ adcc enable, BANDGAP;//363
		p_check_io_in();

		if(flag_mos_pre)
		{
			if(ad_sum < ad_mos_overload)
			{
				p_test_1;
				mos_duty = 0;
				mos_off();			//紧急关闭输出
//				flag_overload = 1;
				if(flag_Screw_charging)
				{
					flag_Screw_charging = 0;
					flag_charge = 0;
				}
//			#if(charge_Type==Screw_charging )
//				flag_charge = 0;
//			#endif
				flag_smoking = 0;
				flag_pre_heat = 0;
			#if(short_reset_deal == reset_keep_Vout_degree)
				enable_mos_working_mack_ram();
			#elseif(short_reset_deal == reset_rst_Vout_degree)
				V_out_degree = 0;
			#endif
				clear_led_select();
				flag_led_w = 1;
				led_blink_count = blink_overload*2+1;//闪烁三次
//				led_blink_half_circle = 50;
//				led_blink_delay = 0;	//闪烁，需要清零闪烁计时
//				flag_blink_force = 1;	//设为强制，闪烁过程清除连击状态
			}
		}
		//--------------充电fast检测断开充电-------------------//
//	#if(Screw_charge_off_mode == Check_off_charge_fast)
//		else if(flag_charge)
		else if(flag_Screw_charging)
		{
			if(!flag_full)
			{
				mos_duty = mos_duty_max;
				mos_on();
			}
		#if(P_out_config == P_out_div_Vout)
			if(ad_sum < ((420-30)*4095*ad_times*10/12/420))
		#else
			if(ad_sum < ((420-30)*4095*ad_times/420))
		#endif
			{
				charge_disconnect_count_fast ++;
				if(charge_disconnect_count_fast > 25)
				{
					charge_disconnect_count_fast = 0;
//					flag_charge = 0;//充电断开
					flag_Screw_charging = 0;//充电断开

//					clear_led_select();
//					flag_led_r = 1;
//					led_blink_count = blink_off_charge*2+1;//闪烁
//					led_blink_delay = 0;	//闪烁，需要清零闪烁计时
//						led_blink_half_circle = 40;	
				}
			}
			else
			{
				charge_disconnect_count_fast = 0;
			}
		}
//	#endif
		//--------------充电fast检测断开充电-------------------//

		math_dword = ad_cal_dword;
		math_word = ad_sum;
		Math_Dword_Div_Word();
		if(flag_mos_pre && (flag_smoking || flag_pre_heat))
		{
			if(math_dword > mos_duty_max)				mos_duty = mos_duty_max;
			elseif(math_dword < mos_duty_max*Vout_min/430)	mos_duty = mos_duty_max*Vout_min/430;
			else										mos_duty = math_dword;
		}

		ad_turn = 0;
	}
#if(charge_Type == USB_charging)
	#if(USB_charge_check_full_mode == AD_check_full_pin)
	else if(ad_turn == 2)
	{
	  $ adcc enable, BANDGAP;//363
		p_charge_io_in();
		ad_turn = 0;

		if(ad_sum > 4095*10/(10+80)*ad_times*3/10)//充满脚串接10K过来，内部上拉阻值大概80K，0.3倍是逻辑划分线
		{//电平信号不是0V左右,就是充电拔掉或者充满信号
			if(ad_sum < 4095*ad_times*3/10)//过滤掉充点拔掉，剩下就是充满信号了
			{
				full_count ++;
				if(full_count >= 100)
				{
					full_count = 0;
					flag_full = 1;//充满脚提示充满
				}
			}
			else
			{
				full_count = 0;
			}
		}
		else
		{
			full_count = 0;
		}
	}
	#endif
#endif
}


//-----------------------------------------//
//---------------AD重置处理----------------//
//-----------------------------------------//
void ad_reset(void)
{
	p_check_io_in();
	ad_turn = 0;
	ad_count = 0;
	ad_sum = 0;
//  $ adcc enable, ADC;
  $ adcc enable, BANDGAP; //363
	AD_Start = 1;
}