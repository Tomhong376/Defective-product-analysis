
#include "../define.h"


#define rf_bit_long		24

dword rf_data;	//|接收数据输出值
dword rf_data_send;	//|接收数据输出值
dword rf_temp;	//|接收数据缓存，相同类型
byte rf_low_count;
byte rf_high_count;
byte rf_bit_time;
byte bit_count;

byte bit_long_count, send_bit_count;


byte rf_value;	//输出结果

byte rf_flag;
bit flag_rf_pre			:rf_flag.0
bit flag_rf_now			:rf_flag.1
bit flag_rf_code_ready	:rf_flag.2
bit	flag_rf_ok			:rf_flag.3
bit flag_rf_active		:rf_flag.4
bit flag_update_rf		:rf_flag.5





/*---------------------------------------------------------------//
//接收程序处理部分                                               //
//---------------------------------------------------------------* /
void rf_receive_deal(void)
{
	byte check_temp;
	if(flag_rf_code_ready)
	{
		flag_rf_code_ready = 0;
//		nixie_data = rf_data;
	}
}
/*{
	if(flag_rf_code_ready)
	{
		flag_rf_code_ready = 0;
		a = rf_data$1 - rf_data$0;
		xor		a,0xA3;
		if(!zf)			{return;}
		a = rf_data$1;
		swap	a;
		and		a,0xf0;
		a = a + rf_data$1;
		and		a,0xf0;
		if(a != 0xf0)	{return;}
		rf_value = rf_data$1 & 0x0f;
		flag_rf_ok = 1;		//接收处理完毕，结果于rf_value
	}
}
*/


/*---------------------------------------------------------------//
//发送程序处理部分                                               //
//---------------------------------------------------------------//
void rf_send_deal(void)
{
	if(!flag_rf_code_ready)
	{
		switch(key_value)
		{
		case 1:
			
			flag_rf_code_ready = 1;
			break;
		case 2:
			flag_rf_code_ready = 1;
			break;
		case 3:
			flag_rf_code_ready = 1;
			break;
		case 4:
			flag_rf_code_ready = 1;
			break;
		default:
			flag_rf_code_ready = 0;
			break;
		}
	}
}
*/
/*---------------------------------------------------------------//
//接收程序中断部分                                               //
//---------------------------------------------------------------* /
void rf_receive(void)
{
	static byte temp1;
	static byte temp2;

	if(flag_rf_now)
	{
		flag_rf_pre = 1;
	}
	else
	{
		flag_rf_pre = 0;
	}
//	if(p_rf_in)
	{
		flag_rf_now = 1;
	}
//	else 
	{
		flag_rf_active = 1;//有低电平都是活跃状态
		flag_rf_now = 0;
	}
	
	if((!flag_rf_pre) && flag_rf_now)
	{
		temp1 = rf_low_count + rf_high_count;
		
		if(temp1 > rf_bit_time)	{	temp2 = temp1 - rf_bit_time;	}			
		else					{	temp2 = rf_bit_time - temp1;	}
		
		if(temp1 > 4)			//可接受的最小位周期
		{
			a = rf_high_count - rf_low_count;
			src		rf_temp$2;
			src		rf_temp$1;
			src		rf_temp$0;
			
			if(temp2 < 4)		//可接受的最大位周期偏差
			{
				bit_count ++;
				if(bit_count >= rf_bit_long)
				{
					rf_data = rf_temp;		//取得结果
					flag_rf_code_ready = 1;	//接收完成标志
					rf_temp = 0;			//可以不清                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
					bit_count = 0;
				}
			}
			else
			{
				bit_count = 1;
				rf_bit_time = temp1;
			}
		}
		else
		{
			bit_count = 0;
		}
		a = 0;
		rf_low_count = a;
		rf_high_count = a;
	}
	else if((!flag_rf_pre) && (!flag_rf_now) && (rf_low_count < 120))
	{
		rf_low_count ++;
	}
	else if(flag_rf_pre && flag_rf_now && (rf_high_count < 120))
	{
		rf_high_count ++;
	}

}

/*---------------------------------------------------------------//
//发送程序中断部分                                               //
//---------------------------------------------------------------* /
void rf_send(void)
{
	if(send_bit_count)
	{
		if(bit_long_count)
		{
			bit_long_count --;
			if(rf_data_send$3.2)
			{
				if(bit_long_count == 0)
				{
					p_rf = 1;
				}
			}
			else
			{
				if(bit_long_count == 1)
				{
					p_rf = 1;
				}
			}
		}
		else
		{
			send_bit_count --;
			if(send_bit_count)
			{
				bit_long_count = 2;
				p_rf = 0;
				slc rf_data_send$0;
				slc rf_data_send$1;
				slc rf_data_send$2;
				slc rf_data_send$3;
			}
		}
	}
}
*/
