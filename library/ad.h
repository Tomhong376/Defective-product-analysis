
extern byte ad_count;
extern word ad_value, v_data, VDD, VDD_smoking, V_out, VDD_charging;
extern eword ad_sum, ad_sum_bandgap;
extern byte ad_turn;

extern byte r_ad_flag;
extern bit flag_data_convey;//		:r_ad_flag.0;
extern bit flag_mos_ever_off;//		:r_ad_flag.1;


void ad_get(void);
void ad_deal(void);
void v_degree_deal(void);
void ad_reset(void);


