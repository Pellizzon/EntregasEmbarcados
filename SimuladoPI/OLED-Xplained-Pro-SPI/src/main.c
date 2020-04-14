#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

#define LED1_PIO PIOA
#define LED1_PIO_ID ID_PIOA
#define LED1_IDX 0
#define LED1_IDX_MASK (1 << LED1_IDX)

#define LED2_PIO PIOC
#define LED2_PIO_ID ID_PIOC
#define LED2_IDX 30
#define LED2_IDX_MASK (1 << LED2_IDX)

#define LED3_PIO PIOB
#define LED3_PIO_ID ID_PIOB
#define LED3_IDX 2
#define LED3_IDX_MASK (1 << LED3_IDX)

#define BUT1_PIO PIOD
#define BUT1_PIO_ID ID_PIOD
#define BUT1_IDX 28
#define BUT1_IDX_MASK (1u << BUT1_IDX)

#define BUT2_PIO PIOC
#define BUT2_PIO_ID ID_PIOC
#define BUT2_IDX 31
#define BUT2_IDX_MASK (1u << BUT2_IDX)

#define BUT3_PIO PIOA
#define BUT3_PIO_ID ID_PIOA
#define BUT3_IDX 19
#define BUT3_IDX_MASK (1 << BUT3_IDX)

volatile char flag_tc0 = 0;
volatile char flag_tc1 = 0;
volatile char flag_tc2 = 0;
volatile char but_flag1 = 1;
volatile char but_flag2 = 1;
volatile char but_flag3 = 1;
volatile Bool f_rtt_alarme = false;
volatile char flag_leds = 0;
volatile char flag_rtc = 0;
volatile char rtt_irq = 0;

typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t seccond;
} calendar;

void LED_init(void);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);
void pin_toggle(Pio *pio, uint32_t mask);
void but1_callback(void);
void but2_callback(void);
void but3_callback(void);
void LEDs_init(void);
void BUTs_init(void);

void but1_callback(void)
{
	but_flag1 = !but_flag1;
}

void but2_callback(void)
{
	but_flag2 = !but_flag2;
}

void but3_callback(void)
{
	but_flag3 = !but_flag3;
}

void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask)) pio_clear(pio, mask);
	else pio_set(pio,mask);
}

void TC0_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	flag_tc0 = 1;
}

void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	flag_tc1 = 1;
}

void TC2_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 2);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	flag_tc2 = 1;
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	/* O TimerCounter é meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupçcão no TC canal 0 */
	/* Interrupção no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
		rtt_irq = 1;
	}

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		flag_leds = !flag_leds;
		f_rtt_alarme = true;       // flag RTT alarme
	}
}

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	 uint32_t ul_previous_time;

	 /* Configure RTT for a 1 second tick interrupt */
	 rtt_sel_source(RTT, false);
	 rtt_init(RTT, pllPreScale);
	 
	 ul_previous_time = rtt_read_timer_value(RTT);
	 while (ul_previous_time == rtt_read_timer_value(RTT));
	 
	 rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

	 /* Enable RTT interrupt */
	 NVIC_DisableIRQ(RTT_IRQn);
	 NVIC_ClearPendingIRQ(RTT_IRQn);
	 NVIC_SetPriority(RTT_IRQn, 0);
	 NVIC_EnableIRQ(RTT_IRQn);
	 rtt_enable_interrupt(RTT, RTT_MR_ALMIEN | RTT_MR_RTTINCIEN);
}

void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/*
	*  Verifica por qual motivo entrou
	*  na interrupcao, se foi por segundo
	*  ou Alarm
	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		flag_rtc = 1;
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
		rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	}
	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.seccond);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 0);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}

void LEDs_init(void){
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_IDX_MASK, PIO_DEFAULT);
	
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_configure(LED2_PIO, PIO_OUTPUT_1, LED2_IDX_MASK, PIO_DEFAULT);

	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_configure(LED3_PIO, PIO_OUTPUT_0, LED3_IDX_MASK, PIO_DEFAULT);
};

void BUTs_init(void){
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_IDX_MASK, PIO_PULLUP);
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_IDX_MASK, PIO_IT_RISE_EDGE, but1_callback);
	pio_enable_interrupt(BUT1_PIO, BUT1_IDX_MASK);
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4); // Prioridade 4
	
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_IDX_MASK, PIO_PULLUP);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_IDX_MASK, PIO_IT_FALL_EDGE, but2_callback);
	pio_enable_interrupt(BUT2_PIO, BUT2_IDX_MASK);
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 4); // Prioridade 4
	
	pmc_enable_periph_clk(BUT3_PIO_ID);
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_IDX_MASK, PIO_PULLUP);
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_IDX_MASK, PIO_IT_RISE_EDGE, but3_callback);
	pio_enable_interrupt(BUT3_PIO, BUT3_IDX_MASK);
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 4); // Prioridade 4
}

int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	LEDs_init();
	BUTs_init();
	
	WDT->WDT_MR = WDT_MR_WDDIS;
	f_rtt_alarme = true;

	TC_init(TC0, ID_TC0, 0, 5);
	TC_init(TC0, ID_TC1, 1, 10);
	TC_init(TC0, ID_TC2, 2, 1);
	
	but_flag1 = 1;
	but_flag2 = 1;
	but_flag3 = 1;
	 // Init OLED
	gfx_mono_ssd1306_init();
	
	//Escreve frequencia dos leds na tela
	//gfx_mono_draw_string("5/10/1", 80, 16, &sysfont);
	
	calendar rtc_initial = {2020, 4, 5, 12, 15, 19 , 1};
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN | RTC_IER_SECEN);
	rtc_set_hour_mode(RTC, 0);

	/* configura alarme do RTC */
	rtc_set_date_alarm(RTC, 1, rtc_initial.month, 1, rtc_initial.day);
	rtc_set_time_alarm(RTC, 1, rtc_initial.hour, 1, rtc_initial.minute, 1, rtc_initial.seccond + 20);
	
	uint32_t h, m, s;
	char timeBuffer[512];
	
	int t = 10;
	
	int counter = 0;
	char stringCounter[10][11] = {"*---------", "**--------", "***-------", "****------", "*****-----", 
								  "******----", "*******---", "********--", "*********-", "**********"};
	
	/* Insert application code here, after the board has been initialized. */
	while(1) {
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		
		if(flag_rtc){
			rtc_get_time(RTC, &h, &m, &s);
			sprintf(timeBuffer, "%2d:%2d:%2d", h, m, s);
			
			gfx_mono_draw_string(timeBuffer, 0, 16, &sysfont);
			flag_rtc = 0;
		}
		
		if (rtt_irq) {
			gfx_mono_draw_string(stringCounter[counter], 0, 0, &sysfont);
			
			counter += 1;
			if (counter > 9) {
				counter = 0;
			}
			rtt_irq = 0;
		}
		
		if (f_rtt_alarme){
      
		  /*
		   * IRQ apos 4s -> 8*0.5
		   */
		  uint16_t pllPreScale = (int) (((float) 32768) / 4.0);
		  uint32_t irqRTTvalue = 10;
      
		  // reinicia RTT para gerar um novo IRQ
		  RTT_init(pllPreScale, irqRTTvalue);
      
		  f_rtt_alarme = false;
		}
		
		if (flag_leds){
			if(flag_tc0 && but_flag1) {
				pin_toggle(LED1_PIO, LED1_IDX_MASK);
				flag_tc0 = 0;
			}
			
			if(flag_tc1 && but_flag2){
				pin_toggle(LED2_PIO, LED2_IDX_MASK);
				flag_tc1 = 0;
			}
			
			if(flag_tc2 && but_flag3){
				pin_toggle(LED3_PIO, LED3_IDX_MASK);
				flag_tc2 = 0;
			}
		}
	}
}
