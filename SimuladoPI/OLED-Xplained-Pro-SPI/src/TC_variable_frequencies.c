
/*
Para usar, renomear esse arquivo para main, e renomear o outro main
 */ 

#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"
/// -------------------------------///
/// DEFINICOES ////

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
/// -------------------------------///

/// definicoes de volatile//
/// -------------------------------///
volatile char but_flag1 = 0;
volatile char but_flag2 = 1;
volatile char but_flag3 = 0;
volatile char flag_tc0 = 0;
/// -------------------------------///

/// voids//
/// -------------------------------///
void but1_callback(void);
void but2_callback(void);
void but3_callback(void);
void BUTs_init(void);
void LEDs_init(void);
void pin_toggle(Pio *pio, uint32_t mask);
void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq);
/// -------------------------------///
// CALL BACKS
/// -------------------------------///

void but1_callback(void)
{
	but_flag1 = 1;
}

void but2_callback(void)
{
	but_flag2 = !but_flag2;
}

void but3_callback(void)
{
	but_flag3 = 1;
}

void LEDs_init(void)
{
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_IDX_MASK, PIO_DEFAULT);

	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_IDX_MASK, PIO_DEFAULT);

	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_configure(LED3_PIO, PIO_OUTPUT_0, LED3_IDX_MASK, PIO_DEFAULT);
};

void BUTs_init(void)
{
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

void TC0_Handler(void)
{
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	flag_tc0 = 1;
}

void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq)
{
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
	NVIC_EnableIRQ((IRQn_Type)ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

void pin_toggle(Pio *pio, uint32_t mask)
{
	if (pio_get_output_data_status(pio, mask))
		pio_clear(pio, mask);
	else
		pio_set(pio, mask);
}

int main(void)
{
	board_init();
	sysclk_init();
	delay_init();
	LEDs_init();
	BUTs_init();

	WDT->WDT_MR = WDT_MR_WDDIS;
	// Init OLED
	gfx_mono_ssd1306_init();

	// Escreve na tela um circulo e um texto
	//gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);

	but_flag1 = 0;
	but_flag2 = 1;
	but_flag3 = 0;

	char Buffer[512];

	int i = 5;
	TC_init(TC0, ID_TC0, 0, i);
	sprintf(Buffer, "%2d Hz", i);
	gfx_mono_draw_string(Buffer, 50, 16, &sysfont);
	/* Insert application code here, after the board has been initialized. */
	while (1)
	{
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		if (but_flag1)
		{
			sprintf(Buffer, "%2d Hz", i);
			gfx_mono_draw_string(Buffer, 50, 16, &sysfont);
			i++;
			if (i > 15)
			{
				i = 15;
			}
			TC_init(TC0, ID_TC0, 0, i);
			but_flag1 = 0;
		}

		if (but_flag3)
		{
			sprintf(Buffer, "%2d Hz", i);
			gfx_mono_draw_string(Buffer, 50, 16, &sysfont);
			i--;
			if (i < 1)
			{
				i = 1;
			}
			TC_init(TC0, ID_TC0, 0, i);
			but_flag3 = 0;
		}

		if (flag_tc0 && but_flag2)
		{
			pin_toggle(LED2_PIO, LED2_IDX_MASK);
			flag_tc0 = 0;
		}
	}
}
