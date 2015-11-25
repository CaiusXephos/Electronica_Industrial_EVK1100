#include <board.h>
#include <compiler.h>
#include <dip204.h>
#include <intc.h>
#include <gpio.h>
#include <adc.h>
#include <pm.h>
#include <delay.h>
#include <spi.h>
#include <conf_clock.h>
#include <pwm.h>
#include <stdio.h>



#define PWM_PIN_3								54 //FORWARD
#define PWM_PIN_Function						0
#define PWM_ID_3								3
#define PWM_PIN_1								52 //REVERSE
#define PWM_ID_1								1

#define EXAMPLE_ADC_POTENTIOMETER_CHANNEL		1
#define EXAMPLE_ADC_POTENTIOMETER_PIN			AVR32_ADC_AD_1_PIN
#define EXAMPLE_ADC_POTENTIOMETER_FUNCTION		AVR32_ADC_AD_1_FUNCTION
#define FOSC0									12000000
#define MAX_VAL									1023
#define MIN_VAL									0
#define DIV		
#define BOTON_DIRECCION							GPIO_PUSH_BUTTON_0
//FUNCIONES
void clear_Display(void);
void init_PM(void);
void fill_Display(void);
void init_LCD(void);
void init_Potentiometer(void);
void clear_Line(int line);
void set_Direccion(int direccion);
void set_Velocidad(int velocidad);
void set_Duty_Cycle(int valor_pot);
void init_PWM(int duty, int direccion);
void update_PWM(int velocidad, int direccion);
void init_INTC(void);
void init_CurrentSensor(void);
void set_Current(int current);
__attribute__((__interrupt__))
static void handler_interrupt(void);
//VARIABLES GLOBALES
signed short int adc_value_pot = 0, adc_value_current=0;
int direccion = 1, bandera = 0, velocidad=9, duty_cycle = 0;

int main(void)
{
	init_PM();
	init_LCD();
	init_Potentiometer();
	init_CurrentSensor();
	init_INTC();
	fill_Display();
	set_Velocidad(velocidad);
	set_Direccion(direccion);
	init_PWM(velocidad,direccion);
	while (1)
	{
		adc_start(&AVR32_ADC);
		adc_value_pot = adc_get_value(&AVR32_ADC,EXAMPLE_ADC_POTENTIOMETER_CHANNEL);
		adc_start(&AVR32_ADC);
		adc_value_current = adc_get_value(&AVR32_ADC,3);
		set_Duty_Cycle(adc_value_pot);
		set_Current(adc_value_current);
		if(bandera ==1){
			set_Velocidad(velocidad);
			set_Direccion(direccion);
			update_PWM(velocidad, direccion);
		}
		delay_ms(400);
	}//WHILE
}//MAIN

void clear_Display(void){
	for (int i = 1; i<5;i++){
		for(int j = 1;j<21;j++){
			dip204_set_cursor_position(j,i);
			dip204_write_string(" ");
		}
	}
}
void fill_Display(void){
	dip204_set_cursor_position(2,1);
	dip204_write_string("Control  Velocidad");
	dip204_set_cursor_position(1,2);
	dip204_write_string("Direccion:");
	dip204_set_cursor_position(1,3);
	dip204_write_string("Velocidad:");
}
void init_PM(void){
	pm_switch_to_osc0(&AVR32_PM,FOSC0,AVR32_PM_OSCCTRL0_STARTUP_4096_RCOSC);
	delay_init(FOSC0);
}
void init_LCD(void){
	 static const gpio_map_t DIP204_SPI_GPIO_MAP =
	 {
		 {DIP204_SPI_SCK_PIN,  DIP204_SPI_SCK_FUNCTION },  // SPI Clock.
		 {DIP204_SPI_MISO_PIN, DIP204_SPI_MISO_FUNCTION},  // MISO.
		 {DIP204_SPI_MOSI_PIN, DIP204_SPI_MOSI_FUNCTION},  // MOSI.
		 {DIP204_SPI_NPCS_PIN, DIP204_SPI_NPCS_FUNCTION}   // Chip Select NPCS.
	 };
	 spi_options_t spiOptions =
	 {
		 .reg          = DIP204_SPI_NPCS,
		 .baudrate     = 1000000,
		 .bits         = 8,
		 .spck_delay   = 0,
		 .trans_delay  = 0,
		 .stay_act     = 1,
		 .spi_mode     = 0,
		 .modfdis      = 1
	 };
	 gpio_enable_module(DIP204_SPI_GPIO_MAP,
	 sizeof(DIP204_SPI_GPIO_MAP) / sizeof(DIP204_SPI_GPIO_MAP[0]));
	  spi_initMaster(DIP204_SPI, &spiOptions);
	  spi_selectionMode(DIP204_SPI, 0, 0, 0);
	  spi_enable(DIP204_SPI);
	  spi_setupChipReg(DIP204_SPI, &spiOptions, FOSC0);
	  dip204_init(backlight_PWM, true);
	  clear_Display();
	  dip204_hide_cursor();
}
void init_Potentiometer(void){
	const gpio_map_t ADC_GPIO_MAP = {
		{EXAMPLE_ADC_POTENTIOMETER_PIN, EXAMPLE_ADC_POTENTIOMETER_FUNCTION}
	};
	gpio_enable_module(ADC_GPIO_MAP, sizeof(ADC_GPIO_MAP) /
	sizeof(ADC_GPIO_MAP[0]));
	AVR32_ADC.mr |= 0x1 << AVR32_ADC_MR_PRESCAL_OFFSET;
	adc_configure(&AVR32_ADC);
	adc_enable(&AVR32_ADC, EXAMPLE_ADC_POTENTIOMETER_CHANNEL);
	adc_start(&AVR32_ADC);
}
void init_CurrentSensor(void){
	#define AVR32_ADC_AD_1_PIN 22
	const gpio_map_t ADC_GPIO_MAP = {
		{AVR32_ADC_AD_3_PIN, AVR32_ADC_AD_3_FUNCTION}
	};
	gpio_enable_module(ADC_GPIO_MAP, sizeof(ADC_GPIO_MAP) /
	sizeof(ADC_GPIO_MAP[0]));
	AVR32_ADC.mr |= 0x1 << AVR32_ADC_MR_PRESCAL_OFFSET;
	adc_configure(&AVR32_ADC);
	adc_enable(&AVR32_ADC, 3);
	adc_start(&AVR32_ADC);
}
void clear_Line(int line){
	for(int i = 0; i<21;i++){
		dip204_set_cursor_position(i,line);
		dip204_write_string(" ");
	}
}
void set_Direccion(int direccion){
	if(direccion == 1){
		clear_Line(2);
		dip204_set_cursor_position(1,2);
		dip204_write_string("Direccion:");
		dip204_set_cursor_position(12,2);
		dip204_write_string("Forward");
	}
	if(direccion == 0){
		clear_Line(2);
		dip204_set_cursor_position(1,2);
		dip204_write_string("Direccion:");
		dip204_set_cursor_position(12,2);
		dip204_write_string("Reverse");
	}
}
void set_Velocidad(int velocidad){
	clear_Line(3);
	dip204_set_cursor_position(1,3);
	dip204_write_string("Velocidad:");
	dip204_set_cursor_position(12,3);
	switch (velocidad){
		case 1:
			dip204_write_string("1");
			break;
		case 2:
			dip204_write_string("2");
			break;
		case 3:
			dip204_write_string("3");
			break;
		case 4:
			dip204_write_string("4");
			break;
		case 5:
			dip204_write_string("5");
			break;
		case 6:
			dip204_write_string("6");
			break;
		case 7:
			dip204_write_string("7");
			break;
		case 8:
			dip204_write_string("8");
			break;
		case 9:
			dip204_write_string("9");
			break;
		case 10:
		dip204_write_string("10");
		break;
	}//SWITCH
}
void set_Duty_Cycle(int valor_pot){
	int limite_inf = 0, limite_sup = 120, anterior = velocidad;
	for(int i = 0;i<10;i++){
		limite_inf = 0+(i*120);
		limite_sup = 120+(i*120);
		if(valor_pot>=limite_inf && valor_pot<=limite_sup){
			duty_cycle = i+1;
			velocidad = i+1;
		}	
	}//FOR
	if(velocidad !=anterior){
		bandera = 1;
	}
}
void init_PWM(int velocidad, int direccion){
	int auxiliar;
	switch(velocidad){
		case 9:
			auxiliar = 2;
			break;
		case 8:
			auxiliar = 4;
			break;
		case 7:
			auxiliar = 6;
			break;
		case 6:
			auxiliar = 8;
			break;
		case 5:
			auxiliar = 10;
			break;
		case 4:
			auxiliar = 12;
			break;
		case 3:
			auxiliar = 14;
			break;
		case 2:
			auxiliar = 16;
			break;
		case 1:
			auxiliar = 18;
			break;
	}//SWITCH
	pwm_opt_t pwm_opt =
	{
		.diva = AVR32_PWM_DIVA_CLK_OFF,
		.divb = AVR32_PWM_DIVB_CLK_OFF,
		.prea = AVR32_PWM_PREA_MCK,
		.preb = AVR32_PWM_PREB_MCK
	};
	
	avr32_pwm_channel_t pwm_channel = { .ccnt = 0 };
		pwm_channel.cdty = auxiliar; /* Channel duty cycle, should be < CPRD. */
		pwm_channel.cprd = 20; /* Channel period. */
		pwm_channel.cupd = 0; /* Channel update is not used here. */
		pwm_channel.CMR.calg = PWM_MODE_LEFT_ALIGNED; /* Channel mode. */
		pwm_channel.CMR.cpol = PWM_POLARITY_LOW;      /* Channel polarity. */
		pwm_channel.CMR.cpd = PWM_UPDATE_DUTY;        /* Not used the first time. */
		pwm_channel.CMR.cpre = AVR32_PWM_CPRE_MCK_DIV_256; /* Channel prescaler. */
		
		gpio_enable_module_pin(PWM_PIN_3, PWM_PIN_Function);
		gpio_enable_module_pin(PWM_PIN_1, PWM_PIN_Function);
		pwm_init(&pwm_opt);
		if(direccion == 1){//FORWARD
			pwm_channel_init(PWM_ID_3, &pwm_channel);
			pwm_start_channels(1 << PWM_ID_3);

		}
		if(direccion == 0){//REVERSE
			pwm_channel_init(PWM_ID_1, &pwm_channel);
			pwm_start_channels(1 << PWM_ID_1);
		}
}
void update_PWM(int velocidad, int direccion){
	int auxiliar;
	switch(velocidad){
		case 9:
		auxiliar = 2;
		break;
		case 8:
		auxiliar = 4;
		break;
		case 7:
		auxiliar = 6;
		break;
		case 6:
		auxiliar = 8;
		break;
		case 5:
		auxiliar = 10;
		break;
		case 4:
		auxiliar = 12;
		break;
		case 3:
		auxiliar = 14;
		break;
		case 2:
		auxiliar = 16;
		break;
		case 1:
		auxiliar = 18;
		break;
	}//SWITCH
		avr32_pwm_channel_t pwm_channel = { .ccnt = 0 };
		pwm_channel.cdty = auxiliar; /* Channel duty cycle, should be < CPRD. */
		pwm_channel.cprd = 20; /* Channel period. */
		pwm_channel.cupd = 0; /* Channel update is not used here. */
		pwm_channel.CMR.calg = PWM_MODE_LEFT_ALIGNED; /* Channel mode. */
		pwm_channel.CMR.cpol = PWM_POLARITY_LOW;      /* Channel polarity. */
		pwm_channel.CMR.cpd = PWM_UPDATE_DUTY;        /* Not used the first time. */
		pwm_channel.CMR.cpre = AVR32_PWM_CPRE_MCK_DIV_256; /* Channel prescaler. */
	
	if(direccion == 1){//FORWARD
		pwm_stop_channels(1<<PWM_ID_1);
		pwm_stop_channels(1<<PWM_ID_3);
		pwm_channel_init(PWM_ID_3, &pwm_channel);
		pwm_start_channels(1 << PWM_ID_3);
		
	}
	if(direccion == 0){//REVERSE
		pwm_stop_channels(1<<PWM_ID_1);
		pwm_stop_channels(1<<PWM_ID_3);
		pwm_channel_init(PWM_ID_1, &pwm_channel);
		pwm_start_channels(1 << PWM_ID_1);
	}
	bandera = 0;
}
static void handler_interrupt(void){
	if (gpio_get_pin_interrupt_flag(GPIO_JOYSTICK_UP)){
		direccion = 1;
		gpio_clear_pin_interrupt_flag(GPIO_JOYSTICK_UP);
	}
	if (gpio_get_pin_interrupt_flag(GPIO_JOYSTICK_DOWN)){
		direccion = 0;
		gpio_clear_pin_interrupt_flag(GPIO_JOYSTICK_DOWN);
	}
	bandera = 1;
}
void init_INTC(void){
	gpio_enable_pin_interrupt(GPIO_JOYSTICK_UP , GPIO_FALLING_EDGE);
	gpio_enable_pin_interrupt(GPIO_JOYSTICK_DOWN , GPIO_FALLING_EDGE);
	Disable_global_interrupt();
	INTC_init_interrupts();
	  INTC_register_interrupt( &handler_interrupt, AVR32_GPIO_IRQ_0 + (GPIO_JOYSTICK_UP/8), AVR32_INTC_INT1);
	  INTC_register_interrupt( &handler_interrupt, AVR32_GPIO_IRQ_0 + (GPIO_JOYSTICK_DOWN/8), AVR32_INTC_INT1);
	Enable_global_interrupt();
}
void set_Current(int current){
	clear_Line(4);
	//delay_ms(200);
	char number[5];
	sprintf(number,"%04d",current);
	dip204_set_cursor_position(1,4);
	dip204_write_string(number);
	
}
