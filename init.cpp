/****************************************************************************
**	INCLUDES
****************************************************************************/

#include "global.h"

/****************************************************************************
**	MACROS
****************************************************************************/

//Configure a protected register in the initialization phase
//CPP: Writing in this register allows operation on IO protected registers for the next 4CLK
#define INIT_CONFIG_PROTECTED( target_register, value_mask, value_position, value )	\
	CCP = CCP_IOREG_gc, (target_register) = ( ((target_register) & ~(value_mask)) | (((value) << (value_position)) & (value_mask))  )


/****************************************************************************
**	FUNCTIONS PROTOTYPES
****************************************************************************/

//Initialize the micro controller fuses
extern void init_fuses( void );
//Initialize clock systems
extern void init_clock( void );
//Initialize pin map
extern void init_pin( void );
//Initialize RTC timer as periodic interrupt
extern void init_rtc( void );
//Initialize timer type A. AT4809 has a single of such timers.
extern void init_timer0a_split( void );
//setup one of four timers type B of the AT4809 as PWM generator
extern void init_timer_b( TCB_t &timer );
//Initialize one of four USART transceivers
extern void init_uart( USART_t &usart );
//Initialize port multiplexer for alternate functions
extern void init_mux( void );



/****************************************************************************
**	FUNCTIONS DECLARATIONS
****************************************************************************/

/****************************************************************************
**  Function
**  init |
****************************************************************************/
//! @return bool |
//! @brief main init function
//! @details initialize all embedded peripherals and pins.
//!	remember to call the initialization functions in the main init
/***************************************************************************/

void init(void)
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Initialize clock systems
	init_clock();
	
	//initialize pin configuration
	init_pin();
	
	//Initialize port multiplexer to set alternate pin functions
	init_mux();
	
	//Initialize RTC timer as Periodic interrupt source: RTC_PIT_vect
	init_rtc();
	
	//Initialize timer type A
	init_timer0a_split();
	
	//Initialize four timers type B as 20KHz 8bit PWM generators for the VNH7040 Motor drivers
	init_timer_b( TCB0 );
	init_timer_b( TCB1 );
	init_timer_b( TCB2 );
	init_timer_b( TCB3 );

	//Initialize USART 3 as async UART 256.4Kb/s
	init_uart( USART3 );

	//Activate interrupts
	sei();

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return;
}	//End: init

/****************************************************************************
**  Function
**  init_clock |
****************************************************************************/
//! @brief initialize clock systems
//! @details setup the clock system multiplexers and the clock output
//! Clock source=internal oscillator 20MHz
//! CLK_PER = 20MHz
/***************************************************************************/

void init_clock( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Main clock switch
	INIT_CONFIG_PROTECTED( CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_gm, CLKCTRL_CLKSEL_gp, (CLKCTRL_CLKSEL_t)CLKCTRL_CLKSEL_OSC20M_gc);
	//Configure CLK_OUT pin as disabled
	INIT_CONFIG_PROTECTED( CLKCTRL.MCLKCTRLA, CLKCTRL_CLKOUT_bm, CLKCTRL_CLKOUT_bp, 0);
	//Disable the main clock prescaler
	INIT_CONFIG_PROTECTED( CLKCTRL.MCLKCTRLB, CLKCTRL_PEN_bm, CLKCTRL_PEN_bm, 0);
	//Set the main clock prescaler to 2
	INIT_CONFIG_PROTECTED( CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_gm, CLKCTRL_PDIV_gp, (CLKCTRL_PDIV_t)CLKCTRL_PDIV_2X_gc);
	//Disable the clock multiplexer and prescaler protection
	INIT_CONFIG_PROTECTED( CLKCTRL.MCLKLOCK, CLKCTRL_LOCKEN_bm, CLKCTRL_LOCKEN_bp, 0);

	CLKCTRL.OSC20MCTRLA |= CLKCTRL_RUNSTDBY_bm; //1<<1;
	CCP = CCP_IOREG_gc;
	CLKCTRL.OSC20MCALIBB |= CLKCTRL_LOCK_bm; //1<<7

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return;
}	//End: init_clock

/****************************************************************************
**  Function
**  init_pin |
****************************************************************************/
//! @brief initialize pin configuration
//! @details Initialize pin configuration and multiplexers
/***************************************************************************/

void init_pin( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------
	//

	//----------------------------------------------------------------
	//!	PORTA
	//!	PA0				:
	//!	PA1				:
	//!	PA2, TCB0		: DRV0_PWM
	//!	PA3, TCB1		: DRV1_PWM
	//!	PA4				: DRV0_CTRLA
	//!	PA5				: DRV0_CTRLB
	//!	PA6				: DRV1_CTRLA
	//!	PA7				: DRV1_CTRLB
	//----------------------------------------------------------------
	//				0		1		2		3		4		5		6		7
	PORT_A_CONFIG(	PIN_Z,	PIN_Z,	PIN_L,	PIN_L,	PIN_L,	PIN_L,	PIN_L,	PIN_L );

	//----------------------------------------------------------------
	//!	PORTB
	//!	PB0, UART3		: uC_TXO
	//!	PB1, UART3		: uC_RXI
	//!	PB2				: DRV2_CTRLA
	//!	PB3				: DRV2_CTRLB
	//!	PB4, TCB2		: DRV2_PWM
	//!	PB5, TCB3		: DRV3_PWM
	//!	PB6				:
	//!	PB7				:
	//----------------------------------------------------------------
	//				0		1		2		3		4		5		6		7
	PORT_B_CONFIG(	PIN_H,	PIN_Z,	PIN_L,	PIN_L,	PIN_L,	PIN_L,	PIN_Z,	PIN_Z );

	//----------------------------------------------------------------
	//!	PORTC
	//!	PC0				: ENC0_CHA
	//!	PC1				: ENC0_CHB
	//!	PC2				: ENC1_CHA
	//!	PC3				: ENC1_CHB 
	//!	PC4				: ENC2_CHA
	//!	PC5				: ENC2_CHB
	//!	PC6				: ENC3_CHA
	//!	PC7				: ENC3_CHB
	//----------------------------------------------------------------
	//				0		1		2		3		4		5		6		7
	PORT_C_CONFIG(	PIN_IE,	PIN_IE,	PIN_IE,	PIN_IE,	PIN_IE,	PIN_IE,	PIN_IE,	PIN_IE );

	//----------------------------------------------------------------
	//!	PORTD
	//!	PD0, ADC		: DRV0_SENSE
	//!	PD1, ADC		: DRV1_SENSE
	//!	PD2, ADC		: DRV2_SENSE
	//!	PD3, ADC		: DRV3_SENSE
	//!	PD4				:
	//!	PD5				:
	//!	PD6				: DRV3_CTRLA
	//!	PD7				: DRV3_CTRLB
	//----------------------------------------------------------------
	//				0		1		2		3		4		5		6		7
	PORT_D_CONFIG(	PIN_Z,	PIN_Z,	PIN_Z,	PIN_Z,	PIN_Z,	PIN_Z,	PIN_L,	PIN_L );

	//----------------------------------------------------------------
	//!	PORTE
	//!	PE0				:
	//!	PE1				:
	//!	PE2				:
	//!	PE3				:
	//!	PE4				:
	//!	PE5				:
	//!	PE6				:
	//!	PE7				:
	//----------------------------------------------------------------
	//				0		1		2		3		4		5		6		7
	PORT_E_CONFIG(	PIN_Z,	PIN_Z,	PIN_Z,	PIN_Z,	PIN_Z,	PIN_Z,	PIN_Z,	PIN_Z );

	//----------------------------------------------------------------
	//!	PORTF
	//!	PF0				: DRV_SEN Driver SENSE Enable
	//!	PF1				: DRV_DIAG Driver Diagnostic enable
	//!	PF2				:
	//!	PF3				:
	//!	PF4				: 
	//!	PF5				: Curiosity Nano LED
	//!	PF6				:
	//!	PF7				:
	//----------------------------------------------------------------
	//				0		1		2		3		4		5		6		7
	PORT_F_CONFIG(	PIN_L,	PIN_L,	PIN_Z,	PIN_Z,	PIN_Z,	PIN_L,	PIN_Z,	PIN_Z );

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return;
}	//End: init_pin

/****************************************************************************
**  Function
**  init_mux |
****************************************************************************/
//! @brief Initialize port multiplexer for alternate functions
//! @details
/***************************************************************************/

void init_mux( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//! Fetch registers
	uint8_t tcb_tmp		= PORTMUX.TCBROUTEA;
	//! Set the configuration for the USART port router
	uint8_t usart_tmp		= PORTMUX.USARTROUTEA;
	
	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

		//----------------------------------------------------------------
		//	Four Timers Type B
		//----------------------------------------------------------------

		//! TCB
	//! Route the output of the Timers Type B PWM generators
	//SET_BIT( tcb_tmp, PORTMUX_TCB0_bp );
	//SET_BIT( tcb_tmp, PORTMUX_TCB1_bp );
	SET_BIT( tcb_tmp, PORTMUX_TCB2_bp );
	//SET_BIT( tcb_tmp, PORTMUX_TCB3_bp );
	
		//----------------------------------------------------------------
		//	Route the USART transceivers
		//----------------------------------------------------------------

		//! USART0
	//Disconnected
	SET_MASKED_BIT( usart_tmp, PORTMUX_USART0_gm, PORTMUX_USART0_NONE_gc );
	//PA[3:0]
	//SET_MASKED_BIT( usart_tmp, PORTMUX_USART0_gm, PORTMUX_USART0_DEFAULT_gc );
	//PA[7:4]
	//SET_MASKED_BIT( usart_tmp, PORTMUX_USART0_gm, PORTMUX_USART0_ALT1_gc );
		
		//! USART1
	//Disconnected
	SET_MASKED_BIT( usart_tmp, PORTMUX_USART1_gm, PORTMUX_USART1_NONE_gc );
	//PC[3:0]
	//SET_MASKED_BIT( usart_tmp, PORTMUX_USART1_gm, PORTMUX_USART1_DEFAULT_gc );
	//PC[7:4]
	//SET_MASKED_BIT( usart_tmp, PORTMUX_USART1_gm, PORTMUX_USART1_ALT1_gc );
		
		//! USART2
	//Disconnected
	SET_MASKED_BIT( usart_tmp, PORTMUX_USART2_gm, PORTMUX_USART2_NONE_gc );
	//PF[3:0]
	//SET_MASKED_BIT( usart_tmp, PORTMUX_USART2_gm, PORTMUX_USART2_DEFAULT_gc );
	//PF[7:4]
	//SET_MASKED_BIT( usart_tmp, PORTMUX_USART2_gm, PORTMUX_USART2_ALT1_gc );
		
		//! USART3
	//Disconnected
	//SET_MASKED_BIT( usart_tmp, PORTMUX_USART3_gm, PORTMUX_USART3_NONE_gc );
	//PB[3:0]
	SET_MASKED_BIT( usart_tmp, PORTMUX_USART3_gm, PORTMUX_USART3_DEFAULT_gc );
	//PB[7:4]
	//SET_MASKED_BIT( usart_tmp, PORTMUX_USART3_gm, PORTMUX_USART3_ALT1_gc );


	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	PORTMUX.TCBROUTEA = tcb_tmp;
	PORTMUX.USARTROUTEA = usart_tmp;

	return;
}	//End: init_mux

/****************************************************************************
**  Function
**  init_rtc |
****************************************************************************/
//! @brief Initialize RTC timer as periodic interrupt
//! @details
//! Interrupt vectors:
//!		RTC_CNT_vect
//!		RTC_PIT_vect
/***************************************************************************/

void init_rtc( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//! Fetch registers
	uint8_t ctrla_tmp		= RTC.CTRLA;
	uint8_t intctrl_tmp		= RTC.INTCTRL;
	uint8_t dgbctrl_tmp		= RTC.DBGCTRL;
	uint8_t clksel_tmp		= RTC.CLKSEL;
	uint8_t pitctrla_tmp	= RTC.PITCTRLA;
	uint8_t pitintctrl_tmp	= RTC.PITINTCTRL;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//! Enable the RTC timer
	SET_BIT( ctrla_tmp, RTC_RTCEN_bp );

	//! Enable the RTC Correction
	//SET_BIT( ctrla_tmp, 2 );

	//! Let the RTC timer run in standby mode
	SET_BIT( ctrla_tmp, RTC_RUNSTDBY_bp );

	//! Let the RTC run in debug when CPU is halted
	//SET_BIT( dgbctrl_tmp, RTC_DBGRUN_bp );

	//----------------------------------------------------------------
	//! RTC Clock Source
	//----------------------------------------------------------------
	//	Clock source for the RTC timer. Select only one

	SET_MASKED_BIT( clksel_tmp, RTC_CLKSEL_gm, RTC_CLKSEL_INT32K_gc );
	//SET_MASKED_BIT( clksel_tmp, RTC_CLKSEL_gm, RTC_CLKSEL_INT1K_gc );
	//SET_MASKED_BIT( clksel_tmp, RTC_CLKSEL_gm, RTC_CLKSEL_TOSC32K_gc );
	//SET_MASKED_BIT( clksel_tmp, RTC_CLKSEL_gm, RTC_CLKSEL_EXTCLK_gc );

	//----------------------------------------------------------------
	//! RTC Clock Prescaler
	//----------------------------------------------------------------
	//	Set prescaler. Only activate one

	SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV1_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV2_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV4_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV8_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV16_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV32_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV64_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV128_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV256_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV512_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV1024_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV2048_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV4096_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV8192_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV16384_gc );
	//SET_MASKED_BIT( ctrla_tmp, RTC_PRESCALER_gm, RTC_PRESCALER_DIV32768_gc );

	//----------------------------------------------------------------
	//! RTC Periodic Interrupt period
	//----------------------------------------------------------------

	//! Enable Periodic Interrupt timer
	SET_BIT( pitctrla_tmp, RTC_PITEN_bp );

	//! Period for the periodic interrupt. Activate only one
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_OFF_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC4_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC8_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC16_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC32_gc );
	SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC64_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC128_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC256_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC512_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC1024_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC2048_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC4096_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC8192_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC16384_gc );
	//SET_MASKED_BIT( pitctrla_tmp, RTC_PERIOD_gm, RTC_PERIOD_CYC32768_gc );

	//----------------------------------------------------------------
	//! RTC Interrupts
	//----------------------------------------------------------------

	//! Enable overflow interrupt
	//SET_BIT( intctrl_tmp, RTC_OVF_bp );
	//! Enable Compare Match interrupt
	//SET_BIT( intctrl_tmp, RTC_CMP_bp );
	//! Enable Periodic Interrupt timer
	SET_BIT( pitintctrl_tmp, RTC_PI_bp );


	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	//Calibration PPM of the RTC counter. Meant to be done in software against more precise clock sources.
	RTC.CALIB = (uint8_t)0x00;

	//Wait for the ***
	//while (IS_BIT_ONE(RTC.STATUS, RTC_PERBUSY_bp));
	RTC.PER = (uint16_t)0;
	//Compare register for compare interrupt
	RTC.CMP = (uint16_t)0;

	//! Registers write back
	//Configuration registers
	RTC.DBGCTRL = dgbctrl_tmp;
	RTC.CLKSEL = clksel_tmp;
	RTC.PITCTRLA = pitctrla_tmp;
	//Write this register last as it activates the timer
	RTC.CTRLA = ctrla_tmp;
	//Activate interrupts
	RTC.INTCTRL = intctrl_tmp;
	RTC.PITINTCTRL = pitintctrl_tmp;

	return;
}	//End: init_rtc

/****************************************************************************
**  Function
**  init_timer_a |
****************************************************************************/
//! @brief initialize timer type a in split mode as two 8bit timers with six compare channels
//! @details setup the only timer type A of the AT4809
//!
//!	Clock from event control is disabled in SPLIT mode
//!
//! Interrupt vectors available:
//! TCA0_LUNF_vect
//! TCA0_OVF_vect
//! TCA0_HUNF_vect
//! TCA0_LCMP0_vect
//! TCA0_CMP0_vect
//! TCA0_CMP1_vect
//! TCA0_LCMP1_vect
//! TCA0_LCMP2_vect
//! TCA0_CMP2_vect
/***************************************************************************/

void init_timer0a_split( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//Load temporary registers
	uint8_t ctrla_tmp			= TCA0.SPLIT.CTRLA;
	uint8_t ctrlb_tmp			= TCA0.SPLIT.CTRLB;
	uint8_t ctrlc_tmp			= TCA0.SPLIT.CTRLC;
	uint8_t ctrld_tmp			= TCA0.SPLIT.CTRLD;
	uint8_t ctrle_tmp			= TCA0.SPLIT.CTRLESET;
	uint8_t dbgctrl_tmp			= TCA0.SPLIT.DBGCTRL;
	uint8_t port_mux_tca0_tmp	= PORTMUX.TCAROUTEA;
	uint8_t intctrl_tmp			= TCA0.SPLIT.INTCTRL;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

		//----------------------------------------------------------------
		//! Enable Split Mode
		//----------------------------------------------------------------
		//	Function of registers change according to the mode.
		//	0 = 3x 16bit
		//	1 = 6x 8bit

	SET_BIT( ctrld_tmp, TCA_SPLIT_SPLITM_bp );

		//----------------------------------------------------------------
		//! Enable TCA
		//----------------------------------------------------------------
		//	0 = disabled
		//	1 = enabled

	SET_BIT( ctrla_tmp, TCA_SPLIT_ENABLE_bp );

		//----------------------------------------------------------------
		//! TCA Clock Prescaler
		//----------------------------------------------------------------
		//	Set the clock prescaler of this TCA. Activate only one value

	//SET_MASKED_BIT( ctrla_tmp, TCA_SPLIT_CLKSEL_gm , TCA_SPLIT_CLKSEL_DIV1_gc );
	//SET_MASKED_BIT( ctrla_tmp, TCA_SPLIT_CLKSEL_gm , TCA_SPLIT_CLKSEL_DIV2_gc );
	SET_MASKED_BIT( ctrla_tmp, TCA_SPLIT_CLKSEL_gm , TCA_SPLIT_CLKSEL_DIV4_gc );
	//SET_MASKED_BIT( ctrla_tmp, TCA_SPLIT_CLKSEL_gm , TCA_SPLIT_CLKSEL_DIV8_gc );
	//SET_MASKED_BIT( ctrla_tmp, TCA_SPLIT_CLKSEL_gm , TCA_SPLIT_CLKSEL_DIV16_gc );
	//SET_MASKED_BIT( ctrla_tmp, TCA_SPLIT_CLKSEL_gm , TCA_SPLIT_CLKSEL_DIV64_gc );
	//SET_MASKED_BIT( ctrla_tmp, TCA_SPLIT_CLKSEL_gm , TCA_SPLIT_CLKSEL_DIV256_gc );
	//SET_MASKED_BIT( ctrla_tmp, TCA_SPLIT_CLKSEL_gm , TCA_SPLIT_CLKSEL_DIV1024_gc );

		//----------------------------------------------------------------
		//! TCA Enable compare output and waveform output pin override for each compare channel
		//----------------------------------------------------------------

	//SET_BIT( ctrlb_tmp, TCA_SPLIT_LCMP0EN_bp );
	//SET_BIT( ctrlb_tmp, TCA_SPLIT_LCMP1EN_bp );
	//SET_BIT( ctrlb_tmp, TCA_SPLIT_LCMP2EN_bp );
	//SET_BIT( ctrlb_tmp, TCA_SPLIT_HCMP0EN_bp );
	//SET_BIT( ctrlb_tmp, TCA_SPLIT_HCMP1EN_bp );
	//SET_BIT( ctrlb_tmp, TCA_SPLIT_HCMP2EN_bp );

		//----------------------------------------------------------------
		//! TCA Enable timer commands
		//----------------------------------------------------------------

	//Disable force commands
	SET_MASKED_BIT( ctrle_tmp, (uint8_t)0x03 , (uint8_t)0x00 );
	//ENABLE force commands for both channels
	//SET_MASKED_BIT( ctrle_tmp, (uint8_t)0x03 , (uint8_t)0x03 );

		//----------------------------------------------------------------
		//! ENABLE TCA interrupts
		//----------------------------------------------------------------

	//Underflow of low counter
	//SET_BIT( intctrl_tmp, TCA_SPLIT_LUNF_bp );
	//Underflow of high counter
	//SET_BIT( intctrl_tmp, TCA_SPLIT_HUNF_bp );
	//Compare channel
	//SET_BIT( intctrl_tmp, TCA_SPLIT_LCMP0_bp );
	//SET_BIT( intctrl_tmp, TCA_SPLIT_LCMP1_bp );
	//SET_BIT( intctrl_tmp, TCA_SPLIT_LCMP2_bp );

		//----------------------------------------------------------------
		//! ENABLE TCA debug
		//----------------------------------------------------------------

	SET_BIT( dbgctrl_tmp, TCA_SPLIT_DBGRUN_bp );

		//----------------------------------------------------------------
		//! Set TCA waveform outputs to a given port. You can activate only one port
		//----------------------------------------------------------------

	//SET_MASKED_BIT( port_mux_tca0_tmp, PORTMUX_TCA0_gm, PORTMUX_TCA0_PORTA_gc );	//default
	SET_MASKED_BIT( port_mux_tca0_tmp, PORTMUX_TCA0_gm, PORTMUX_TCA0_PORTB_gc );
	//SET_MASKED_BIT( port_mux_tca0_tmp, PORTMUX_TCA0_gm, PORTMUX_TCA0_PORTC_gc );
	//SET_MASKED_BIT( port_mux_tca0_tmp, PORTMUX_TCA0_gm, PORTMUX_TCA0_PORTD_gc );
	//SET_MASKED_BIT( port_mux_tca0_tmp, PORTMUX_TCA0_gm, PORTMUX_TCA0_PORTE_gc );
	//SET_MASKED_BIT( port_mux_tca0_tmp, PORTMUX_TCA0_gm, PORTMUX_TCA0_PORTF_gc );

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	//! Register write back.
	//Write back control registers
	TCA0.SPLIT.CTRLB = ctrlb_tmp;
	TCA0.SPLIT.CTRLC = ctrlc_tmp;
	TCA0.SPLIT.CTRLD = ctrld_tmp;
	TCA0.SPLIT.CTRLESET = ctrle_tmp;
	TCA0.SPLIT.DBGCTRL = dbgctrl_tmp;
	//Write back output waveform port selector
	PORTMUX.TCAROUTEA = port_mux_tca0_tmp;

	//Period registers for the High and Low 8bit counters
	TCA0.SPLIT.LPER = (uint8_t)255;
	TCA0.SPLIT.HPER = (uint8_t)255;
	//Initial PWM settings for channels L0 to L2
	TCA0.SPLIT.LCMP0 = (uint8_t)0x00;
	TCA0.SPLIT.LCMP1 = (uint8_t)0x00;
	TCA0.SPLIT.LCMP2 = (uint8_t)0x00;
	//Initial PWM settings for channels H0 to H2
	TCA0.SPLIT.HCMP0 = (uint8_t)0x00;
	TCA0.SPLIT.HCMP1 = (uint8_t)0x00;
	TCA0.SPLIT.HCMP2 = (uint8_t)0x0;

	//Write back control A for last as it's the one that sets the clock and starts the timer
	TCA0.SPLIT.CTRLA = ctrla_tmp;
	//Write back interrupt enable
	TCA0.SPLIT.INTCTRL = intctrl_tmp;

	return;
}	//End: init_timer0a

/****************************************************************************
**  Function
**  init_timer_b | TCB_t &
****************************************************************************/
//! @param timer | TCB_t: one of four timers type B TCB0,TCB1,TCB2,TCB3
//! @brief initialize timer type B in PWM mode
//! @details setup one of four timer type B of the AT4809 as PWM generator
//!	Generates interrupts:
//!	TCB0_INT_vect
//!	TCB1_INT_vect
//!	TCB2_INT_vect
//!	TCB3_INT_vect
//! 
/***************************************************************************/

void init_timer_b( TCB_t &timer )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//! Fetch registers
	uint8_t ctrla_tmp		= timer.CTRLA;
	uint8_t ctrlb_tmp		= timer.CTRLB;
	uint8_t evctrl_tmp		= timer.EVCTRL;
	uint8_t intctrl_tmp		= timer.INTCTRL;
	uint8_t dbgctrl_tmp		= timer.DBGCTRL;
	

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------
	
	//! Enable this timer
	SET_BIT( ctrla_tmp, TCB_ENABLE_bp );
	
	//! This timer will reset whenever TC0 resets
	SET_BIT( ctrla_tmp, TCB_SYNCUPD_bp );
	
	//! Run in standby mode
	//SET_BIT( ctrla_tmp, TCB_RUNSTDBY_bp );
	
	//! Select clock source
	//SET_MASKED_BIT( ctrla_tmp, TCB_CLKSEL_gm, TCB_CLKSEL_CLKDIV1_gc );		// Clock
	//SET_MASKED_BIT( ctrla_tmp, TCB_CLKSEL_gm, TCB_CLKSEL_CLKDIV2_gc );	// Clock/2
	SET_MASKED_BIT( ctrla_tmp, TCB_CLKSEL_gm, TCB_CLKSEL_CLKTCA_gc );		// TCA Clock source
	
	//! Select the mode of operation of this timer
	//SET_MASKED_BIT( ctrlb_tmp, TCB_CNTMODE_gm, TCB_CNTMODE_INT_gc );		// PIT Periodic interrupt mode
	//SET_MASKED_BIT( ctrlb_tmp, TCB_CNTMODE_gm, TCB_CNTMODE_TIMEOUT_gc );	// Periodic Timeout
	//SET_MASKED_BIT( ctrlb_tmp, TCB_CNTMODE_gm, TCB_CNTMODE_CAPT_gc );		// Input Capture Event
	//SET_MASKED_BIT( ctrlb_tmp, TCB_CNTMODE_gm, TCB_CNTMODE_FRQ_gc );		// Input Capture Frequency measurement
	//SET_MASKED_BIT( ctrlb_tmp, TCB_CNTMODE_gm, TCB_CNTMODE_PW_gc );		// Input Capture Pulse-Width measurement
	//SET_MASKED_BIT( ctrlb_tmp, TCB_CNTMODE_gm, TCB_CNTMODE_FRQPW_gc );	// Input Capture Frequency and Pulse-Width measurement
	//SET_MASKED_BIT( ctrlb_tmp, TCB_CNTMODE_gm, TCB_CNTMODE_SINGLE_gc );	// Single Shot
	SET_MASKED_BIT( ctrlb_tmp, TCB_CNTMODE_gm, TCB_CNTMODE_PWM8_gc );		// 8bit PWM mode
	
	//! Enable the waveform output signal
	SET_BIT( ctrlb_tmp, TCB_CCMPEN_bp );
	
	//! Initial output level is high
	//SET_BIT( ctrlb_tmp, TCB_CCMPINIT_bp );
	
	//! false=signal is updated at timer start in single shot | true=signal is updated as event arrives in single shot
	//SET_BIT( ctrlb_tmp, TCB_ASYNC_bp );
	
	//! enable input capture
	//SET_BIT( evctrl_tmp, TCB_CAPTEI_bp );
	
	//! event capture edge sensitivity. Dependent on mode of operation. Look datasheet for details
	//SET_BIT( evctrl_tmp, TCB_EDGE_bp );
	
	//! enable input noise canceler
	//SET_BIT( evctrl_tmp, TCB_FILTER_bp );
	
	//! enable interrupt for capture event
	//SET_BIT( evctrl_tmp, TCB_CAPT_bp );
	
	//! this timer will run when UPDI is in debug mode
	//SET_BIT( dbgctrl_tmp, TCB_DBGRUN_bp );
	
	
	//----------------------------------------------------------------
	//	WRITE BACK
	//----------------------------------------------------------------
	
	//TOP in PWM 8-bit mode
	timer.CCMPL = 255;
	//PWM in PWM 8bit mode
	timer.CCMPH = 127;
	
	timer.CTRLB = ctrlb_tmp;
	timer.EVCTRL = evctrl_tmp;
	timer.DBGCTRL = dbgctrl_tmp;
	//Writing back this register will start the timer
	timer.CTRLA = ctrla_tmp;
	//Writing back this register will enable interrupts
	timer.INTCTRL = intctrl_tmp;
	
	return;
}

/****************************************************************************
**  Function
**  init_uart |
****************************************************************************/
//! @brief Initialize one of four USART transceivers
//! @details
//!		Interrupt Vectors
//!	USART0_RXC_vect
//!	USART0_DRE_vect
//!	USART0_TXC_vect
//!	USART1_RXC_vect
//!	USART1_DRE_vect
//!	USART1_TXC_vect
//!	USART2_RXC_vect
//!	USART2_DRE_vect
//!	USART2_TXC_vect
//!	USART3_RXC_vect 
//!	USART3_DRE_vect
//!	USART3_TXC_vect
//!	
//!		| Normal Mode	| Fast Mode	| Sync Mode
//! ---------------------------------------------
//!	S	| 16			| 8			| 2
//!	---------------------------------------------
//!		Communication speed of the UART
//!	Speed [Hz] = 64 *clk [Hz] / S / BAUD
//!		Computation of the baud rate register
//! BAUD = 64 *clk [Hz] / S / Speed [Hz]
//!
//! Baud Rate table. CLK_PER is set from the main prescaler which minimum value is 2
//!	Speed [Hz]	||	BAUD	| CLK_PER [Hz]	| Mode		| Actual Speed [Hz]
//!	------------------------------------------------------------------------
//!	250.0K		||	320		| 20MHz			| Normal	| 250.0KHz
//!	250.0K		||	640		| 20MHz			| Fast		| 250.0KHz
//!	256.0K		||	313		| 20MHz			| Normal	| 255.6KHz <<<
//!	256.0K		||	625		| 20MHz			| Fast		| 265.0KHz
//!	------------------------------------------------------------------------
/***************************************************************************/

void init_uart( USART_t &usart )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//! Fetch registers
	uint8_t ctrl_a		= usart.CTRLA;
	uint8_t ctrl_b		= usart.CTRLB;
	uint8_t ctrl_c		= usart.CTRLC;
	uint8_t dbgctrl_tmp	= usart.DBGCTRL;
	uint8_t ctrl_ev		= usart.EVCTRL;
	
	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

		
		//!Generic configuration bits
	//Enable Loop back mode (internally connect RXI with TXO
	//SET_BIT( ctrl_a, USART_LBME_bp );
	//Multi processor mode: The receiver will wait for a frame containing an address
	//SET_BIT( ctrl_b, USART_MPCM_bp );
	//TXO will work in open drain mode, requiring a pull up resistor but allowing multiple transmitters on the TXO line
	//SET_BIT( ctrl_b, USART_ODME_bp );
	//Start mode enable: A UART start bit will wake the device up
	//SET_BIT( ctrl_b, USART_SFDEN_bp );
	//Enable the transmitter
	SET_BIT( ctrl_b, USART_TXEN_bp );
	//Enable the receiver
	SET_BIT( ctrl_b, USART_RXEN_bp );
	//When enabled, the transmitter will send the configured number of stop bits. Otherwise the transmitter send a single stop bit.
	SET_BIT( ctrl_c, USART_SBMODE_bp );
	//UART will keep running when in debug
	//SET_BIT( dbgctrl_tmp, USART_DBGRUN_bp);
	
		//! UART operation mode
	//Asynchronous UART Mode
	SET_MASKED_BIT( ctrl_c, USART_CMODE_gm, USART_CMODE_ASYNCHRONOUS_gc );
	//Synchronous UART Mode
	//SET_MASKED_BIT( ctrl_c, USART_CMODE_gm, USART_CMODE_SYNCHRONOUS_gc );
	//Infrared Communication UART Mode
	//SET_MASKED_BIT( ctrl_c, USART_CMODE_gm, USART_CMODE_IRCOM_gc );
	//Master SPI Mode
	//SET_MASKED_BIT( ctrl_c, USART_CMODE_gm, USART_CMODE_MSPI_gc );
	
		//! 485 Mode
	//Disable 485 Mode
	SET_MASKED_BIT( ctrl_a, USART_RS485_gm, USART_RS485_OFF_gc );
	//External 485 mode: Transmit Enable pin will activate external driver upon TX
	//SET_MASKED_BIT( ctrl_a, USART_RS485_gm, USART_RS485_EXT_gc );
	//Internal 485 mode
	//SET_MASKED_BIT( ctrl_a, USART_RS485_gm, USART_RS485_INT_gc );
	
		//!	RX Mode
	//Normal mode
	SET_MASKED_BIT( ctrl_b, USART_RXMODE_gm, USART_RXMODE_NORMAL_gc );
	//Double Speed mode
	//SET_MASKED_BIT( ctrl_b, USART_RXMODE_gm, USART_RXMODE_CLK2X_gc );
	//Asynchronous Slave mode: A sync character will be used to automatically set baud rate
	//SET_MASKED_BIT( ctrl_b, USART_RXMODE_gm, USART_RXMODE_GENAUTO_gc );
	//Asynchronous Slave mode: A sync character will be used to automatically set baud rate. Special rules allow validation of sync character
	//SET_MASKED_BIT( ctrl_b, USART_RXMODE_gm, USART_RXMODE_LINAUTO_gc );
	
		//! Word Size
	//5 bit
	//SET_MASKED_BIT( ctrl_c, USART_CHSIZE_gm, USART_CHSIZE_5BIT_gc );
	//6 bit
	//SET_MASKED_BIT( ctrl_c, USART_CHSIZE_gm, USART_CHSIZE_6BIT_gc );
	//7 bit
	//SET_MASKED_BIT( ctrl_c, USART_CHSIZE_gm, USART_CHSIZE_7BIT_gc );
	//8 bit
	SET_MASKED_BIT( ctrl_c, USART_CHSIZE_gm, USART_CHSIZE_8BIT_gc );
	//9 bit, low byte first
	//SET_MASKED_BIT( ctrl_c, USART_CHSIZE_gm, USART_CHSIZE_9BITL_gc );
	//9 bit, high byte first
	//SET_MASKED_BIT( ctrl_c, USART_CHSIZE_gm, USART_CHSIZE_9BITH_gc );
	
	
		//! Parity Mode
	//No parity bit
	SET_MASKED_BIT( ctrl_c, USART_PMODE_gm, USART_PMODE_DISABLED_gc );
	//Parity bit is automatically computed and sent in each frame and checked by the receiver. Even parity
	//SET_MASKED_BIT( ctrl_c, USART_PMODE_gm, USART_PMODE_EVEN_gc );
	//Parity bit is automatically computed and sent in each frame and checked by the receiver. Odd parity
	//SET_MASKED_BIT( ctrl_c, USART_PMODE_gm, USART_PMODE_ODD_gc );

		//! Master SPI Mode
	//In master SPI mode set the data order. Enabled=LSB first
	//SET_BIT( ctrl_c, USART_UDORD_bp );
	//Set the phase sensitivity of the clock
	//SET_BIT( ctrl_c, USART_UCPHA_bp );
	
	
	//----------------------------------------------------------------
	//	INITERRUPTS
	//----------------------------------------------------------------

	//RX interrupt will be trigger in case of auto baud error detected through ISFIF flag
	//SET_BIT( ctrl_a, USART_ABEIE_bp );
	//Enable Receiver Start Frame interrupt
	//SET_BIT( ctrl_a, USART_RXSIE_bp );
	//Enable Data register empty interrupt
	//SET_BIT( ctrl_a, USART_DREIE_bp );
	//Enable TX Interrupt
	//SET_BIT( ctrl_a, USART_TXCIE_bp );
	//Enable RX Interrupt
	SET_BIT( ctrl_a, USART_RXCIE_bp );
	//Enable Infrared Interrupt
	//SET_BIT( ctrl_ev, USART_IREI_bp );
	

	//----------------------------------------------------------------
	//	WRITE BACK
	//----------------------------------------------------------------
	
		//! Baud rate register
	//Set the baud rate of the peripheral
	usart.BAUD = 313;
	
		//! Infrared mode
	//Infrared transmitter pulse length
	usart.TXPLCTRL = 0;
	//Infrared receiver pulse length
	usart.RXPLCTRL = 0;
	
	//Write back configuration registers
	usart.DBGCTRL = dbgctrl_tmp;
	usart.CTRLC = ctrl_c;
	//Writing back this register enables interrupts
	usart.EVCTRL = ctrl_ev;
	usart.CTRLA = ctrl_a;
	//Writing back his register will enables the UART
	usart.CTRLB = ctrl_b;

	return;
}	//End: init_uart
