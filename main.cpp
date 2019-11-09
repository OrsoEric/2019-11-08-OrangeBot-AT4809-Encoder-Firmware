/****************************************************************
**	OrangeBot Project
*****************************************************************
**        /
**       /
**      /
** ______ \
**         \
**          \
*****************************************************************
**	OrangeBot AT4809 Firmware
*****************************************************************
**  This firmware is meant to run on the OrangeBot robot due to
**	participate in the PiWars 2020
**
**	Compiler flags:
**		-Os							|
**		-std=c++11					|
**		-fno-threadsafe-statics		| disable mutex around static local variables
****************************************************************/

/****************************************************************
**	DESCRIPTION
****************************************************************
**	OrangeBot MVP
**	UART interface with RPI
**	Uniparser V4
**	Set PWM from serial interface
****************************************************************/

/****************************************************************
**	HISTORY VERSION
****************************************************************
**		2019-11-01
**	Add UART3 uc <--> RPI
**	Add Uniparser V4
**	Add platform messages
**	Test OrangeBot Remote Control MVP
**		2019-11-05
**	Clone 2019-10-02 OrangeBot MVP Remote control
**	Add Encoder ISR
**		2019-11-06
**	
****************************************************************/

/****************************************************************
**	USED PINS
****************************************************************
**	VNH7040
**				|	DRV0	|	DRV1	|	DRV2	|	DRV3	|	VNH7040
**	-------------------------------------------------------------------------
**	uC_SEN		|	PF0		|	PF0		|	PF0		|	PF0		|	SENSE ENABLE
**	uC_DIAG		|	PF1		|	PF1		|	PF1		|	PF1		|	SEL1
**	uC_PWM		|	PA2,B20	|	PA3,B21	|	PB4,B22	|	PB5,B23	|	PWM
**	uC_CTRLA	|	PA4		|	PA6		|	PB2		|	PD6		|	INA, SEL0
**	uC_CTRLB	|	PA5		|	PA7		|	PB3		|	PD7		|	INB
****************************************************************/

/****************************************************************
**	KNOWN BUGS
****************************************************************
**
****************************************************************/

/****************************************************************
**	DEFINES
****************************************************************/

#define EVER (;;)

/****************************************************************
**	INCLUDES
****************************************************************/


#include "global.h"
//Universal Parser V4
#include "uniparser.h"
//from number to string
#include "at_string.h"

/****************************************************************
** FUNCTION PROTOTYPES
****************************************************************/
	
	///----------------------------------------------------------------------
	///	PERIPHERALS
	///----------------------------------------------------------------------

//Initialize motors
extern void init_motors( void );
//Set PWM of all motor channels applying slew rate limiting
extern void update_pwm( void );

/****************************************************************
** GLOBAL VARIABLES
****************************************************************/

volatile Isr_flags g_isr_flags;

	///----------------------------------------------------------------------
	///	BUFFERS
	///----------------------------------------------------------------------
	//	Buffers structure and data vectors

//Safe circular buffer for UART input data
volatile At_buf8_safe rpi_rx_buf;
//Safe circular buffer for uart tx data
At_buf8 rpi_tx_buf;
//allocate the working vector for the buffer
uint8_t v0[ RPI_RX_BUF_SIZE ];
//allocate the working vector for the buffer
uint8_t v1[ RPI_TX_BUF_SIZE ];

	///--------------------------------------------------------------------------
	///	MOTORS
	///--------------------------------------------------------------------------

//Desired setting for the DC motor channels
Dc_motor_pwm dc_motor[DC_MOTOR_NUM];
//Two DC Motor channels
Dc_motor_pwm dc_motor_target[DC_MOTOR_NUM];

	///--------------------------------------------------------------------------
	///	ENCODERS
	///--------------------------------------------------------------------------

//Global 32b encoder counters
volatile int32_t g_enc_cnt[ENC_NUM];

/****************************************************************************
**  Function
**  main |
****************************************************************************/
//! @return bool |
//! @brief dummy method to copy the code
//! @details test the declaration of a lambda method
/***************************************************************************/

int main(void)
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//system tick prescaler
	uint8_t pre = 0;
	//activity LED prescaler
	uint8_t pre_led = 0;
	
	//Blink speed of the LED. Start slow
	uint8_t blink_speed = 99;
	//Raspberry PI UART RX Parser
	Orangebot::Uniparser rpi_rx_parser = Orangebot::Uniparser();
	
	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

		///UART RX BUFFER INIT
	//I init the rx and tx buffers
	//attach vector to buffer
	AT_BUF_ATTACH( rpi_rx_buf, v0, RPI_RX_BUF_SIZE);
	//attach vector to buffer
	AT_BUF_ATTACH( rpi_tx_buf, v1, RPI_TX_BUF_SIZE);

	//! Initialize AT4809 internal peripherals
	init();
	//! Initialize external peripherals
	init_motors();
	
	//! Initialize the static vars of the encoder decoding ISR
	quad_encoder_decoder( PORTC.IN );

		//!	Initialize VNH7040
	//Enable sense output
	SET_BIT_VALUE( PORTF.OUT, 0, true );
	//Diagnostic mode OFF
	SET_BIT_VALUE( PORTF.OUT, 1, false );
	
		///----------------------------------------------------------------------
		///	REGISTER PARSER COMMANDS
		///----------------------------------------------------------------------

	//! Register commands and handler for the universal parser class. A masterpiece :')
	//Register ping command. It's used to reset the communication timeout
	rpi_rx_parser.add_cmd( "P", (void *)&ping_handler );
	//Register the Find command. Board answers with board signature
	rpi_rx_parser.add_cmd( "F", (void *)&signature_handler );
	//Set individual motor speed command. mm/s
	rpi_rx_parser.add_cmd( "M%SPWM%S", (void *)&set_speed_handler );
	//Set platform speed handler to be retro compatible with SoW-B
	rpi_rx_parser.add_cmd( "PWMR%SL%S", (void *)&set_platform_speed_handler );
	//Send encoder reading through UART
	rpi_rx_parser.add_cmd( "ENC", (void *)&get_encoder_cnt_handler );
	
	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Main loop
	for EVER
	{
		//If: System Tick 500Hz
		if (g_isr_flags.system_tick == 1)
		{
			//Clear system tick
			g_isr_flags.system_tick = 0;
			
			//----------------------------------------------------------------
			//	FULL SPEED CODE
			//----------------------------------------------------------------
			
			//Update PWM of the motors while applying the slew rate limiter
			update_pwm();
			
			//----------------------------------------------------------------
			//	SYSTEM PRESCALER SPEED CODE
			//----------------------------------------------------------------
				
			//If prescaler has reset
			if (pre == 0)
			{
				//----------------------------------------------------------------
				//	LED BLINK
				//----------------------------------------------------------------
				//		Two speeds
				//	slow: not in timeout and commands can be executed
				//	fast: in timeout, motor stopped
				
				if (pre_led == 0)
				{
					//Toggle PF5.
					SET_BIT( PORTF.OUTTGL, 5 );	
				}
				//Increment with top
				pre_led = AT_TOP_INC( pre_led, blink_speed );
				
				//----------------------------------------------------------------
				//	PARSER TIMEOUT
				//----------------------------------------------------------------
				
				//Update communication timeout counter
				uart_timeout_cnt++;
				//
				if (uart_timeout_cnt >= RPI_COM_TIMEOUT)
				{
					//Clip timeout counter
					uart_timeout_cnt = RPI_COM_TIMEOUT;
					//If: it's the first time the communication timeout is detected
					if (f_timeout_detected == false)
					{						
						//LED is blinking faster
						blink_speed = 9;
					}
					//raise the timeout flag
					f_timeout_detected = true;
				}
				else
				{
					//This is the only code allowed to reset the timeout flag
					f_timeout_detected = false;
					//LED is blinking slower
					blink_speed = 99;
				}		
				
			}
			//Increment prescaler and reset if it exceeds the TOP.
			pre = AT_TOP_INC( pre, 4);
			
		}	//End If: System Tick
		
		//----------------------------------------------------------------
		//	AT4809 --> RPI USART TX
		//----------------------------------------------------------------
		
		//if: RPI TX buffer is not empty and the RPI TX HW buffer is ready to transmit
		if ( (AT_BUF_NUMELEM( rpi_tx_buf ) > 0) && (IS_BIT_ONE(USART3.STATUS, USART_DREIF_bp)))
		{
			//temp var
			uint8_t tx_tmp;
			//Get the byte to be filtered out
			tx_tmp = AT_BUF_PEEK( rpi_tx_buf );
			AT_BUF_KICK( rpi_tx_buf );
			//Send data through the UART3
			USART3.TXDATAL = tx_tmp;
		}	//End If: RPI TX
		
		//----------------------------------------------------------------
		//	RPI --> AT4809 USART RX
		//----------------------------------------------------------------
		
		//if: RX buffer is not empty	
		if (AT_BUF_NUMELEM( rpi_rx_buf ) > 0)
		{
			//temp var
			uint8_t rx_tmp;
				
				///Get data
			//Get the byte from the RX buffer (ISR put it there)
			rx_tmp = AT_BUF_PEEK( rpi_rx_buf );
			AT_BUF_KICK_SAFER( rpi_rx_buf );

				///Loop back
			//Push into tx buffer
			//AT_BUF_PUSH( rpi_tx_buf, rx_tmp );

				///Command parser
			//feed the input RX byte to the parser
			rpi_rx_parser.exe( rx_tmp );
			
		} //endif: RPI RX buffer is not empty

	}	//End: Main loop

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return 0;
}	//end: main

/****************************************************************************
**  Function
**  init_motors
****************************************************************************/
//! @return void |
//! @brief Initialize motors
//! @details Initialize motors
//!
/***************************************************************************/

void init_motors( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//counter
	uint8_t t;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------
	
	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//For: scan motors
	for (t = 0;t < DC_MOTOR_NUM;t++)
	{
		//Initialize motor
		dc_motor[t].f_dir = false;
		dc_motor[t].pwm = (uint8_t)0x00;
	}	//End For: scan motors
	
	//For: scan motors
	for (t = 0;t < DC_MOTOR_NUM;t++)
	{
		//Initialize motor
		dc_motor_target[t].f_dir = false;
		dc_motor_target[t].pwm = (uint8_t)0x00;
	}
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End: init_motors

/****************************************************************************
**  Function
**  set_vnh7040_speed
****************************************************************************/
//! @return void |
//! @param index	| index of the motor to be controlled. 0 to 3
//! @param f_dir	| direction of rotation of the motor
//! @param speed	| Speed of the motor
//! @brief Set direction and speed setting of the VNH7040 controlled motor
//! @details 
//!
/***************************************************************************/

void set_vnh7040_speed( uint8_t index, bool f_dir, uint8_t speed )
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

	//Driver 0
	if (index == 0)
	{
		//Set INA, INB and SEL0 to select the direction of rotation of the motor
		if (f_dir == true)
		{
			SET_MASKED_BIT( PORTA.OUT, 0x30, 0x10);
		}
		else
		{
			SET_MASKED_BIT( PORTA.OUT, 0x30, 0x20);
		}
		//Set the PWM value of the right PWM channel of TCA0
		TCB0.CCMPH = speed;	
	}
	//Driver 1
	else if (index == 1)
	{
		//Set INA, INB and SEL0 to select the direction of rotation of the motor
		if (f_dir == true)
		{
			SET_MASKED_BIT( PORTA.OUT, 0xc0, 0x40);
		}
		else
		{
			SET_MASKED_BIT( PORTA.OUT, 0xc0, 0x80);
		}
		//Set the PWM value of the right PWM channel of TCA0
		TCB1.CCMPH = speed;
	}
	//Driver 2
	else if (index == 2)
	{
		//Set INA, INB and SEL0 to select the direction of rotation of the motor
		if (f_dir == true)
		{
			SET_MASKED_BIT( PORTB.OUT, 0x0c, 0x04);
		}
		else
		{
			SET_MASKED_BIT( PORTB.OUT, 0x0c, 0x08);
		}
		//Set the PWM value of the right PWM channel of TCA0
		TCB2.CCMPH = speed;
	}
	//Driver 3
	else if (index == 3)
	{
		//Set INA, INB and SEL0 to select the direction of rotation of the motor
		if (f_dir == true)
		{
			SET_MASKED_BIT( PORTD.OUT, 0xc0, 0x40);
		}
		else
		{
			SET_MASKED_BIT( PORTD.OUT, 0xc0, 0x80);
		}
		//Set the PWM value of the right PWM channel of TCA0
		TCB3.CCMPH = speed;
	}
	//Default case
	else
	{
		//Driver index not installed.
		//Do nothing
	}

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End: 

/***************************************************************************/
//!	@brief convert from a DC motor PWM structure to a speed number
//!	convert_pwm_to_s16 | Dc_motor_pwm
/***************************************************************************/
//! @param input | Dc_motor_pwm	input PWM structure
//! @param f_dir | bool			conversion between clockwise/counterclockwise to forward/backward
//! @return int16_t |			speed number
/***************************************************************************/

inline int16_t convert_pwm_to_s16( Dc_motor_pwm input, bool f_dir )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//temp return
	int16_t ret;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Extract PWM setting
	ret = input.pwm;
	//if: direction is backward
	if (input.f_dir != f_dir)
	{
		//Correct sign
		ret = -ret;
	}

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return ret;
}

/***************************************************************************/
//!	@brief convert from a speed number to a dc motor PWM structure
//!	convert_s16_to_pwm | int16_t, bool
/***************************************************************************/
//! @param input | int16_t 		Input speed number
//! @param f_dir | bool			conversion between clockwise/counterclockwise to forward/backward
//! @return Dc_motor_pwm		PWM structure
/***************************************************************************/

inline Dc_motor_pwm convert_s16_to_pwm( int16_t input, bool f_dir )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//temp return
	Dc_motor_pwm ret;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//For each DC motor channel
	//If: right direction is backward
	if (input < 0)
	{
		//Assign the pwm to the right temp channel
		ret.pwm = -input;
		ret.f_dir = !f_dir;
	}
	//If: right direction is forward
	else
	{
		//Assign the pwm to the right temp channel
		ret.pwm = input;
		ret.f_dir = f_dir;
	}

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return ret;
}

/***************************************************************************/
//!	@brief update DC motor PWM
//!	update_pwm | void
/***************************************************************************/
//! @return void
//!	@details
//! Move PWM toward target PWM
//! Apply slew rate limiter
/***************************************************************************/

void update_pwm( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//temp counter
	uint8_t t;
	//true if speed has changed
	//bool f_change[DC_MOTOR_NUM];

	Dc_motor_pwm target_speed, actual_speed;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//If the communication failed
	if (f_timeout_detected == true)
	{
		//Counter
		uint8_t t;
		//For: scan motors
		for (t = 0;t < DC_MOTOR_NUM;t++)
		{
			//Stop the motors
			set_vnh7040_speed( (uint8_t)t, (uint8_t)false, (uint8_t)0x00 );
			//Update actual PWM so tat slew rate limiter will do sensible things when restarting
			dc_motor[t].pwm = (uint8_t)0x00;
		}
		//
		return;
	}

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//For: each DC motor channel
	for (t = 0;t < DC_MOTOR_NUM;t++)
	{
		//Fetch current settings
		target_speed = dc_motor_target[t];
		actual_speed = dc_motor[t];
		
		//If directions are different
		if (target_speed.f_dir != actual_speed.f_dir)
		{
			//if pwm is above slew rate
			if (actual_speed.pwm > DC_MOTOR_SLEW_RATE)
			{
				//Slow down by the slew rate
				actual_speed.pwm -= DC_MOTOR_SLEW_RATE;
			}
			//if pwm is below or at slew rate
			else
			{
				//Set neutral speed
				actual_speed.pwm = 0;
			}
			//if pwm is zero
			if (actual_speed.pwm == 0)
			{
				//I'm authorized to change direction
				actual_speed.f_dir = target_speed.f_dir;
			}
		}	//End If directions are different
		//if direction is the same
		else
		{
			//if pwm is above target
			if (actual_speed.pwm > target_speed.pwm)
			{
				//Decrease speed by PWM
				actual_speed.pwm = AT_SAT_SUM( actual_speed.pwm, -DC_MOTOR_SLEW_RATE, DC_MOTOR_MAX_PWM, 0 );
				//if: overshoot
				if (actual_speed.pwm < target_speed.pwm)
				{
					//I reached the target
					actual_speed.pwm = target_speed.pwm;
				}
			}
			//if pwm is below target
			else if (actual_speed.pwm < target_speed.pwm)
			{
				//Decrease speed by PWM
				actual_speed.pwm = AT_SAT_SUM( actual_speed.pwm, +DC_MOTOR_SLEW_RATE, DC_MOTOR_MAX_PWM, 0 );
				//if: overshoot
				if (actual_speed.pwm > target_speed.pwm)
				{
					//I reached the target
					actual_speed.pwm = target_speed.pwm;
				}
			}
			//if: I'm already at the right speed
			else
			{
				//do nothing
			}
		}
		
		//Apply setting
		set_vnh7040_speed( t, actual_speed.f_dir, actual_speed.pwm );
		
		//Write back setting
		dc_motor[t] = actual_speed;
	}	//End For: each DC motor channel

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return; //OK
}	//end handler: update_pwm | void

/***************************************************************************/
//!	function
//!	get_enc_cnt
/***************************************************************************/
//! @param enc_cnt | int32_t vector. Function returns in this vector the value of the global encoder counters
//! @return bool | false=OK | true=failed to update global counters
//! @brief Force an update and save the 32b encoder counters in an input vector
//! @details
//!		Algorithm:
//!	>disable interrupt
//!	>raise sync flag
//!	>manually execute encoder decoding routine
//! >Transfer register value to local vars
/***************************************************************************/

bool get_enc_cnt( int32_t *enc_cnt )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//Counter
	uint8_t t;
	//return flag
	bool f_ret;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Disable interrupts
	cli();
	//Force update of 32b global counters
	g_isr_flags.enc_updt = true;
	//reserve global encoder counters
	g_isr_flags.enc_sem = true;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Decode encoders and update 32b counters 
	quad_encoder_decoder( PORTC.IN );
	//Enable interrupts
	sei();
	//If encoder routine failed to update global counters
	f_ret = g_isr_flags.enc_updt;
	//For: all encoder channels
	for (t = 0;t < ENC_NUM;t++)
	{
		//Copy
		enc_cnt[t] = g_enc_cnt[t];
	}
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	//Unreserve global counters
	g_isr_flags.enc_sem = false;
	//false = OK | true = fail (global counters failed to update)
	return f_ret;
}

/***************************************************************************/
//!	function
//!	function_template
/***************************************************************************/
//! @param x |
//! @return void |
//! @brief
//! @details
/***************************************************************************/

void function_template( void )
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

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}
