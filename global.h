#ifndef GLOBAL_H
	//header environment variable, is used to detect multiple inclusion
	//of the same header, and can be used in the c file to detect the
	//included library
	#define GLOBAL_H

	/****************************************************************************
	**	ENVROIMENT VARIABILE
	****************************************************************************/

	#define F_CPU 20000000

	/****************************************************************************
	**	GLOBAL INCLUDE
	**	TIPS: you can put here the library common to all source file
	****************************************************************************/

	//type definition using the bit width and signedness
	#include <stdint.h>
	//define the ISR routune, ISR vector, and the sei() cli() function
	#include <avr/interrupt.h>
	//name all the register and bit
	#include <avr/io.h>
	//hard delay
	#include <util/delay.h>
	//General purpose macros
	#include "at_utils.h"
	//AT4809 PORT macros definitions
	#include "at4809_port.h"

	/****************************************************************************
	**	DEFINE
	****************************************************************************/

		///----------------------------------------------------------------------
		///	BUFFERS
		///----------------------------------------------------------------------

	#define RPI_RX_BUF_SIZE		16
	#define RPI_TX_BUF_SIZE		64
	
		///----------------------------------------------------------------------
		///	PARSER
		///----------------------------------------------------------------------
			
	#define RPI_COM_TIMEOUT		50
	
		///----------------------------------------------------------------------
		///	MOTORS
		///----------------------------------------------------------------------
	
	//Number of DC motors mounted on the platform
	#define DC_MOTOR_NUM		4
	//Maximum slew rate. PWM increment per tick
	#define DC_MOTOR_SLEW_RATE	1
	//Maximum PWM setting
	#define DC_MOTOR_MAX_PWM	50
	
		///----------------------------------------------------------------------
		///	ENCODERS
		///----------------------------------------------------------------------
	
	//Number of quadrature encoders
	#define ENC_NUM				4
	//Threshold upon which local counters are synced with global counters
	#define ENC_UPDATE_TH		100

	/****************************************************************************
	**	MACRO
	****************************************************************************/

		///----------------------------------------------------------------------
		///	LEDS
		///----------------------------------------------------------------------

	#define LED0_TOGGLE()	\
		TOGGLE_BIT( PORTB, PB6 )

	/****************************************************************************
	**	TYPEDEF
	****************************************************************************/

	//Global flags raised by ISR functions
	typedef struct _Isr_flags Isr_flags;
	
	//PWM and direction of a DC motor
	typedef struct _Dc_motor_pwm Dc_motor_pwm;

	/****************************************************************************
	**	STRUCTURE
	****************************************************************************/

	//Global flags raised by ISR functions
	struct _Isr_flags
	{
		//First byte
		U8 system_tick		: 1;	//System Tick
		U8 enc_double_event	: 1;	//true = At least one encoder double event detected
		U8 enc_sem			: 1;	//true = Encoder ISR is forbidden to write into the 32b encoder counters
		U8 enc_updt			: 1;	//true = Encoder ISR is forced to update the 32b counters and clear this flag if possible.
		U8 					: 4;	//unused bits
	};

	//PWM and direction of a DC motor
	struct _Dc_motor_pwm
	{
		uint8_t pwm;			//DC Motor PWM setting. 0x00 = stop | 0xff = maximum
		uint8_t f_dir;			//DC Motor direction. false=clockwise | true=counterclockwise
	};

	/****************************************************************************
	**	PROTOTYPE: INITIALISATION
	****************************************************************************/

	//port configuration and call the peripherals initialization
	extern void init( void );

	/****************************************************************************
	**	PROTOTYPE: FUNCTION
	****************************************************************************/
	
		///----------------------------------------------------------------------
		///	PARSER
		///----------------------------------------------------------------------
		//	Handlers are meant to be called automatically when a command is decoded
		//	User should not call them directly

	//Handler for the ping command. Keep alive connection
	extern void ping_handler( void );
	//Handler for the get board signature command. Send board signature via UART
	extern void signature_handler( void );
	//Handler for the motor speed set command
	extern void set_speed_handler( int16_t motor_index, int16_t pwm );
	//Handler for the platform speed. Firmware handles logical configuration of the motors. TODO: evolve to forward and turn
	extern void set_platform_speed_handler(int16_t right, int16_t left );
	//Handler for the get encoder count message
	extern void get_encoder_cnt_handler( void );
	//Send all 32b encoder counters through the UART
	extern void send_encoder_cnt( void );
		
		///----------------------------------------------------------------------
		///	MOTORS
		///----------------------------------------------------------------------
		
	//Set direction and speed setting of the VNH7040 controlled motor
	extern void set_vnh7040_speed( uint8_t index, bool f_dir, uint8_t speed );
	
		///----------------------------------------------------------------------
		///	ENCODERS
		///----------------------------------------------------------------------
	
	//Decode four quadrature encoder channels
	void quad_encoder_decoder( uint8_t enc_in );
	//Force an update and save the 32b encoder counters in an input vector
	bool get_enc_cnt( int32_t *enc_cnt );
	
	/****************************************************************************
	**	PROTOTYPE: GLOBAL VARIABILE
	****************************************************************************/

		///----------------------------------------------------------------------
		///	STATUS FLAGS
		///----------------------------------------------------------------------

	//Volatile flags used by ISRs
	extern volatile	Isr_flags g_isr_flags;
	
		///----------------------------------------------------------------------
		///	BUFFERS
		///----------------------------------------------------------------------
		//	Buffers structure and data vectors

	//Safe circular buffer for UART input data
	extern volatile At_buf8_safe rpi_rx_buf;
	//Safe circular buffer for uart tx data
	extern At_buf8 rpi_tx_buf;
	//allocate the working vector for the buffer
	extern uint8_t v0[ RPI_RX_BUF_SIZE ];
	//allocate the working vector for the buffer
	extern uint8_t v1[ RPI_TX_BUF_SIZE ];
	
		///--------------------------------------------------------------------------
		///	PARSER
		///--------------------------------------------------------------------------

	//Board Signature
	extern U8 *board_sign;
	//communication timeout counter
	extern U8 uart_timeout_cnt;
	//Communication timeout has been detected
	extern bool f_timeout_detected;
	
		///--------------------------------------------------------------------------
		///	MOTORS
		///--------------------------------------------------------------------------

	//Desired setting for the DC motor channels
	extern Dc_motor_pwm dc_motor_target[DC_MOTOR_NUM];
	//Two DC Motor channels current setting
	extern Dc_motor_pwm dc_motor[DC_MOTOR_NUM];
	
		///--------------------------------------------------------------------------
		///	ENCODERS
		///--------------------------------------------------------------------------

	//Global 32b encoder counters
	extern volatile int32_t g_enc_cnt[ENC_NUM];

#else
	#warning "multiple inclusion of the header file global.h"
#endif


