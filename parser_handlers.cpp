
/****************************************************************
**	INCLUDES
****************************************************************/

#include "global.h"
//Universal Parser V4
//#include "uniparser.h"

#include "at_string.h"

/****************************************************************
** GLOBAL VARIABLES
****************************************************************/

	///--------------------------------------------------------------------------
	///	PARSER
	///--------------------------------------------------------------------------

//Board Signature
U8 *board_sign = (U8 *)"Seeker-Of-Ways-B-00002";
//communication timeout counter
U8 uart_timeout_cnt = 0;
//Communication timeout has been detected
bool f_timeout_detected = false;

/***************************************************************************/
//!	@brief ping command handler
//!	ping_handler | void
/***************************************************************************/
//! @return void
//!	@details
//! Handler for the ping command. Keep alive connection
/***************************************************************************/

void ping_handler( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------
	
	//Reset communication timeout handler
	uart_timeout_cnt = 0;
	
	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return; //OK
}	//end handler: ping_handler | void

/***************************************************************************/
//!	@brief board signature handler
//!	signature_handler | void
/***************************************************************************/
//! @return void
//!	@details
//! Handler for the get board signature command. Send board signature via UART
/***************************************************************************/

void signature_handler( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	uint8_t t;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Reset communication timeout handler
	uart_timeout_cnt = 0;
	
	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Init while
	t = 0;
	//while: no termination and tx buffer width is not exceeded
	while ((t < RPI_TX_BUF_SIZE) && (board_sign[t]!= '\0'))
	{
		//Send the next signature byte
		AT_BUF_PUSH(rpi_tx_buf, board_sign[t]);
	}

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return; //OK
}	//end handler: signature_handler | void

/***************************************************************************/
//!	@brief set the target speed of the DC motors
//!	set_speed_handler | int16_t, int16_t
/***************************************************************************/
//! @param motor_index | index of the motor being controlled
//! @param pwm | new PWM setting of the motor
//! @return false: OK | true: fail
//!	@details
//! Handler for the motor speed set command. It's going to be called automatically when command is received
/***************************************************************************/

void set_speed_handler( int16_t motor_index, int16_t pwm )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	bool f_dir;
	uint8_t tcb_pwm;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Reset communication timeout handler
	uart_timeout_cnt = 0;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	if (pwm < 0)
	{
		f_dir = true;
		pwm = -pwm;
	}
	else
	{
		f_dir = false;
	}
	
	tcb_pwm = pwm;

	//Set direction and speed setting of the VNH7040 controlled motor
	set_vnh7040_speed( motor_index, f_dir, tcb_pwm );
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return; //OK
}	//end handler: set_speed_handler | void

/***************************************************************************/
//!	@brief set the target speed of the DC motors
//!	set_platform_speed_handler | int16_t, int16_t
/***************************************************************************/
//! @param pwm_r | pwm for right side of platform
//! @param pwm_l | pwm for left side of platform
//! @return false: OK | true: fail
//!	@details
//! Handler for the platform speed. Firmware handles logical configuration of the motors. TODO: evolve to forward and turn
//! Numeration is handled like an IC with dot on the back, viewing from the top
//! Direction is corrected so that plus is forward
//!			Left	Right
//!	Front	2		1
//! Rear	3		0
/***************************************************************************/

void set_platform_speed_handler(int16_t right, int16_t left )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	uint8_t f_dir_r, f_dir_l;
	uint8_t tcb_pwm_l, tcb_pwm_r;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Reset communication timeout handler
	uart_timeout_cnt = 0;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	MOTORS LAYOUT CORRECTIONS
	//----------------------------------------------------------------

	if (right < 0)
	{
		f_dir_r = false;
		tcb_pwm_r = -right;
	}
	else
	{
		f_dir_r = true;
		tcb_pwm_r = right;
	}
	
	if (left < 0)
	{
		f_dir_l = true;
		tcb_pwm_l = -left;
	}
	else
	{
		f_dir_l = false;
		tcb_pwm_l = left;
	}
	
	//----------------------------------------------------------------
	//	UPDATE TARGET PWM AND DIRECTION
	//----------------------------------------------------------------
	//	upgrade_pwm will take care of trying to reach the desired setting

	dc_motor_target[0].f_dir = 	f_dir_r;
	dc_motor_target[0].pwm = 	tcb_pwm_r;
	dc_motor_target[1].f_dir = 	f_dir_r;
	dc_motor_target[1].pwm = 	tcb_pwm_r;
	dc_motor_target[2].f_dir = 	f_dir_l;
	dc_motor_target[2].pwm = 	tcb_pwm_l;
	dc_motor_target[3].f_dir = 	f_dir_l;
	dc_motor_target[3].pwm = 	tcb_pwm_l;
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return; //OK
}	//end handler: set_platform_speed_handler | void

/***************************************************************************/
//!	function
//!	get_encoder_cnt_handler
/***************************************************************************/
//! @param x |
//! @return void |
//! @brief Send all 32b encoder counters
//! @details
/***************************************************************************/

void get_encoder_cnt_handler( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//counters
	uint8_t t, ti;
	//temp encoder counters
	int32_t enc_cnt[ENC_NUM];
	//Temp message
	uint8_t msg[15];
	//temp return
	uint8_t ret;
	
	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Reset communication timeout handler
	uart_timeout_cnt = 0;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Safely get updated encoder counts from global counters
	get_enc_cnt( enc_cnt );
	
	//Preamble
	AT_BUF_PUSH( rpi_tx_buf, 'E' );
	AT_BUF_PUSH( rpi_tx_buf, 'N' );
	AT_BUF_PUSH( rpi_tx_buf, 'C' );
	
	//Scan each encoder channel
	for (t = 0;t < ENC_NUM;t++)	
	{
		//Encoder channel identifier
		AT_BUF_PUSH( rpi_tx_buf, 'E' );
		AT_BUF_PUSH( rpi_tx_buf, '0'+t );
		AT_BUF_PUSH( rpi_tx_buf, 'N' );
		//Decode S32 into a string
		ret = s32_to_str( enc_cnt[t], msg );
		//Scan each byte inside the string
		for (ti = 0;ti < ret;ti++)
		{
			//Send number
			AT_BUF_PUSH( rpi_tx_buf, msg[ti] );
		}
	}
	//Termination
	AT_BUF_PUSH( rpi_tx_buf, '\0' );

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}

/***************************************************************************/
//!	function
//!	send_encoder_cnt
/***************************************************************************/
//! @param x |
//! @return void |
//! @brief Send all 32b encoder counters
//! @details
/***************************************************************************/

void send_encoder_cnt( int32_t *enc_cnt )
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
} //End function: send_encoder_cnt
