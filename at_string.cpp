/****************************************************************************
**	ROBOTICA@POLITO PROJECT
*****************************************************************************
**	AT_STRING
*****************************************************************************
**	Author: 			Orso Eric
**	Creation Date:
**	Last Edit Date:
**	Revision:			2019-11-07
**	Version:			2.0
****************************************************************************/

/****************************************************************************
**	HYSTORY VERSION
*****************************************************************************
**		R1 V0.1ALPHA
**	>Tested the u8_to_str for all values
**	>Tested the s8_to_str, it's basicaly a wrapper of u8_to_str that handles the signs
**	>tester u16_to_str for all values (I used sprintf and compared output strings)
**	>tester s16_to_str for all values (I used sprintf and compared output strings)
**		2019-11-07
**  >Adding 32bit support
**	>Refactor code to doxygen compatible
**	>Remove dependency on custom types
****************************************************************************/

/****************************************************************************
**	INCLUDE
****************************************************************************/

#include "stdint.h"
#include "at_string.h"

/****************************************************************************
** FUNCTIONS
****************************************************************************/

/***************************************************************************/
//!	function
//!	u8_to_str
/***************************************************************************/
//! @param num		| number to be converted
//! @param str		| return string provied by the caller
//! @return uint8_t	| number of digits written in the string
//! @brief convert an unsigned 8b into a string
//! @details
//! Constants
//!	max uint8_t			: 256
//! max uint8_t base	: 100
//! max uint8_t digits	: 3
/***************************************************************************/

uint8_t u8_to_str( uint8_t num, uint8_t *str )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//decimal base
	uint8_t base[] =
	{
		100,
		10,
		1
	};
	//temp var
	uint8_t t, u8t;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//index to the string
	uint8_t index = 0;
	//flag used to blank non meaningful zeros
	bool flag = true;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//For all bases
	for (t = 0;t < MAX_DIGIT8; t++)
	{
		//If the base is bigger or equal than the number (division is meaningful)
		if (base[t] <= num)
		{
			//Divide number by base, get the digit
			u8t = num/base[t];
			//Write the digit
			str[ index ] = '0' +u8t;
			//Update the number
			num = num - base[t] * u8t;
			//I have found a meaningful digit
			flag = false;
			//Jump to the next digit
			index++;
		}
		//If: The base is smaller then the number, and I have yet to find a non zero digit, and I'm not to the last digit
		else if ( (flag == true) && (t != (MAX_DIGIT8 -1)) )
		{
			//do nothing
		}
		//If: I have a meaningful zero
		else
		{
			//It's a zero
			str[ index ] = '0';
			//Jump to the next digit
			index++;
		}
	}	//End for: all bases

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	//Append the terminator
	str[ index ] = '\0';

	return index;
}   //End function: u8_to_str

/***************************************************************************/
//!	function
//!	s8_to_str
/***************************************************************************/
//! @param num		| number to be converted
//! @param str		| return string provied by the caller
//! @return uint8_t	| number of digits written in the string
//! @brief Convert an int16_t to string
//! @details
//!	This function is a thin wrapper of u16_to_str that apply sign correction and detection
/***************************************************************************/

uint8_t s8_to_str( int8_t num, uint8_t *str )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//number of character written
	uint8_t ret;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//If: negative
	if (num < 0)
	{
		//Write sign '-'
		str[ 0 ] = '-';
		//Correct sign
		num = -num;

	}
	//If: zero or positive
	else
	{
		//Write sign '+'
		str[ 0 ] = '+';
	}
	//launch the u8_to_str to the corrected num, but feed the vector shifted by 1 to make room for the sign. save the return value
	ret = u8_to_str( num, &str[1] );

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return (ret +1);
} //End function: s8_to_str

/***************************************************************************/
//!	function
//!	u16_to_str
/***************************************************************************/
//! @param num		| number to be converted
//! @param str		| return string provied by the caller
//! @return uint8_t	| number of digits written in the string
//! @brief convert an unsigned 16b into a string
//! @details
//! Constants
//!	max uint16_t		: 65536
//! max uint16_t base	: 10000
//! max uint16_t digits	: 5
/***************************************************************************/

uint8_t u16_to_str( uint16_t num, uint8_t *str )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//decimal base
	uint16_t base[] =
	{
		10000,
		1000,
		100,
		10,
		1
	};
	//temp var
	uint8_t t, u8t;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//index to the string
	uint8_t index = 0;
	//flag used to blank non meaningful zeros
	bool flag = true;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//For all bases
	for (t = 0;t < MAX_DIGIT16; t++)
	{
		//If the base is bigger or equal than the number (division is meaningful)
		if (base[t] <= num)
		{
			//Divide number by base, get the digit
			u8t = num/base[t];
			//Write the digit
			str[ index ] = '0' +u8t;
			//Update the number
			num = num - base[t] * u8t;
			//I have found a meaningful digit
			flag = false;
			//Jump to the next digit
			index++;
		}
		//If: The base is smaller then the number, and I have yet to find a non zero digit, and I'm not to the last digit
		else if ( (flag == false) && (t != (MAX_DIGIT16 -1)) )
		{
			//do nothing
		}
		//If: I have a meaningful zero
		else
		{
			//It's a zero
			str[ index ] = '0';
			//Jump to the next digit
			index++;
		}
	}	//End for: all bases

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	//Append the terminator
	str[ index ] = '\0';

	return index;
}   //End function: u16_to_str

/***************************************************************************/
//!	function
//!	s16_to_str
/***************************************************************************/
//! @param num		| number to be converted
//! @param str		| return string provied by the caller
//! @return uint8_t	| number of digits written in the string
//! @brief Convert an int16_t to string
//! @details
//!	This function is a thin wrapper of u16_to_str that apply sign correction and detection
/***************************************************************************/

uint8_t s16_to_str( int16_t num, uint8_t *str )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//number of character written
	uint8_t ret;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//If: negative
	if (num < 0)
	{
		//Write sign '-'
		str[ 0 ] = '-';
		//Correct sign
		num = -num;

	}
	//If: zero or positive
	else
	{
		//Write sign '+'
		str[ 0 ] = '+';
	}
	//launch the u8_to_str to the corrected num, but feed the vector shifted by 1 to make room for the sign. save the return value
	ret = u16_to_str( num, &str[1] );

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return (ret +1);
} //End function: s16_to_str

/***************************************************************************/
//!	function
//!	u32_to_str
/***************************************************************************/
//! @param num		| number to be converted
//! @param str		| return string provied by the caller
//! @return uint8_t	| number of digits written in the string
//! @brief convert an unsigned 32b into a string
//! @details
//! Constants
//!	max uint32_t		: 4294967295
//! max uint32_t base	: 1000000000
//! max uint32_t digits	: 9
/***************************************************************************/

uint8_t u32_to_str( uint32_t num, uint8_t *str )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//decimal base
	uint32_t base[] =
	{
		1000000000,
		100000000,
		10000000,
		1000000,
		100000,
		10000,
		1000,
		100,
		10,
		1
	};
	//temp var
	uint8_t t, u8t;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//index to the return string
	uint8_t index = 0;
	//flag used to blank non meaningful zeros
	bool flag = true;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//For all bases
	for (t = 0;t < MAX_DIGIT32; t++)
	{
		//If the base is bigger or equal than the number (division is meaningful)
		if (base[t] <= num)
		{
			//Divide number by base, get the digit
			u8t = num/base[t];
			//Write the digit
			str[ index ] = '0' +u8t;
			//Update the number
			num = num - base[t] *u8t;
			//I have found a meaningful digit
			flag = false;
			//Jump to the next digit
			index++;
		}
		//If: The base is smaller then the number, and I have yet to find a non zero digit, and I'm not to the last digit
		else if ( (flag == true) && (t != (MAX_DIGIT32 -1)) )
		{
			//do nothing
		}
		//If: I have a meaningful zero
		else
		{
			//It's a zero
			str[ index ] = '0';
			//Jump to the next digit
			index++;
		}
	}	//End for: all bases
	//Append the terminator
	str[ index ] = '\0';

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	//Return the length of the string
	return index;
}	//End function: u32_to_str

/***************************************************************************/
//!	function
//!	s32_to_str
/***************************************************************************/
//! @param num		| number to be converted
//! @param str		| return string provied by the caller
//! @return uint8_t	| number of digits written in the string
//! @brief Convert an int32_t to string
//! @details
//!	This function is a thin wrapper of u32_to_str that apply sign correction and detection
/***************************************************************************/

uint8_t s32_to_str( int32_t num, uint8_t *str )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//number of character written
	uint8_t ret;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//If: negative
	if (num < 0)
	{
		//Write sign '-'
		str[ 0 ] = '-';
		//Correct sign
		num = -num;

	}
	//If: zero or positive
	else
	{
		//Write sign '+'
		str[ 0 ] = '+';
	}
	//launch the u8_to_str to the corrected num, but feed the vector shifted by 1 to make room for the sign. save the return value
	ret = u32_to_str( num, &str[1] );

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return (ret +1);
} //End function: s32_to_str
