#ifndef AT_STRING_H
	//header envroiment variabile, is used to detect multiple inclusion
	//of the same header, and can be used in the c file to detect the
	//included library
	#define AT_STRING_H

	/****************************************************************************
	**	ENVROIMENT VARIABILE
	****************************************************************************/

	/****************************************************************************
	**	GLOBAL INCLUDE
	**	TIPS: you can put here the library common to all source file
	****************************************************************************/

	/****************************************************************************
	**	DEFINE
	****************************************************************************/

	//max digit of a 8 bit number
	#define MAX_DIGIT8				3
	//max digit of a 16 bit number
	#define MAX_DIGIT16				5
	//max digit of a 32 bit number
	#define MAX_DIGIT32				10

	/****************************************************************************
	**	MACRO
	****************************************************************************/

	/****************************************************************************
	**	TYPEDEF
	****************************************************************************/

	/****************************************************************************
	**	STRUCTURE
	****************************************************************************/

	/****************************************************************************
	**	PROTOTYPE: GLOBAL VARIABILE
	****************************************************************************/

	/****************************************************************************
	**	PROTOTYPE: FUNCTIONS
	****************************************************************************/

	//NOTE: User must care about providing a valid vector with sufficient space. Return the index of the terminator inside the vector
	//Convert a uint8_t in string form, and loads it into a provided vector, terminating it with '\0'
	extern uint8_t u8_to_str( uint8_t num, uint8_t *str );
	//Convert an int8_t to string
	extern uint8_t s8_to_str( int8_t num, uint8_t *str );
	//Convert a uint16_t in string form, and loads it into a provided vector, terminating it with '\0'
	extern uint8_t u16_to_str( uint16_t num, uint8_t *str );
	//Convert an int16_t to string
	extern uint8_t s16_to_str( int16_t num, uint8_t *str );
	//Convert a uint32_t in string form, and loads it into a provided vector, terminating it with '\0'
	extern uint8_t u32_to_str( uint32_t num, uint8_t *str );
	//Convert an uint32_t to string
	extern uint8_t s32_to_str( int32_t num, uint8_t *str );

#else
	#warning "multiple inclusion of the header file at_string.h"
#endif

