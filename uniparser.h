/**********************************************************************************
**	LOGS
***********************************************************************************
**		>2019-06-17
**	found a way to pass handler functions
**		void *this -> g_handler;									//Generic pointer to store function
**		void (*my_void_void_ptr)(void ); 							//temporary pointer
**		my_void_void_ptr = (void (*)(void))this -> g_handler;		//Promote pointer to the right type
**		(*my_void_void_ptr)();										//Execute the pointed function
**	found how to store txt and function handler. Built add_cmd MVP
**	building exe
**		added status. it's going to be upgraded to a structure with special bits
**		each command now has an index that specify the partial it up to a character in the command
**		>2019-06-18
**	Changed index. Now if valid it points to the next character in the dictionary
**		VEL\0 	//V verified. E to verify
**		 ^
**	Changed index. Now 0 is unused. Use zero as no match and promote to unsigned int
**		this -> g_cmd_index[1] = 0	//command 1 is unmatched and is ignored by partial match search
**	Adding argument vector
**		u	0x00	S	0xff	0xff
**		^first argument is U8
**			^content of first argument (U8)0x00 = 0
**					^second argument is S16
**								^content of second argument (S16)0xffff = -1
**	set_u8 argument added. It sets the content of the argument vector for an u8
**		>2019-06-19
**	this -> g_num_match: When Nummatch is negative it means I have a single match and I skip the search
**	implemented structure Arg_descriptor to encode the status of the argument decoder FSM
**	detection of command after argument matching implemented
**	added argument get and set and accumulate for U8.
**	added execution handler for void->void and U8->void functions
**	Successfully decoded and linked function both void and U8
**	multi byte ID match implemented
**	limitation: a command can only have one type of argument
**	success with double argument U8 callback
**	success with quadruple S8 U8 S16
**	adding u16 set_u16 and get_u16 added
**		>2019-06-20
**	added u16 function handler and arguments
**	added FSM transition ID->IDLE
**	TODO added FSM retry function
**	added s32 argument. s32 is limited to two arguments functions
**	decoded s32 argument
**	remove digit handling. not needed anymore
**	added handling of return value for set argument to accumulate_arg, add_arg
**	set argument inlined
**		>2019-06-21
**	cmd_chk now check syntax of command during add
**	added syntax errors
**	added syntax error decoder to string
**	added get_syntax_error to decode last command error in string form
**	added guard against failure of set_
**		>2019-10-09
**	Fixed sign bug in add_cmd
**********************************************************************************/

/**********************************************************************************
**	ENVIROMENT VARIABILE
**********************************************************************************/

#ifndef UNIPARSER_H_
	#define UNIPARSER_H_

/**********************************************************************************
**	GLOBAL INCLUDES
**********************************************************************************/

/**********************************************************************************
**	DEFINES
**********************************************************************************/

//!redudant checks meant for debug only
#define UNIPARSER_PENDANTIC_CHECKS	true
//!Maximum number of commands that can be registered
#define UNIPARSER_MAX_CMD			16
//!Commands can have at most two arguments
#define UNIPARSER_MAX_ARGS			4
//!Size of argument vector. one byte for each identifier plus bytes for the raw data
#define UNIPARSER_ARG_VECTOR_SIZE	13
//!maximum value the argument index can have. arg_index has limited bit allocated to it inside struct _Arg_fsm_status
#define UNIPARSER_MAX_ARG_INDEX		15
//! @todo Upon miss, the FSM will relunch execution of the past # characters allowing partial matches
//#define UNIPARSER_FSM_RETRY			4
//! @todo maximum command length
#define UNIPARSER_MAX_CMD_LENGTH	32

/**********************************************************************************
**	MACROS
**********************************************************************************/

#define IS_SIGN( x )	\
	(((x) == '+') || ((x) == '-'))
//x is a number?
#define IS_NUMBER( x )	\
	(((x) >= '0') && ((x) <= '9'))

#define IS_LOWCASE_LETTER( x )	\
	(((x) >= 'a') && ((x) <= 'z'))

#define IS_HIGHCASE_LETTER( x )	\
	(((x) >= 'A') && ((x) <= 'Z'))

#define IS_LETTER( x )	\
	(IS_LOWCASE_LETTER( x ) || IS_HIGHCASE_LETTER( x ))

//Enumerate possible argument descriptors
#define IS_ARG_DESCRIPTOR( x )	\
	( ((x) == Arg_descriptor::ARG_S8) || ((x) == Arg_descriptor::ARG_U8) || ((x) == Arg_descriptor::ARG_S16) || ((x) == Arg_descriptor::ARG_U16) || ((x) == Arg_descriptor::ARG_S32) )

/**********************************************************************************
**	NAMESPACE
**********************************************************************************/

//! @namespace Orangebot My custom namespace
namespace Orangebot
{

/**********************************************************************************
**	TYPEDEFS
**********************************************************************************/

//! List of error codes of the parser FSM
enum _Err_codes
{
	NO_ERR,					//FSM is Okay
	ERR_INVALID_CMD,		//An invalid command was given
	ERR_ADD_MAX_CMD,		//Parser already contain the maximum number of commands
	ERR_GENERIC				//Uncategorized error
};
typedef enum _Err_codes Err_codes;

//! Statuses of the FSM
enum _Parser_status
{
	PARSER_IDLE,			//IDLE, awaiting for ID
	PARSER_ID,				//ID successfully decoded, decoding command
	PARSER_ARG				//Process input sign or positive or argment number
};
typedef enum _Parser_status Parser_status;

//! Syntax errors in the command
enum _Cmd_syntax_error
{
	SYNTAX_OK,					//Command is valid
	SYNTAX_BAD_POINTER,			//nullptr
	SYNTAX_ARG_TYPE_INVALID,	//An invalid argument descriptor has been used
	SYNTAX_ARG_TYPE_NOTSAME,	//Multiple argument descriptor types in the same command
	SYNTAX_ARG_TOOMANY,			//Too many arguments have been specified for this command
    SYNTAX_ARG_BACKTOBACK,		//At least an ID byte required before an argument
	SYNTAX_LENGTH,				//Command is too long
	SYNTAX_FIRST_NOLETTER		//First byte must be a letter
};
typedef enum _Cmd_syntax_error Cmd_syntax_error;

//! Argument descriptors. used after % in a command to specify that the command has numeric arguments.
enum _Arg_descriptor
{
	ARG_S8 = 's',
	ARG_U8 = 'u',
	ARG_S16 = 'S',
	ARG_U16 = 'U',
	ARG_S32 = 'd'
};
typedef enum _Arg_descriptor Arg_descriptor;

//! argument stride. space inside the argument descriptor vector
enum _Arg_size
{
	ARG_DESCRIPTOR_SIZE = 1,
	ARG_S8_SIZE = 1,
	ARG_U8_SIZE = 1,
	ARG_S16_SIZE = 2,
	ARG_U16_SIZE = 2,
	ARG_S32_SIZE = 4
};
typedef enum _Arg_size Arg_size;

/**********************************************************************************
**	PROTOTYPE: STRUCTURES
**********************************************************************************/

//! status of the argument decoder FSM
struct _Arg_fsm_status
{
	//! Sign of the argument | false = positive | true = negative.
	uint8_t arg_sign	: 1;
	//! Number of arguments fully processed by the argument decoder. Up to 7 arguments
	uint8_t num_arg		: 3;
	//! Index of the argument currently being decoded in the argument vector
	uint8_t arg_index	: 4;
};
typedef struct _Arg_fsm_status Arg_fsm_status;

/**********************************************************************************
**	PROTOTYPE: GLOBAL VARIABILES
**********************************************************************************/

/**********************************************************************************
**	PROTOTYPE: CLASS
**********************************************************************************/

/************************************************************************************/
//! @class 		Uniparser
/************************************************************************************/
//!	@author		Orso Eric
//! @version	0.1 alpha
//! @date		2019/06
//! @brief		Uniparser V4. Universal command decoder with arguments and callback funcrion
//! @details
//! UNIPARSER V4 \n
//!	Adding a parser is a time consuming, bug prone and thankless task. \n
//! This library is the current iteration of a series of libraries meant to minimize \n
//! the workload required to add a command parser to a microcontroller application
//! @pre		No prerequisites
//! @bug		Sign bug: PWMR-127L127 is decoded as -127 | -127 (wrong) instead of -127 | +127 (right) \n
//!				SOLVED | Was caused by the sign of the argument decoder not being initialized if a sign was not specified
//! @warning	No warnings
//! @copyright	License ?
//! @todo		add special error handler function callback. It's passed a code and a string with the error
//! @todo		check argument range | it adds overhead, and doesn't really help much...
/************************************************************************************/

class Uniparser
{
	//Visible to all
	public:
		//--------------------------------------------------------------------------
		//	CONSTRUCTORS
		//--------------------------------------------------------------------------

		//! Default constructor
		Uniparser( void );

		//--------------------------------------------------------------------------
		//	DESTRUCTORS
		//--------------------------------------------------------------------------

		//!Default destructor
		~Uniparser( void );

		//--------------------------------------------------------------------------
		//	OPERATORS
		//--------------------------------------------------------------------------

		//--------------------------------------------------------------------------
		//	SETTERS
		//--------------------------------------------------------------------------

		//! @todo Add special function handler that will be called when the parser experience an error
		//bool add_error_handler( void *handler  );
		//! Add a command to the parser. Provide text that will trigger the call and function to be executed. false=command added successfully
		bool add_cmd( const char * cmd, void *handler );
		bool add_cmd( const char * cmd, void *handler, Cmd_syntax_error &err_code );

		//--------------------------------------------------------------------------
		//	GETTERS
		//--------------------------------------------------------------------------

		//! Decode syntax error of the parser in string form. nullptr means no syntax error detected
		const char *get_syntax_error( void );

		//--------------------------------------------------------------------------
		//	TESTERS
		//--------------------------------------------------------------------------

		//--------------------------------------------------------------------------
		//	PUBLIC METHODS
		//--------------------------------------------------------------------------

		//! Process a byte through the parser. Handler function is automatically called when a full command is decoded
		bool exe( uint8_t data );

		//--------------------------------------------------------------------------
		//	PUBLIC STATIC METHODS
		//--------------------------------------------------------------------------

		//--------------------------------------------------------------------------
		//	PUBLIC VARS
		//--------------------------------------------------------------------------

	//Visible to derived classes
	protected:
		//--------------------------------------------------------------------------
		//	PROTECTED METHODS
		//--------------------------------------------------------------------------

		//--------------------------------------------------------------------------
		//	PROTECTED VARS
		//--------------------------------------------------------------------------

	//Visible only inside the class
	private:
		//--------------------------------------------------------------------------
		//	PRIVATE METHODS
		//--------------------------------------------------------------------------

		//! initialize class vars
		void init( void );

		//!Check command syntax
		Cmd_syntax_error chk_cmd( const uint8_t *cmd );
		//!Decode syntax error code
		const char *decode_syntax_err( Cmd_syntax_error cmd_err );

			//! Argument descriptor group
		//!	initialize argument decoder for a new command
		void init_arg_decoder( void );
        //! add a command to the command string
		bool add_arg( uint8_t cmd_id );
		//!Write an number inside the argument vector. Index must point to an argument descriptor
		bool set_s8( uint8_t arg_index, int8_t data );
		bool set_u8( uint8_t arg_index, uint8_t data );
		bool set_s16( uint8_t arg_index, int16_t data );
		bool set_u16( uint8_t arg_index, uint16_t data );
		bool set_s32( uint8_t arg_index, int32_t data );
		//! fetch an U8 argument from the argument vector
		int8_t get_s8( uint8_t arg_index );
		uint8_t get_u8( uint8_t arg_index );
		int16_t get_s16( uint8_t arg_index );
		uint16_t get_u16( uint8_t arg_index );
		int32_t get_s32( uint8_t arg_index );
		//! decode an input character and accumulate it into the argument vector
		bool accumulate_arg( uint8_t data );
		//! Argument has been fully decoded into argument string. Update argument descriptor FSM.
		bool close_arg( void );

			//! Execute Callback
		//! Execute the handler of function of index cmd_id. Arguments are to be axtracted from the argument vector.
		bool exe_handler( uint8_t exe_index );

		//--------------------------------------------------------------------------
		//	PRIVATE VARS
		//--------------------------------------------------------------------------

			/// Parser dictionary and handler functions
		// Number of commands currently registered inside the parser
		uint8_t g_num_cmd;
		//String that will trigger a command detetion and execute the callback
		const uint8_t *g_cmd_txt[UNIPARSER_MAX_CMD];
		//Partial hit status inside each command. 0 means no hit. Point to the next unmatched char in a command.
		uint8_t g_cmd_index[UNIPARSER_MAX_CMD];
		//Register the callback to be executed when the command is decoded
		void *g_cmd_handler[UNIPARSER_MAX_CMD];
		//latest error code of the dictionary
		Cmd_syntax_error g_cmd_err;

			///	Argument Decoder
		//! Structure that encode the status of the argument decoder FSM
		Arg_fsm_status g_arg_fsm_status;
		//! Arguments decoded are stored in this string
		uint8_t g_arg[UNIPARSER_ARG_VECTOR_SIZE];

			/// FSM working variables
		//number of partial matches
		//	0: no matches or parser IDLE
		//	1: just one match found. The number will become negative to skip search of commandin g_cmd_index[]
		//	>0: a number of partial matches have been found.
		//	<0: just one match found. g_num_match now holds the index of the partially matched command
		int8_t g_num_match;
		//State of the FSM
		Parser_status g_status;
		//Error status of the parser. NO_ERR means OK
		Err_codes g_err;

};	//End Class: Uniparser

/**********************************************************************************
**	NAMESPACE
**********************************************************************************/

} //End Namespace

#else
    #warning "Multiple inclusion of hader file"
#endif
