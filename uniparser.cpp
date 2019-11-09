/****************************************************************************
**	OrangeBot Project
*****************************************************************************
**        /
**       /
**      /
** ______ \
**         \
**          \
*****************************************************************************
**	UNIVERSAL PARSER
*****************************************************************************
**	Author: 			Orso Eric
**	Creation Date:		2019-06-17
**	Last Edit Date:		2019-10-09
**	Revision:			2
**	Version:			4.1
****************************************************************************/

/****************************************************************************
**	DESCRIPTION
*****************************************************************************
**	Do a universap parser that does not require malloc/free and do not use explicit dictionary
**	I create a special register command function in which the message and an handler is provided
****************************************************************************/

/****************************************************************************
**	COMMAND RULES
*****************************************************************************
**		ADD COMMAND TO PARSER
**	A command is composed by fixed alphanumeric chars and argument descriptors
**	Each command is associated with a unique function callback
**	A command can only start with letters
**
**		EXAMPLE ADD COMMAND TO PARSER
**	myparser.add_cmd("P", (void *)&my_ping_handler );
**	Add a new command that is triggered when the string P\0 is received.
**	function my_ping_handler will be automatically executed when the \0 is processed
**	myparser.exe( 'P' );
**	myparser.exe( '\0' );
**	send manually bytes to the parser to test the system
*****************************************************************************
**	Command restriction:
**	>Can only start with a letter
**	>Valid arguments are %u %s %U %S %d
**	>There can be no commands with same start but two different argument type. Only the first one will be considered in case
**	>After valid ID access parameters using Parser Macros
****************************************************************************/


/****************************************************************************
**	KNOWN BUG
*****************************************************************************
**
****************************************************************************/

/****************************************************************************
**	INCLUDES
****************************************************************************/

#include <stdint.h>
//#define ENABLE_DEBUG
#include "debug.h"
//Class Header
#include "uniparser.h"

/****************************************************************************
**	NAMESPACES
****************************************************************************/

namespace Orangebot
{

/****************************************************************************
**	GLOBAL VARIABILES
****************************************************************************/

/****************************************************************************
*****************************************************************************
**	CONSTRUCTORS
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Empty Constructor
//!	Uniparser | void
/***************************************************************************/
// @param
//! @return no return
//!	@details
//! Empty constructor
/***************************************************************************/

Uniparser::Uniparser( void )
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
	//! @details algorithm:

	//Initialize structure to safe values
	this -> init();
	//Pass a terminator to the parser to have it initialize itself
	this -> exe( '\0' );

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return;
}	//end constructor:

/****************************************************************************
*****************************************************************************
**	DESTRUCTORS
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Empty Destructor
//!	Uniparser | void
/***************************************************************************/
// @param
//! @return no return
//!	@details
//! Empty destructor
/***************************************************************************/

Uniparser::~Uniparser( void )
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
	//! @details algorithm:

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return;
}	//end destructor:

/****************************************************************************
*****************************************************************************
**	OPERATORS
*****************************************************************************
****************************************************************************/

/****************************************************************************
*****************************************************************************
**	SETTERS
*****************************************************************************
****************************************************************************/

/****************************************************************************
*****************************************************************************
**	GETTERS
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Public Getter
//!	get_syntax_error | void
/***************************************************************************/
// @param
//! @return no return
//!	@details
//! Decode syntax error of the parser in string form. nullptr means no syntax error detected
/***************************************************************************/

const char *Uniparser::get_syntax_error( void )
{
	DENTER();

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	const char *ret_str;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Decode error code
	ret_str = this -> decode_syntax_err( this -> g_cmd_err );

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	DRETURN_ARG(">%s<\n",ret_str);
	return ret_str;
}	//end method: get_syntax_error | void

/****************************************************************************
*****************************************************************************
**	TESTERS
*****************************************************************************
****************************************************************************/

/****************************************************************************
*****************************************************************************
**	PUBLIC METHODS
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Public Method
//!	add_command | const char *, void *
/***************************************************************************/
//! @param cmd | const char * 		Text that will trigger the command
//!	@param handler | void * 		Pointer to callback function for this command
//! @return bool | false: OK | true: fail
//!	@details
//! Add a string and a function pointer to the parser
//!	@todo check that command is valid
/***************************************************************************/

bool Uniparser::add_cmd( const char *cmd, void *handler )
{
	DENTER_ARG("cmd: %p >%s<\n", (void *)cmd, cmd );

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//index
	uint8_t t;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//if: input is invalid
	if ((cmd == nullptr) || (handler == nullptr))
	{
		this -> g_err = ERR_INVALID_CMD;
		DRETURN_ARG("ERR%d: ERR_INVALID_CMD\n", this -> g_err);
		return true;	//fail
	}
	//If: num command is invalid
	if ((UNIPARSER_PENDANTIC_CHECKS) && ((this -> g_num_cmd < 0) || (this->g_num_cmd >= UNIPARSER_MAX_CMD)) )
	{
		this -> g_err = ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}
	//if: maximum number of command has been reached
	if (this -> g_num_cmd >= (UNIPARSER_MAX_CMD-1))
	{
		this -> g_err = ERR_ADD_MAX_CMD;
		DRETURN_ARG("ERR%d: ERR_ADD_MAX_CMD in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}
	// check the validity of the string
	this -> g_cmd_err = this -> chk_cmd((const uint8_t *)cmd);
	//If: command had a syntax error
	if (this -> g_cmd_err != Cmd_syntax_error::SYNTAX_OK)
	{
		DRETURN_ARG("command didnt get past argument descriptor check\n");
		return true;
	}

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Fetch currently used command
	t = this -> g_num_cmd;
	//Link command handler and command text
	this -> g_cmd_txt[t] = (uint8_t *)cmd;
	this -> g_cmd_handler[t] = handler;
	DPRINT("Command >%s< with handler >%p< has been added with index: %d\n", cmd, (void *)handler, t);
	//A command has been added
	this -> g_num_cmd = t +1;
	DPRINT("Total number of commands: %d\n", this -> g_num_cmd);

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	DRETURN();
	return false;
}	//end method: add_command | const char *, void *

/***************************************************************************/
//!	@brief Public Method
//!	add_cmd | const char *, void *, Cmd_syntax_error &
/***************************************************************************/
//! @param cmd | string containing the command
//! @param handler | pointer to callback function for the command
//! @param err_code | return the error code of the command if any
//! @return Cmd_syntax_error::SYNTAX_OK if all is good. other codes specify error. decode_err will return a string with error descritpion.
//!	@details
//! Add command to dictionary
/***************************************************************************/

bool Uniparser::add_cmd( const char *cmd, void *handler, Cmd_syntax_error &err_code )
{
	DENTER_ARG("cmd: %p >%s<\n", (void *)cmd, cmd );

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//index
	uint8_t t;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//if: input is invalid
	if ((cmd == nullptr) || (handler == nullptr))
	{
		this -> g_err = ERR_INVALID_CMD;
		DRETURN_ARG("ERR%d: ERR_INVALID_CMD\n", this -> g_err);
		return true;	//fail
	}
	//If: num command is invalid
	if ((UNIPARSER_PENDANTIC_CHECKS) && ((this->g_num_cmd < 0) || (this->g_num_cmd >= UNIPARSER_MAX_CMD)) )
	{
		this -> g_err = ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}
	//if: maximum number of command has been reached
	if (this -> g_num_cmd >= (UNIPARSER_MAX_CMD-1))
	{
		this -> g_err = ERR_ADD_MAX_CMD;
		DRETURN_ARG("ERR%d: ERR_ADD_MAX_CMD in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}

	// check the validity of the string
	err_code = this -> chk_cmd((const uint8_t *)cmd);
	//If: command had a syntax error
	if (err_code != Cmd_syntax_error::SYNTAX_OK)
	{
		DRETURN_ARG("command didnt get past argument descriptor check\n");
		return true;
	}

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Fetch currently used command
	t = this -> g_num_cmd;
	//Link command handler and command text
	this -> g_cmd_txt[t] = (uint8_t *)cmd;
	this -> g_cmd_handler[t] = handler;
	DPRINT("Command >%s< with handler >%p< has been added with index: %d\n", cmd, (void *)handler, t);
	//A command has been added
	this -> g_num_cmd = t +1;
	DPRINT("Total number of commands: %d\n", this -> g_num_cmd);

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	DRETURN();
	return false;
}	//end method: add_cmd | const char *, void *, Cmd_syntax_error &

/***************************************************************************/
//!	@brief Public Method
//!	exe | char
/***************************************************************************/
// @param
//! @return no return
//!	@details
//! Intricate FSM. The objective is to check the incoming character against
//!	charaters inside the various commands and decode a command when a \0 is detected
/***************************************************************************/

bool Uniparser::exe( uint8_t data )
{
	if ((data < '0') || (data > 'z'))
	{
		DENTER_ARG("exe: >0x%x<\n", data );
	}
	else
	{
		DENTER_ARG("exe: >0x%x< >%c<\n", data, data );
	}

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//when true, reset the FMS
	bool f_rst_fsm = false;
	//Index of the handler to be executer
	int8_t exe_index = -1;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------
	//! @details algorithm:
	//!

		//----------------------------------------------------------------
		//	TERMINATOR
		//----------------------------------------------------------------
		//! @details when terminator is detected, close the command.
		//!	I either have a full match, or the command is invalid and I have to reset the FSM

	//If: input terminator from user
	if (data == '\0')
	{
		DPRINT("Terminator detected | Number of partial matches: %d\n", this -> g_num_match);
		//counter
		uint8_t t;
		//If: i was decoding an argument
		if (this -> g_status == Orangebot::Parser_status::PARSER_ARG)
		{
			//I'm done decoding
			DPRINT("Terminator after ARG\n");
			//Close current argument and update the argument detector FSM.
			bool f_ret = this -> close_arg();
			//if could not close argument
			if (f_ret == true)
			{
				//I can recover from this.
				//no matches and reset the FSM.
				this -> g_num_match = 0;
			}
		}
		//Index inside the command
		uint8_t cmd_index;
		//if: I have at least one parser_tmp.id_index entry
		if (this -> g_num_match > 0)
		{
			DPRINT("Scanning partially matched commands for terminators\n");
			//For: scan all commands
			for (t = 0;t<this -> g_num_cmd;t++)
			{
				//Fetch command index
				cmd_index = this -> g_cmd_index[t];
				//If: this command is a partial match
				if (cmd_index > 0)
				{
					//If: i was decoding an argument
					if (this -> g_status == Parser_status::PARSER_ARG)
					{
						//If: index is not pointing to an argument descriptor inside the command
						if ((UNIPARSER_PENDANTIC_CHECKS) && (this -> g_cmd_txt[t][cmd_index] != '%'))
						{
							DPRINT("ERR: This command should have an argument descriptor in this position. | cmd: %d | cmd_index: %d | actual content: >0x%x<\n", t, cmd_index, this -> g_cmd_txt[t][cmd_index]);
							this -> g_err = Err_codes::ERR_GENERIC;
							DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
							return true;	//fail
						}
						//The argument has been closed. I need to skip the argument descriptor "%?"
						cmd_index += 2;
						//With argument closed I'm now doing an ID matching for terminator. Can be skipped since it's the last char
						this -> g_status = Parser_status::PARSER_ID;
						//I should write back the index. Can be skipped since it's the last char
						this -> g_cmd_index[t] = cmd_index;
					}
					//Check match  against the character after the one already matched.
					//If: the next command data would be the terminator '\0'
					if ( this -> g_cmd_txt[t][ cmd_index ] == '\0')
					{
						//If there is a pending execution already
						if ((UNIPARSER_PENDANTIC_CHECKS) && (exe_index != -1))
						{
							//this should never happen
							this -> g_err = Err_codes::ERR_GENERIC;
							DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
							return true;	//fail
						}
						//Issue execution of the callback function linked
						exe_index = t;
						DPRINT("Valid command ID%d decoded\n", t);
						//I can stop the search
						t = this -> g_num_cmd;
					}
					else
					{
						DPRINT("no match in %x cmd %x | " ,data , this -> g_cmd_txt[t][ cmd_index ] );
					}
				}	//End If: this command is a partial match
			}	//End For: scan all commands
		}	//end if: I have at least one parser_tmp.id_index entry
		//If I have just one match. g_num_match now holds the index of the match
		else if (this -> g_num_match < 0)
		{
			//decode command index
			t = -this -> g_num_match;
			DPRINT("just one partial match: %d\n", t);
			//Fetch command index
			cmd_index = this -> g_cmd_index[t];
			//If: i was decoding an argument
			if (this -> g_status == Parser_status::PARSER_ARG)
			{
				//If: index is not pointing to an argument descriptor inside the command
				if ((UNIPARSER_PENDANTIC_CHECKS) && (this -> g_cmd_txt[t][cmd_index] != '%'))
				{
					DPRINT("ERR: This command should have an argument descriptor in this position. | cmd: %d | cmd_index: %d | actual content: >0x%x<\n", t, cmd_index, this -> g_cmd_txt[t][cmd_index]);
					this -> g_err = Err_codes::ERR_GENERIC;
					DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
					return true;	//fail
				}
				//The argument has been closed. I need to skip the argument descriptor "%?"
				cmd_index += 2;
				//With argument closed I'm now doing an ID matching for terminator. Can be skipped since it's the last char
				//this -> g_status == Parser_status::PARSER_ID;
			}
			//Check match  against the character after the one already matched.
			//If: the next command data would be the terminator '\0'
			if ( this -> g_cmd_txt[t][ cmd_index ] == '\0')
			{
				//If there is a pending execution already
				if ((UNIPARSER_PENDANTIC_CHECKS) && (exe_index != -1))
				{
					//this should never happen
					this -> g_err = Err_codes::ERR_GENERIC;
					DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
					return true;	//fail
				}
				//Issue execution of the callback function linked
				exe_index = t;
				DPRINT("Valid command ID%d decoded\n", t);
				//I can stop the search
				t = this -> g_num_cmd;
			}
			//if: I'm given a terminator but dictionary does not contain a terminator
			else
			{
				//This happen if user gives a command that lack one char
				DPRINT("no match. given: >0x%x< expected: >0x%x<\n" ,data , this -> g_cmd_txt[t][ cmd_index ] );
				//Issue a FSM reset
				f_rst_fsm = true;
			}
		}
		//if: no partial matches
		else
		{
			//do nothing
		}
		//Issue a FSM reset
		f_rst_fsm = true;
	}	//End If: input terminator from user

		//--------------------------------------------------------------------------
		//	PARSER_IDLE
		//--------------------------------------------------------------------------
		//	Only letters can be used as first character in a command
		//	This section matches the first character in each command
		//	This section takes care of initializing g_cmd_index[] to valid partial match values

	//If: PARSER_IDLE
	else if (this -> g_status == Parser_status::PARSER_IDLE)
	{
		DPRINT("PARSER_IDLE\n");
		//If: letter First character in a command can only be a letter
		if (IS_LETTER( data ))
		{
			//counter
			uint8_t t;
			//for each command
			for (t = 0; t < this -> g_num_cmd;t++)
			{
				//if: dictionary is unlinked
				if ((UNIPARSER_PENDANTIC_CHECKS) && (this -> g_cmd_txt[t] == nullptr))
				{
					this -> g_err = Err_codes::ERR_GENERIC;
					DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
					return true;	//fail
				}
				//If: partial match
				if (this -> g_cmd_txt[t][0] == data)
				{
					//A partial match has been found
					this -> g_num_match++;
					//Match has been found up to first character. Point to the next unmatched char
					this -> g_cmd_index[t] = 1;
					//Next, I'm matching ID entries
					this -> g_status = Parser_status::PARSER_ID;
					//TIP: I can't speculatively detect % here because two commands may have the same first section and diverge later.
					DPRINT("Match command %d, total partial matches: %d\n", t, this -> g_num_match);
				}
				//if: no match
				else
				{
					//special code for no match found
					this -> g_cmd_index[t] = 0;
				}
			}	//end for: each command
		}	//End If: letter
		//Non letter can never be a first character
		else
		{
			//Issue a FSM reset
			f_rst_fsm = true;
		}
	}	//End If: PARSER_IDLE

		//--------------------------------------------------------------------------
		//	ID matching
		//--------------------------------------------------------------------------

	//if: I'm ID matching
	else if (this -> g_status == Parser_status::PARSER_ID)
	{
		DPRINT("PARSER_ID ");
		//! @todo only go ARG if a command has an arg descriptor if there are number or sign. allow number or sign to be used as command ID
		//if: I'm being fed an argument
		if (IS_NUMBER( data ) || IS_SIGN( data ))
		{
			DPRINT_NOTAB("- ARG | num_match %d\n", this -> g_num_match);
			//if: I have at least one partial match
			if (this -> g_num_match > 0)
			{
				//for each dictionary command
				for (uint8_t t = 0;t < this -> g_num_cmd;t++)
				{
					//if: the command is a partial match
					if (this -> g_cmd_index[t] > 0)
					{
						//Search in the dictionary for a % entry. An argument descriptor
						if (this -> g_cmd_txt[t][ this -> g_cmd_index[t] ] == '%')
						{
								//! This is the fi
							//Do not increment index but go in ARG parsing mode
							this -> g_status = Parser_status::PARSER_ARG;
							//Do not advance index until argument decoding is complete
							DPRINT("ARG begins | command: %d\n", t);
							//Add an argument using current partial match as template
							bool f_ret = this -> add_arg( t );
							//if: adding argument failed
							if ((UNIPARSER_PENDANTIC_CHECKS) && (f_ret == true))
							{
								this -> g_err = Err_codes::ERR_GENERIC;
								DPRINT("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
								//I can recover from this by resetting the FSM
								f_rst_fsm = true;
							}
							//initialize agrument
							this -> accumulate_arg( data );
							//argument detection make the detection unique. Remove all other partial matches
							this -> g_num_match = -t;
							//Single match has been found. Break the cycle
							t = UNIPARSER_MAX_CMD;
						}
						//Else: this dictionary entry does not contain an argument descriptor
						else
						{
							//Prune away the partial match
							this -> g_cmd_index[t] = 0;
							DPRINT("Prune away partial match: %d | ", t);
							//if: I still have partial matches after pruning
							if (this -> g_num_match >= 2)
							{
								//One fewer partial match
								this -> g_num_match--;
								DPRINT_NOTAB(" num_match: %d\n", this -> g_num_match );
							}
							//if: all partial matches have been pruned away
							else
							{
								//No more commands. Reset the machine
								f_rst_fsm = true;
								//No more matches. Can breack cycle early
								t = UNIPARSER_MAX_CMD;
								DPRINT_NOTAB("Last partial match has been pruned away... RESET\n");
								//! @todo command refeed function. Safe and refeed last char to detect other partial commands
							}
						}
					}
					else
					{
						//do nothing
					}
				} //end for each dictionary command
			}	//end if: I have at least one partial match
			//If I only have one match
			else if (this -> g_num_match < 0)
			{
				//decode command index
				uint8_t t = -this -> g_num_match;
				//Search in the dictionary for a % entry. An argument descriptor
				if (this -> g_cmd_txt[t][ this -> g_cmd_index[t] ] == '%')
				{
						//! This is the first char of an argument
					//Do not increment index but go in ARG parsing mode
					this -> g_status = Parser_status::PARSER_ARG;
					//Do not advance index until argument decoding is complete
					DPRINT("ARG begins | command: %d\n", t);
					//Add an argument using current partial match as template
					bool f_ret = this -> add_arg( t );
					//if: adding argument failed
					if ((UNIPARSER_PENDANTIC_CHECKS) && (f_ret == true))
					{
						this -> g_err = Err_codes::ERR_GENERIC;
						DPRINT("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
						//I can recover from this by resetting the FSM
						f_rst_fsm = true;
					}
					//!	accumulate argument character inside argument.
					f_ret = this -> accumulate_arg( data );
					//if: adding argument failed
					if ((UNIPARSER_PENDANTIC_CHECKS) && (f_ret == true))
					{
						this -> g_err = Err_codes::ERR_GENERIC;
						DPRINT("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
						//I can recover from this by resetting the FSM
						f_rst_fsm = true;
					}
				}
				//Else: this dictionary entry does not contain an argument descriptor
				else
				{
					DPRINT("Command does not have argument in this position. Prune away partial match: %d\n", t);
					//Issue a FSM reset
					f_rst_fsm = true;
				}
			}	//If I only have one match
			//if: no matches
			else
			{
				//I shouldn't have been in ID to begin with, but I can recover from this error
				DPRINT("ERR: FSM was in ID matching but no partial matches were detected.\n");
				//Issue a FSM reset
				f_rst_fsm = true;
			}
		}	//end if: I'm being fed an argument
		//if: I'm matching a non argument non terminator
		else
		{
			DPRINT_NOTAB("- ID\n");
			//if: I have at least one partial match
			if (this -> g_num_match > 0)
			{
				//If I have just one match, I can upgrade num_match to skip for next time
				bool f_match = (this -> g_num_match == 1);
				//for each dictionary command
				for (uint8_t t = 0;t < this -> g_num_cmd;t++)
				{
					//if: the command is a partial match
					if (this -> g_cmd_index[t] > 0)
					{
						//check that the dictionary holds the same value as data
						if (this -> g_cmd_txt[t][ this -> g_cmd_index[t] ] == data)
						{
							//Advance to the next dictionary entry for this command
							this -> g_cmd_index[t]++;
							//if: This is the sole partial match surviving
							if (f_match == true)
							{
								//Save the index inside num match to skip the for next time
								this -> g_num_match = -t;
								//Single match has been found. Break the cycle
								t = UNIPARSER_MAX_CMD;
								DPRINT("Only one partial match remains. Num_match: %d\n", this -> g_num_match);
							}
						}
						//Else: this dictionary entry does not contain an argument descriptor
						else
						{
							//Prune away the partial match
							this -> g_cmd_index[t] = 0;
							DPRINT("Prune away partial match: %d | ", t);
							//if: I still have partial matches after pruning
							if (this -> g_num_match >= 2)
							{
								//One fewer partial match
								this -> g_num_match--;
								DPRINT_NOTAB(" num_match: %d\n", this -> g_num_match );
							}
							//if: all partial matches have been pruned away
							else
							{
								//No more commands. Reset the machine
								f_rst_fsm = true;
								//No more matches. Can breack cycle early
								t = UNIPARSER_MAX_CMD;
								DPRINT_NOTAB("Last partial match has been pruned away... RESET\n");
								//! @todo command refeed function. Safe and refeed last char to detect other partial commands
							}
						}
					}
					//if: the command was pruned away long ago
					else
					{
						//do nothing
					}
				} //end for each dictionary command
			}	//end if: I have at least one partial match
			//If I only have one match
			else if (this -> g_num_match < 0)
			{
				//decode command index
				uint8_t t = -this -> g_num_match;
				//check that the dictionary holds the same value as data
				if (this -> g_cmd_txt[t][ this -> g_cmd_index[t] ] == data)
				{
					//Match! Scan next entry
					this -> g_cmd_index[t]++;
				}
				//no match
				else
				{
					//Last partial match has been pruned away. Issue a FSM reset
					f_rst_fsm = true;
					DPRINT("Last match was pruned away. Expected >0x%x< got >0x%x< instead\n", this -> g_cmd_txt[t][ this -> g_cmd_index[t] ], data);
				}
			}	//If I only have one match
			//if: no matches
			else
			{
				//I shouldn't have been in ID to begin with, but I can recover from this error
				DPRINT("ERR: FSM was in ID matching but no partial matches were detected.\n");
				//Issue a FSM reset
				f_rst_fsm = true;
			}
		}	//end if: I'm matching a non argument non terminator
	}	//end if: I'm ID matching

		//--------------------------------------------------------------------------
		//	ARG decoder
		//--------------------------------------------------------------------------

	//if: I'm decoding arguments
	else if (this -> g_status == Parser_status::PARSER_ARG)
	{
		DPRINT("PARSER_ARG\n");
		//If: I'm fed a number
		if (IS_NUMBER(data))
		{
			//!	accumulate argument character inside argument.
			bool f_ret = this -> accumulate_arg( data );
			//if: adding argument failed
			if ((UNIPARSER_PENDANTIC_CHECKS) && (f_ret == true))
			{
				this -> g_err = Err_codes::ERR_GENERIC;
				DPRINT("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
				//I can recover from this by resetting the FSM
				f_rst_fsm = true;
			}
			//Do not advance to next dictionary entry
		}	//If: I'm fed a number
		//if: I'm fed a non number
		else
		{
			DPRINT("Closing argument\n");
			//! Exit argument mode
			//Close current argument and update argument FSM
			this -> close_arg();
			//if: I have no matches or multiple matches. I should have just one match by this point.
			if ((UNIPARSER_PENDANTIC_CHECKS) && (this -> g_num_match >= 0))
			{
				this -> g_err = Err_codes::ERR_GENERIC;
				DPRINT("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
				//I can recover from this by resetting the FSM
				f_rst_fsm = true;
			}
			else
			{
				//After argument, I can only have one match.
				uint8_t cmd_id = -this -> g_num_match;
				//Update the parser index by skipping % and the argument descriptor
				this -> g_cmd_index[ cmd_id ] += 2;
				//check that the dictionary holds the same value as data
				if (this -> g_cmd_txt[ cmd_id ][ this -> g_cmd_index[ cmd_id ] ] == data)
				{
					//Advance to the next dictionary entry for this command
					this -> g_cmd_index[ cmd_id ]++;
				}
				//No match
				else
				{
					DPRINT("Pruning away last match\n");
					//I can recover from this by resetting the FSM
					f_rst_fsm = true;
				}
				this -> g_status = Parser_status::PARSER_ID;
			}
		}	//if: I'm fed a non number
	}	//if: I'm decoding arguments

		//----------------------------------------------------------------
		//	FSM RESET
		//----------------------------------------------------------------
		//!	@detail
		//! FSM reset. Clear up the machine for the next clean execution

	//If: a reset was issued
	if (f_rst_fsm == true)
	{
		DPRINT("FSM RESET\n");
		//Clear reset flag
		f_rst_fsm = false;
		//Status becomes IDLE
		this -> g_status = Orangebot::Parser_status::PARSER_IDLE;
		//I have no partial matches anymore
		this -> g_num_match = 0;
		//If I don't have a pending execution
		if (exe_index == -1)
		{
			//Reset the argument decoder and prepare for a new command
			this -> init_arg_decoder();
		}
	}	//If: a reset was issued

		//----------------------------------------------------------------
		//	HANDLER EXECUTION
		//----------------------------------------------------------------
		//!	@detail
		//! FSM reset. Clear up the machine for the next clean execution

	//If: an execution event has been launched
	if (exe_index > -1)
	{
		//if execution index is out of range.
		if ((UNIPARSER_PENDANTIC_CHECKS) && (exe_index == this -> g_num_cmd))
		{
			this -> g_err = Err_codes::ERR_GENERIC;
			DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
			return true;	//fail
		}
		DPRINT("Executing handler of command %d | num arguments: %d\n", exe_index, this -> g_arg_fsm_status.num_arg);
		//Execute handler of given function. Automatically deduce arguments from argument vector
        this -> exe_handler( exe_index );
        //Reset the argument decoder and prepare for a new command
		this -> init_arg_decoder();
	}	//If: a reset was issued

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	//Trace Return from main
	DRETURN();

	return false;	//OK
}	//end method:

/****************************************************************************
*****************************************************************************
**	PUBLIC STATIC METHODS
*****************************************************************************
****************************************************************************/

/****************************************************************************
*****************************************************************************
**	PRIVATE METHODS
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Private Method
//!	init | void
/***************************************************************************/
//! @return no return
//!	@details
//! initialize structure
/***************************************************************************/

inline void Uniparser::init( void )
{
	DENTER();

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
	//! @details initialize structure

	//No commands are currently loaded inside the parser
	this -> g_num_cmd = 0;
	//for: each possible command
	for (t = 0;t < UNIPARSER_MAX_CMD;t++)
	{
		//command has no txt identifier linked
		this -> g_cmd_txt[t] = nullptr;
		//command has no function handler linked
		this -> g_cmd_handler[t] = nullptr;
	}
	//I have no partial matches
	this -> g_num_match = 0;
	//FSM begins in idle
	this -> g_status = Orangebot::Parser_status::PARSER_IDLE;
	//No error
	this -> g_err = Orangebot::Err_codes::NO_ERR;

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	//Trace Return from main
	DRETURN();

	return;	//OK
}	//end method: init | void

/***************************************************************************/
//!	@brief Private Method
//!	init_arg_decoder | void
/***************************************************************************/
//!	@details
//! initialize argument decoder for a new command
/***************************************************************************/

inline void Uniparser::init_arg_decoder( void )
{
	//Trace Enter with arguments
	DENTER();

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------
	//! @details algorithm:

	//Prepare the argument descriptor. zero arguments are in store
	this -> g_arg_fsm_status.num_arg = 0;
	//First free slot in the argument vector is the first byte
	this -> g_arg_fsm_status.arg_index = 0;
	//if sign is not specified, default argument sign is plus
	this -> g_arg_fsm_status.arg_sign = false;

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	//Trace Return vith return value
	DRETURN();

	return; //OK
}	//end method: init_arg_decoder | void

/***************************************************************************/
//!	@brief Private Method
//!	chk_cmd | const uint8_t *
/***************************************************************************/
//! @param cmd |
//! @return syntax error | SYNTAX_OK means the command has no errors and can be parsed safely
//!	@details
//! Check command syntax
/***************************************************************************/

Cmd_syntax_error Uniparser::chk_cmd( const uint8_t *cmd )
{
	//Trace Enter with arguments
	DENTER_ARG("cmd: %p\n", (void *)cmd);

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//Counter
	uint8_t t;
	//initialize error vreturn
	Cmd_syntax_error err = SYNTAX_OK;
	//Remember first argument descriptor used
	uint8_t arg_mem = 0;
	//remember number of argument descriptors
	uint8_t arg_num = 0;
	//Pointer to syntax error string
	const char *str = nullptr;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	if (cmd == nullptr)
	{
		err = Cmd_syntax_error::SYNTAX_BAD_POINTER;
		DRETURN_ARG("ERR%d: | Bad handler function pointer\n", err);
		return err;
	}

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//if: first char is not a letter
	if (!IS_LETTER(cmd[0]))
	{
		err = Cmd_syntax_error::SYNTAX_LENGTH;
		DRETURN_ARG("ERR%d | First character of a command must be a letter\n", err);
		return err;
	}

	//Initialize counter
	t = 0;
	//While: parsing is not done
	while ((cmd[t] != '\0') && (t < UNIPARSER_MAX_CMD_LENGTH))
	{
		//if: argument descriptor
		if (cmd[t] == '%')
		{
			//i have an argument descriptor
			arg_num++;
			//if: first argument descriptor
			if (arg_num == 1)
			{
				arg_mem = cmd[t+1];
			}
			//if: following argument descriptors
			else
			{
				//if i'm using multiple types of argument descriptors
				if (arg_mem != cmd[t+1])
				{
					err = Cmd_syntax_error::SYNTAX_ARG_TYPE_NOTSAME;
					str = this -> decode_syntax_err( err );
					DRETURN_ARG("ERR%d | %s\n", err, str);
					return err;
				}
			}

			if ((arg_mem == Arg_descriptor::ARG_S32) && (arg_num > 2))
			{
				err = Cmd_syntax_error::SYNTAX_ARG_TOOMANY;
				str = this -> decode_syntax_err( err );
				DRETURN_ARG("ERR%d | %s | max is two S32\n", err, str);
				return err;
			}
			else if (arg_num > 4)
			{
				err = Cmd_syntax_error::SYNTAX_ARG_TOOMANY;
				str = this -> decode_syntax_err( err );
				DRETURN_ARG("ERR%d | %s | max is four S8 or U8 or S16 or U16\n", err, str);
				return err;
			}
		}
		//if: bad argument descriptor
		if ((cmd[t] == '%') && (!IS_ARG_DESCRIPTOR(cmd[t+1])))
		{
			err = Cmd_syntax_error::SYNTAX_ARG_TYPE_INVALID;
			str = this -> decode_syntax_err( err );
			DRETURN_ARG("ERR%d | %s | Valid arguments descriptors are: %c %c %c %c %c\n", err, str, Arg_descriptor::ARG_S8, Arg_descriptor::ARG_U8, Arg_descriptor::ARG_S16, Arg_descriptor::ARG_U16, Arg_descriptor::ARG_S32);
			return err;
		}
		//if: two arguments back to back
		if ( (cmd[t] == '%') && (cmd[t+2] == '%') )
		{
			err = Cmd_syntax_error::SYNTAX_ARG_BACKTOBACK;
			str = this -> decode_syntax_err( err );
			DRETURN_ARG("ERR%d | %s\n", err, str);
			return err;
		}
		//Parse next byte
		t++;
	}
	//If size is 255
	if (t == UNIPARSER_MAX_CMD_LENGTH)
	{
		err = Cmd_syntax_error::SYNTAX_LENGTH;
		str = this -> decode_syntax_err( err );
		DRETURN_ARG("ERR%d | %s\n", err, str );
		return err;
	}

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	DRETURN();
	return err;
}	//end method: chk_cmd | const uint8_t *

/***************************************************************************/
//!	@brief Private Method
//!	decode_syntax_err | Cmd_syntax_error
/***************************************************************************/
//! @param cmd_err | syntax error returned from chk_cmd
//! @return string explaining the error. nullptr means all is good
//!	@details
//! Decode syntax error code.
/***************************************************************************/

const char *Uniparser::decode_syntax_err( Cmd_syntax_error cmd_err )
{
	DENTER_ARG("err: %d\n", cmd_err);

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	const char *str;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//switch: syntax error
	switch (cmd_err)
	{
		case Cmd_syntax_error::SYNTAX_OK:
		{
			str = "";
			break;
		}
		case Cmd_syntax_error::SYNTAX_BAD_POINTER:
		{
			str = "Bad handler function pointer";
			break;
		}
		case Cmd_syntax_error::SYNTAX_ARG_TYPE_INVALID:
		{
			str = "Invalid argument descriptor";
			break;
		}
		case Cmd_syntax_error::SYNTAX_ARG_TYPE_NOTSAME:
		{
			str = "You must use only arguments of a single type";
			break;
		}
		case Cmd_syntax_error::SYNTAX_ARG_TOOMANY:
		{
			str = "Too many arguments";
			break;
		}
		case Cmd_syntax_error::SYNTAX_ARG_BACKTOBACK:
		{
			str = "Arguments must be separated by an ID character";
			break;
		}
		case Cmd_syntax_error::SYNTAX_LENGTH:
		{
			str = "Command too long...";
			break;
		}
		case Cmd_syntax_error::SYNTAX_FIRST_NOLETTER:
		{
			str = "Command must begin with a letter";
			break;
		}
        default:
        {
			str = "Error. Unrecognized error!";
			break;
        }
	}	//end switch: syntax error

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	DRETURN_ARG("Decoded: >%s<\n", str);
	return str;
}	//end method: decode_syntax_err | Cmd_syntax_error

/***************************************************************************/
//!	@brief Private Method
//!	add_arg | uint8_t
/***************************************************************************/
//! @param cmd_id | index of the command the argument is inferred from
//! @return false: ok | true: fail
//!	@details
//! Add an argument to the parser argument storage.
//!	It expect the command to be a partial match and the index to be pointing to '%'
//! The command is added to the class argument storage string in the format
//!	'u' data0 ... data 1
/***************************************************************************/

bool Uniparser::add_arg( uint8_t cmd_id )
{
	//Trace Enter with arguments
	DENTER_ARG("command index: %d\n", cmd_id);

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//index to command
	uint8_t cmd_index;
	//index to argument vector
	uint8_t arg_index;
	//return
	bool f_ret = false;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//If input index is out of range
	if ((UNIPARSER_PENDANTIC_CHECKS) && (cmd_id > this -> g_num_cmd) )
	{
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}
	//Fetch index inside the command
	cmd_index = this -> g_cmd_index[ cmd_id ];
	//if: the command is not an argument
	if ((UNIPARSER_PENDANTIC_CHECKS) && (this -> g_cmd_txt[cmd_id][ cmd_index ] != '%'))
	{
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}
	//Point to the argument type
    cmd_index++;
    //if: the command is not an argument descriptor. PEDANTIC because dictionary should have been checked before hand
	if ((UNIPARSER_PENDANTIC_CHECKS) && (!IS_ARG_DESCRIPTOR(this -> g_cmd_txt[cmd_id][ cmd_index ])) )
	{
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------
	//! @details algorithm:
	//! Compute index to argument vector and number of arguments
	//!	Compute
	//!	ARG Index points to first free slot in the argument vector
	//! I save argument type and initialize the content

		//! Initialize argument identifier
	//fetch argument index
	arg_index = this -> g_arg_fsm_status.arg_index;
	//argument descriptor is held in the dictionary
	uint8_t arg_descriptor = this -> g_cmd_txt[cmd_id][cmd_index];
	//Store argument identifier as first char in the argument
	this -> g_arg[ arg_index ] = arg_descriptor;
	//BUGFIX: default sign is + | fixes an issue when sign is not specified in an argument following a negative argument
	//*-1*2 -> decoded as -1 | -2
	this -> g_arg_fsm_status.arg_sign = false;
	DPRINT("argument of type >%c< added | Num arguments: %d | Arg index: %d | Arg sign %c \n", this -> g_arg[ arg_index ], this -> g_arg_fsm_status.num_arg, this -> g_arg_fsm_status.arg_index, (this -> g_arg_fsm_status.arg_sign)?('-'):('+') );

		//! Initialize argument content
	//switch: decode argument desriptor
	switch (arg_descriptor)
	{
		case Arg_descriptor::ARG_S8:
		{
			//Write zero inside the argument
			f_ret = this -> set_s8( arg_index, (int8_t)0 );
			break;
		}
		case Arg_descriptor::ARG_U8:
		{
			//Write zero inside the argument
			f_ret = this -> set_u8( arg_index, (uint8_t)0 );
			break;
		}
		case Arg_descriptor::ARG_S16:
		{
			//Write zero inside the argument
			f_ret = this -> set_s16( arg_index, (int16_t)0 );
			break;
		}
		case Arg_descriptor::ARG_U16:
		{
			//Write zero inside the argument
			f_ret = this -> set_u16( arg_index, (uint16_t)0 );
			break;
		}
		case Arg_descriptor::ARG_S32:
		{
			//Write zero inside the argument
			f_ret = this -> set_s32( arg_index, (int32_t)0 );
			break;
		}
	}	//end switch: decode argument desriptor

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	//Trace Return vith return value
	DRETURN_ARG("Success: %x\n", f_ret);

	return f_ret; //OK
}	//end method: add_arg | uint8_t

/***************************************************************************/
//!	@brief Private Method
//!	set_s8 | uint8_t, int8_t
/***************************************************************************/
//! @param arg_index | must point to an 'u' argument descriptor
//! @param data | number to be recorded as argument
//! @return false: ok | true: fail
//!	@details
//! Write an S8 inside the argument vector. Index must point to an 's' argument descriptor
/***************************************************************************/

inline bool Uniparser::set_s8( uint8_t arg_index, int8_t data )
{
	DENTER_ARG("arg_index: %d | data: %d\n", arg_index, data );

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//pointer to my argument
	int8_t *s8p;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//if index is out of bound. I need one byte after index for the argument descriptor
	if ((UNIPARSER_PENDANTIC_CHECKS) && ( arg_index >= UNIPARSER_ARG_VECTOR_SIZE-2))
	{
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}

	//If argument descriptor is bad
	if ((UNIPARSER_PENDANTIC_CHECKS) && (this -> g_arg[ arg_index] != Arg_descriptor::ARG_S8))
	{
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Link the pointer to the actual content
	s8p = (int8_t *)&(this -> g_arg[arg_index +1]);
	//Fill the destination with input data
    *s8p = data;

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	DRETURN();
	return false; //OK
}	//end method: set_s8 | uint8_t, int8_t

/***************************************************************************/
//!	@brief Private Method
//!	set_u8 | void
/***************************************************************************/
//! @param arg_index | must point to an 'u' argument descriptor
//! @param data | number to be recorded as argument
//! @return false: ok | true: fail
//!	@details
//! Write an U8 inside the argument vector.
/***************************************************************************/

inline bool Uniparser::set_u8( uint8_t arg_index, uint8_t data )
{
	DENTER_ARG("arg_index: %d | data: %d\n", arg_index, data );

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//pointer to my argument
	uint8_t *u8p;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//if index is out of bound. I need one byte after index for the argument descriptor
	if ((UNIPARSER_PENDANTIC_CHECKS) && ( arg_index >= UNIPARSER_ARG_VECTOR_SIZE-2))
	{
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}

	//If argument descriptor is bad
	if ((UNIPARSER_PENDANTIC_CHECKS) && (this -> g_arg[ arg_index] != Arg_descriptor::ARG_U8))
	{
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Link the pointer to the actual content
	u8p = (uint8_t *)&(this -> g_arg[arg_index +1]);
	//Fill the destination with input data
    *u8p = data;

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	DRETURN();

	return false; //OK
}	//end method: set_u8

/***************************************************************************/
//!	@brief Private Method
//!	set_s16 | uint8_t, int16_t
/***************************************************************************/
//! @param arg_index | must point to an 'u' argument descriptor
//! @param data | number to be recorded as argument
//! @return false: ok | true: fail
//!	@details
//! Write an S8 inside the argument vector. Index must point to an 's' argument descriptor
/***************************************************************************/

inline bool Uniparser::set_s16( uint8_t arg_index, int16_t data )
{
	DENTER_ARG("arg_index: %d | data: %d\n", arg_index, data );

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//pointer to my argument
	int16_t *s16p;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//if index is out of bound. I need one byte after index for the argument descriptor
	if ((UNIPARSER_PENDANTIC_CHECKS) && ( arg_index >= UNIPARSER_ARG_VECTOR_SIZE-2))
	{
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}

	//If argument descriptor is bad
	if ((UNIPARSER_PENDANTIC_CHECKS) && (this -> g_arg[ arg_index] != Arg_descriptor::ARG_S16))
	{
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Link the pointer to the actual content
	s16p = (int16_t *)&(this -> g_arg[arg_index +1]);
	//Fill the destination with input data
    *s16p = data;

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	DRETURN();
	return false; //OK
}	//end method: set_s16 | uint8_t, int16_t

/***************************************************************************/
//!	@brief Private Method
//!	set_u16 | uint8_t, uint16_t
/***************************************************************************/
//! @param arg_index | must point to an argument descriptor
//! @param data | number to be recorded as argument
//! @return false: ok | true: fail
//!	@details
//! Write an S8 inside the argument vector. Index must point to an 's' argument descriptor
/***************************************************************************/

inline bool Uniparser::set_u16( uint8_t arg_index, uint16_t data )
{
	DENTER_ARG("arg_index: %d | data: %d\n", arg_index, data );

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//pointer to my argument
	uint16_t *u16p;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//if index is out of bound. I need one byte after index for the argument descriptor
	if ((UNIPARSER_PENDANTIC_CHECKS) && ( arg_index >= UNIPARSER_ARG_VECTOR_SIZE-2))
	{
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}

	//If argument descriptor is bad
	if ((UNIPARSER_PENDANTIC_CHECKS) && (this -> g_arg[ arg_index] != Arg_descriptor::ARG_U16))
	{
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Link the pointer to the actual content
	u16p = (uint16_t *)&(this -> g_arg[arg_index +1]);
	//Fill the destination with input data
    *u16p = data;

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	DRETURN();
	return false; //OK
}	//end method: set_u16 | uint8_t, int16_t

/***************************************************************************/
//!	@brief Private Method
//!	set_s32 | uint8_t, int32_t
/***************************************************************************/
//! @param arg_index | must point to an 'u' argument descriptor
//! @param data | number to be recorded as argument
//! @return false: ok | true: fail
//!	@details
//! Write an S32 inside the argument vector. Index must point to an 's' argument descriptor
/***************************************************************************/

inline bool Uniparser::set_s32( uint8_t arg_index, int32_t data )
{
	DENTER_ARG("arg_index: %d | data: %d\n", arg_index, data );

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//pointer to my argument
	int32_t *s32p;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//if index is out of bound. I need one byte after index for the argument descriptor
	if ((UNIPARSER_PENDANTIC_CHECKS) && ( arg_index >= UNIPARSER_ARG_VECTOR_SIZE-2))
	{
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}

	//If argument descriptor is bad
	if ((UNIPARSER_PENDANTIC_CHECKS) && (this -> g_arg[ arg_index] != Arg_descriptor::ARG_S32))
	{
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Link the pointer to the actual content
	s32p = (int32_t *)&(this -> g_arg[arg_index +1]);
	//Fill the destination with input data
    *s32p = data;

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	DRETURN();
	return false; //OK
}	//end method: set_s32 | uint8_t, int32_t

/***************************************************************************/
//!	@brief Private Method
//!	get_s8 | uint8_t
/***************************************************************************/
//! @param arg_index | point to an argument descriptor inside the argument vector
//! @return content of argument vector at given position
//!	@details
//! fetch an U8 argument from the argument vector
/***************************************************************************/

inline int8_t Uniparser::get_s8( uint8_t arg_index )
{
	DENTER_ARG("arg_index: %d\n", arg_index );

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//Return value
	int8_t ret;
	//pointer to U8
	int8_t *s8p;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//if index is out of bound. I need one byte after index for the argument descriptor
	if ((UNIPARSER_PENDANTIC_CHECKS) && ( arg_index >= UNIPARSER_ARG_VECTOR_SIZE-2))
	{
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return (int8_t)0xff;	//fail
	}
	//If argument descriptor is bad
	if ((UNIPARSER_PENDANTIC_CHECKS) && (this -> g_arg[ arg_index ] != Arg_descriptor::ARG_S8))
	{
		DPRINT("ERR: Expected S8 argument descriptor >0x%x< | Found instead: >0x%x< in index: %d\n", Arg_descriptor::ARG_S8, this -> g_arg[ arg_index ], arg_index);
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return (int8_t)0xff;	//fail
	}

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Link the pointer to the actual content
	s8p = (int8_t *)&(this -> g_arg[arg_index +1]);
	//Fill the destination with input data
    ret = *s8p;

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	DRETURN_ARG("Decoded S8: %d\n", ret);
	return ret; //OK
}	//end method: get_s8 | uint8_t

/***************************************************************************/
//!	@brief Public Method
//!	get_u8 | uint8_t
/***************************************************************************/
//! @param arg_index | point to an argument descriptor inside the argument vector
//! @return no return
//!	@details
//! fetch an U8 argument from the argument vector
/***************************************************************************/

inline uint8_t Uniparser::get_u8( uint8_t arg_index )
{
	DENTER_ARG("arg_index: %d\n", arg_index );

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//Return value
	uint8_t ret;
	//pointer to U8
	uint8_t *u8p;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//if index is out of bound. I need one byte after index for the argument descriptor
	if ((UNIPARSER_PENDANTIC_CHECKS) && ( arg_index >= UNIPARSER_ARG_VECTOR_SIZE-2))
	{
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}
	//If argument descriptor is bad
	if ((UNIPARSER_PENDANTIC_CHECKS) && (this -> g_arg[ arg_index ] != Arg_descriptor::ARG_U8))
	{
		DPRINT("ERR: Expected U8 argument descriptor >0x%x< | Found instead: >0x%x< in index: %d\n", Arg_descriptor::ARG_U8, this -> g_arg[ arg_index ], arg_index);
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Link the pointer to the actual content
	u8p = (uint8_t *)&(this -> g_arg[arg_index +1]);
	//Fill the destination with input data
    ret = *u8p;

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	DRETURN_ARG("Decoded U8: %d\n", ret);
	return ret; //OK
}	//end method: get_u8 | uint8_t

/***************************************************************************/
//!	@brief Private Method
//!	get_s16 | uint8_t
/***************************************************************************/
//! @param arg_index | point to an argument descriptor inside the argument vector
//! @return content of argument vector at given position
//!	@details
//! fetch an U8 argument from the argument vector
/***************************************************************************/

inline int16_t Uniparser::get_s16( uint8_t arg_index )
{
	DENTER_ARG("arg_index: %d\n", arg_index );

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//Return value
	int16_t ret;
	//pointer to U8
	int16_t *s16p;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//if index is out of bound. I need one byte after index for the argument descriptor
	if ((UNIPARSER_PENDANTIC_CHECKS) && ( arg_index >= UNIPARSER_ARG_VECTOR_SIZE-2))
	{
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return (int16_t)0xffff;	//fail
	}
	//If argument descriptor is bad
	if ((UNIPARSER_PENDANTIC_CHECKS) && (this -> g_arg[ arg_index ] != Arg_descriptor::ARG_S16))
	{
		DPRINT("ERR: Expected S16 argument descriptor >0x%x< | Found instead: >0x%x< in index: %d\n", Arg_descriptor::ARG_S16, this -> g_arg[ arg_index ], arg_index);
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return (int16_t)0xffff;	//fail
	}

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Link the pointer to the actual content
	s16p = (int16_t *)&(this -> g_arg[arg_index +1]);
	//Fill the destination with input data
    ret = *s16p;

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	DRETURN_ARG("Decoded S16: %d\n", ret);
	return ret; //OK
}	//end method: get_s16 | uint8_t

/***************************************************************************/
//!	@brief Private Method
//!	get_u16 | uint8_t
/***************************************************************************/
//! @param arg_index | point to an argument descriptor inside the argument vector
//! @return content of argument vector at given position
//!	@details
//! fetch an U8 argument from the argument vector
/***************************************************************************/

uint16_t Uniparser::get_u16( uint8_t arg_index )
{
	DENTER_ARG("arg_index: %d\n", arg_index );

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//Return value
	uint16_t ret;
	//pointer to U8
	uint16_t *u16p;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//if index is out of bound. I need one byte after index for the argument descriptor
	if ((UNIPARSER_PENDANTIC_CHECKS) && ( arg_index >= UNIPARSER_ARG_VECTOR_SIZE-2))
	{
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return (uint16_t)0xffff;	//fail
	}
	//If argument descriptor is bad
	if ((UNIPARSER_PENDANTIC_CHECKS) && (this -> g_arg[ arg_index ] != Arg_descriptor::ARG_U16))
	{
		DPRINT("ERR: Expected U16 argument descriptor >0x%x< | Found instead: >0x%x< in index: %d\n", Arg_descriptor::ARG_S16, this -> g_arg[ arg_index ], arg_index);
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return (uint16_t)0xffff;	//fail
	}

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Link the pointer to the actual content
	u16p = (uint16_t *)&(this -> g_arg[arg_index +1]);
	//Fill the destination with input data
    ret = *u16p;

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	DRETURN_ARG("Decoded U16: %d\n", ret);
	return ret; //OK
}	//end method: get_u16 | uint8_t

/***************************************************************************/
//!	@brief Private Method
//!	get_s32 | uint8_t
/***************************************************************************/
//! @param arg_index | point to an argument descriptor inside the argument vector
//! @return content of argument vector at given position
//!	@details
//! fetch an U8 argument from the argument vector
/***************************************************************************/

int32_t Uniparser::get_s32( uint8_t arg_index )
{
	DENTER_ARG("arg_index: %d\n", arg_index );

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//Return value
	int32_t ret;
	//pointer to U8
	int32_t *s32p;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//if index is out of bound. I need one byte after index for the argument descriptor
	if ((UNIPARSER_PENDANTIC_CHECKS) && ( arg_index >= UNIPARSER_ARG_VECTOR_SIZE-2))
	{
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return (int16_t)0xffff;	//fail
	}
	//If argument descriptor is bad
	if ((UNIPARSER_PENDANTIC_CHECKS) && (this -> g_arg[ arg_index ] != Arg_descriptor::ARG_S32))
	{
		DPRINT("ERR: Expected S32 argument descriptor >0x%x< | Found instead: >0x%x< in index: %d\n", Arg_descriptor::ARG_S32, this -> g_arg[ arg_index ], arg_index);
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return (int16_t)0xffff;	//fail
	}

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Link the pointer to the actual content
	s32p = (int32_t *)&(this -> g_arg[arg_index +1]);
	//Fill the destination with input data
    ret = *s32p;

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	DRETURN_ARG("Decoded S32: %d\n", ret);
	return ret; //OK
}	//end method: get_s16 | uint8_t

/***************************************************************************/
//!	@brief Private Method
//!	accumulate_arg | uint8_t
/***************************************************************************/
//! @param data | input character to be decoded. It should be a number or a sign
//! @return false: ok | true: fail
//!	@details
//! decode an input character and accumulate it into the argument vector
/***************************************************************************/

bool Uniparser::accumulate_arg( uint8_t data )
{
	//Trace Enter with arguments
	DENTER_ARG("data >%c<\n", data);

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//detect failure state
	bool f_ret;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//if index is out of bound. I need one byte after index for the argument descriptor
	if ((UNIPARSER_PENDANTIC_CHECKS) && ( this -> g_arg_fsm_status.arg_index >= UNIPARSER_ARG_VECTOR_SIZE-2))
	{
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}
	//If argument descriptor is bad
	if ((UNIPARSER_PENDANTIC_CHECKS) && (!IS_ARG_DESCRIPTOR(this -> g_arg[ this -> g_arg_fsm_status.arg_index ])))
	{
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}
	//If: input char is bad
	if ((UNIPARSER_PENDANTIC_CHECKS) && (!IS_SIGN(data)) && (!IS_NUMBER(data)))
	{
		DPRINT("ERR: bad input char. Expecting number or sign and got >0x%x< instead\n", data);
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

		//!Detect sign
	//if sign
	if (IS_SIGN(data))
	{
		//If sign is minus, argument sign is true which means minus
		this -> g_arg_fsm_status.arg_sign = (data == '-');
		DRETURN();
		//I'm done updating argument for now. Next round will come actual numbers.
		return false;
	}
		//!Decode numeric value
	//Fetch argument index
	uint8_t arg_index = this -> g_arg_fsm_status.arg_index;
	//decode the argument descriptor
	switch (this -> g_arg[ arg_index ])
	{
		//if: the command expects a uint8_t data
		case (Arg_descriptor::ARG_S8):
		{
			//Fetch old argument
			int8_t old = this -> get_s8( arg_index );
			//Shift by one digit left
			old *= 10;
			//If number is positive
			if (this -> g_arg_fsm_status.arg_sign == false)
			{
				//Accumulate new digit
				old += data -'0';
			}
			//if: number is negative
			else
			{

				//Accumulate new digit
				old -= data -'0';
			}
			//Write back argument inside argument vector
			f_ret = this -> set_s8( arg_index, old );
			//If set arg failed
			if (f_ret == true)
			{
				return true;
			}
			DPRINT("Updated argument with value: %d | index: %d\n", old, arg_index);
			break;
		}
		//if: the command expects a uint8_t data
		case (Arg_descriptor::ARG_U8):
		{
			//Fetch old argument
			uint8_t old = this -> get_u8( arg_index );
			//Shift by one digit left
			old *= 10;
			//If number is positive
			if (this -> g_arg_fsm_status.arg_sign == false)
			{
				//Accumulate new digit
				old += data -'0';
			}
			//if: number is negative
			else
			{

				//Accumulate new digit
				old -= data -'0';
			}
			//Write back argument inside argument vector
			f_ret = this -> set_u8( arg_index, old );
			//If set arg failed
			if (f_ret == true)
			{
				return true;
			}
			DPRINT("Updated argument with value: %d | index: %d\n", old, arg_index);
			break;
		}
		//if: the command expects a uint8_t data
		case (Arg_descriptor::ARG_S16):
		{
			//Fetch old argument
			int16_t old = this -> get_s16( arg_index );
			//Shift by one digit left
			old *= 10;
			//If number is positive
			if (this -> g_arg_fsm_status.arg_sign == false)
			{
				//Accumulate new digit
				old += data -'0';
			}
			//if: number is negative
			else
			{

				//Accumulate new digit
				old -= data -'0';
			}
			//Write back argument inside argument vector
			f_ret = this -> set_s16( arg_index, old );
			//If set arg failed
			if (f_ret == true)
			{
				return true;
			}
			DPRINT("Updated argument with value: %d | index: %d\n", old, arg_index);
			break;
		}
		//if: the command expects a uint8_t data
		case (Arg_descriptor::ARG_U16):
		{
			//Fetch old argument
			uint16_t old = this -> get_u16( arg_index );
			//Shift by one digit left
			old *= 10;
			//If number is positive
			if (this -> g_arg_fsm_status.arg_sign == false)
			{
				//Accumulate new digit
				old += data -'0';
			}
			//if: number is negative
			else
			{

				//Accumulate new digit
				old -= data -'0';
			}
			//Write back argument inside argument vector
			f_ret = this -> set_u16( arg_index, old );
			//If set arg failed
			if (f_ret == true)
			{
				return true;
			}
			DPRINT("Updated argument with value: %d | index: %d\n", old, arg_index);
			break;
		}
		//if: the command expects a uint8_t data
		case (Arg_descriptor::ARG_S32):
		{
			//Fetch old argument
			int32_t old = this -> get_s32( arg_index );
			//Shift by one digit left
			old *= 10;
			//If number is positive
			if (this -> g_arg_fsm_status.arg_sign == false)
			{
				//Accumulate new digit
				old += data -'0';
			}
			//if: number is negative
			else
			{

				//Accumulate new digit
				old -= data -'0';
			}
			//Write back argument inside argument vector
			f_ret = this -> set_s32( arg_index, old );
			//If set arg failed
			if (f_ret == true)
			{
				return true;
			}
			DPRINT("Updated argument with value: %d | index: %d\n", old, arg_index);
			break;
		}
		//Unrecognized argument descriptor in the dictionary
		default:
		{
			this -> g_err = Err_codes::ERR_GENERIC;
			DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
			return true;	//fail
		}
	}

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	//Trace Return vith return value
	DRETURN();

	return false; //OK
}	//end method: accumulate_arg | uint8_t

/***************************************************************************/
//!	@brief Private Method
//!	close_arg | void
/***************************************************************************/
//! @return false: OK | true: fail
//!	@details
//! Argument has been fully decoded into argument string. Update argument descriptor FSM.
/***************************************************************************/

bool Uniparser::close_arg( void )
{
	//Trace Enter with arguments
	DENTER();

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Fetch arg index inside tha argument vector
	uint8_t arg_index = this -> g_arg_fsm_status.arg_index;
	//if index is out of bound. I need one byte after index for the argument descriptor
	if ((UNIPARSER_PENDANTIC_CHECKS) && ( arg_index >= UNIPARSER_ARG_VECTOR_SIZE-2))
	{
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}
	//Fetch argument descriptor
	uint8_t arg_descriptor = this -> g_arg[ this -> g_arg_fsm_status.arg_index ];
	//If argument descriptor is bad
	if ((UNIPARSER_PENDANTIC_CHECKS) && (!IS_ARG_DESCRIPTOR(arg_descriptor)))
	{
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

		//! Advance the argument index to the first free byte in the argument vector
	//If: one byte argument
	if ((arg_descriptor == Arg_descriptor::ARG_U8) || (arg_descriptor == Arg_descriptor::ARG_S8))
	{
		//Skip the argument descriptor and the argument itself
		arg_index += Arg_size::ARG_DESCRIPTOR_SIZE +Arg_size::ARG_S8_SIZE;
	}
	//If: two bytes argument
	else if ((arg_descriptor == Arg_descriptor::ARG_U16) || (arg_descriptor == Arg_descriptor::ARG_S16))
	{
		//Skip the argument descriptor and the argument itself
		arg_index += Arg_size::ARG_DESCRIPTOR_SIZE +Arg_size::ARG_S16_SIZE;
	}
	//If: four bytes argument
	else if (arg_descriptor == Arg_descriptor::ARG_S32)
	{
		//Skip the argument descriptor and the argument itself
		arg_index += Arg_size::ARG_DESCRIPTOR_SIZE +Arg_size::ARG_S32_SIZE;
	}
	//pedantic because the ditionary should have been cecked before hand for bad argument descriptors
	else if (UNIPARSER_PENDANTIC_CHECKS)
	{
		//This error means the argument descriptor was unrecognized
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;
	}
	else
	{
		//failure
		return true;
	}
		//! Check that index is valid
	//if: index is above bit width or exceed the argument vector size
	if ((arg_index > UNIPARSER_MAX_ARG_INDEX) || (arg_index >= UNIPARSER_ARG_VECTOR_SIZE))
	{
		//Restart the argument decoder
		this -> init_arg_decoder();
		//
		DPRINT("ERR: Exceeded alloted argument vector size with index: %d\n", arg_index);
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}
	//Write back index
	this -> g_arg_fsm_status.arg_index = arg_index;
	//Update number of decoded arguments
	this -> g_arg_fsm_status.num_arg++;
	DPRINT("Argument closed | num arg: %d | arg index: %d\n", this -> g_arg_fsm_status.num_arg, this -> g_arg_fsm_status.arg_index);

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	//Trace Return vith return value
	DRETURN();

	return false; //OK
}	//end method: close_arg | void

/***************************************************************************/
//!	@brief Private Method
//!	exe_handler | uint8_t
/***************************************************************************/
// @param
//! @return no return
//!	@details
//! Execute the handler of function of index cmd_id. Arguments are to be axtracted from the argument vector.
/***************************************************************************/

bool Uniparser::exe_handler( uint8_t exe_index )
{
	//Trace Enter with arguments
	DENTER_ARG("exe_index: %d\n", exe_index);

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//if execution index is out of range.
	if ((UNIPARSER_PENDANTIC_CHECKS) && (exe_index == this -> g_num_cmd))
	{
		DPRINT("ERR: execution index is out of range\n");
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}
	//Fetch number of arguments
	uint8_t num_arg = this -> g_arg_fsm_status.num_arg;
	//if too many arguments have been deduced
	if ((UNIPARSER_PENDANTIC_CHECKS) && (num_arg > UNIPARSER_MAX_ARGS))
	{
		this -> g_err = Err_codes::ERR_GENERIC;
		DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
		return true;	//fail
	}

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------
	//	>no arguments
	//		CALL -> Void
	//	>First argument u
	//		>No second argument
	//			-> CALL u
	//		>Second argument u
	//			-> CALLL u, u
	//		...
	//	>First argument s
	//		>No second argument
	//			-> CALL s
	//		>Second argument s
	//			-> CALLL s, s
	//		...

	//If: no arguments have been deduced
	if (num_arg == 0)
	{
		//! void argument -> void return
		//Create a pointer to function with no arguments and no return
		void (*my_function_ptr)(void);
		//promote the pointer to the right kind
		my_function_ptr = (void(*)(void))this -> g_cmd_handler[exe_index];
		DPRINT("Executing void function with address: >%p<\n", (void *)this -> g_cmd_handler[exe_index]);
		//execute handler
		(*my_function_ptr)();
	}
	//If: at least one argument has been deduced
	else
	{
		//counter
		uint8_t t, index;
		//fetch argument descriptor. Command is limited to having arguments of same type
		uint8_t arg_descriptor = this -> g_arg[0];
		//switch: decode the argument descriptor
		switch(arg_descriptor)
		{
			//case: ARG_S8
			case (Arg_descriptor::ARG_S8):
			{
				//argument vector
				int8_t arg[ UNIPARSER_MAX_ARGS ];

					//! Fetch all arguments
				//initialize argument index
				index = 0;
				//for all arguments
				for (t = 0;t < num_arg;t++)
				{
					//if too many arguments have been deduced
					if ((UNIPARSER_PENDANTIC_CHECKS) && (this -> g_arg[index] != Arg_descriptor::ARG_S8))
					{
						DPRINT("ERR: argument is not >0x%x<. it's instead: >0x%x<", Arg_descriptor::ARG_S8, this -> g_arg[index]);
						this -> g_err = Err_codes::ERR_GENERIC;
						DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
						return true;	//fail
					}
					//get first argument from argument vector
					arg[t] = get_s8( index );
					//advance index
					index += Arg_size::ARG_DESCRIPTOR_SIZE +Arg_size::ARG_S8_SIZE;
					//if too many arguments have been deduced
					if ((UNIPARSER_PENDANTIC_CHECKS) && ((index >= UNIPARSER_ARG_VECTOR_SIZE) || (index > UNIPARSER_MAX_ARG_INDEX)))
					{
						DPRINT("ERR: index is out of range: %d\n", index);
						this -> g_err = Err_codes::ERR_GENERIC;
						DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
						return true;	//fail
					}
				}

					//! Call appropriate function
				//switch number of arguments
				switch( num_arg )
				{
					case 1:
					{
						//Create a pointer to function with no arguments and no return
						void (*my_function_ptr)(int8_t);
						//promote the pointer to the right kind
						my_function_ptr = (void(*)(int8_t))this -> g_cmd_handler[exe_index];
						//execute handler
						(*my_function_ptr)( arg[0]);
						DPRINT("Executing function >%p< with arguments: %d\n", (void *)this -> g_cmd_handler[exe_index], arg[0]);
						break;
					}
					case 2:
					{
						//Create a pointer to function with no arguments and no return
						void (*my_function_ptr)(int8_t, int8_t);
						//promote the pointer to the right kind
						my_function_ptr = (void(*)(int8_t, int8_t))this -> g_cmd_handler[exe_index];
						//execute handler
						(*my_function_ptr)( arg[0], arg[1] );
						DPRINT("Executing function >%p< with arguments: %d %d\n", (void *)this -> g_cmd_handler[exe_index], arg[0], arg[1]);
						break;
					}
					case 3:
					{
						//Create a pointer to function with no arguments and no return
						void (*my_function_ptr)(int8_t, int8_t, int8_t);
						//promote the pointer to the right kind
						my_function_ptr = (void(*)(int8_t, int8_t, int8_t))this -> g_cmd_handler[exe_index];
						//execute handler
						(*my_function_ptr)( arg[0], arg[1], arg[2] );
						DPRINT("Executing function >%p< with arguments: %d %d %d\n", (void *)this -> g_cmd_handler[exe_index], arg[0], arg[1], arg[2]);
						break;
					}
					case 4:
					{
						//Create a pointer to function with no arguments and no return
						void (*my_function_ptr)(int8_t, int8_t, int8_t, int8_t);
						//promote the pointer to the right kind
						my_function_ptr = (void(*)(int8_t, int8_t, int8_t, int8_t))this -> g_cmd_handler[exe_index];
						//execute handler
						(*my_function_ptr)( arg[0], arg[1], arg[2], arg[3] );
						DPRINT("Executing function >%p< with arguments: %d %d %d %d\n", (void *)this -> g_cmd_handler[exe_index], arg[0], arg[1], arg[2], arg[3]);
						break;
					}
					default:
					{
						this -> g_err = Err_codes::ERR_GENERIC;
						DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
						return true;	//fail
					}
				}	//end switch number of arguments

				break;
			} //end case: ARG_S8
			//case: ARG_U8
			case (Arg_descriptor::ARG_U8):
			{
				//argument vector
				uint8_t arg[ UNIPARSER_MAX_ARGS ];

					//! Fetch all arguments
				//initialize argument index
				index = 0;
				//for all arguments
				for (t = 0;t < num_arg;t++)
				{
					//if too many arguments have been deduced
					if ((UNIPARSER_PENDANTIC_CHECKS) && (this -> g_arg[index] != Arg_descriptor::ARG_U8))
					{
						DPRINT("ERR: argument is not >0x%x<. it's instead: >0x%x<", Arg_descriptor::ARG_U8, this -> g_arg[index]);
						this -> g_err = Err_codes::ERR_GENERIC;
						DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
						return true;	//fail
					}
					//get first argument from argument vector
					arg[t] = get_u8( index );
					//advance index
					index += Arg_size::ARG_DESCRIPTOR_SIZE +Arg_size::ARG_U8_SIZE;
					//if too many arguments have been deduced
					if ((UNIPARSER_PENDANTIC_CHECKS) && ((index >= UNIPARSER_ARG_VECTOR_SIZE) || (index > UNIPARSER_MAX_ARG_INDEX)))
					{
						DPRINT("ERR: index is out of range: %d\n", index);
						this -> g_err = Err_codes::ERR_GENERIC;
						DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
						return true;	//fail
					}
				}

					//! Call appropriate function
				//switch number of arguments
				switch( num_arg )
				{
					case 1:
					{
						//Create a pointer to function with no arguments and no return
						void (*my_function_ptr)(uint8_t);
						//promote the pointer to the right kind
						my_function_ptr = (void(*)(uint8_t))this -> g_cmd_handler[exe_index];
						//execute handler
						(*my_function_ptr)( arg[0]);
						DPRINT("Executing function >%p< with arguments: %d\n", (void *)this -> g_cmd_handler[exe_index], arg[0]);
						break;
					}
					case 2:
					{
						//Create a pointer to function with no arguments and no return
						void (*my_function_ptr)(uint8_t, uint8_t);
						//promote the pointer to the right kind
						my_function_ptr = (void(*)(uint8_t, uint8_t))this -> g_cmd_handler[exe_index];
						//execute handler
						(*my_function_ptr)( arg[0], arg[1] );
						DPRINT("Executing function >%p< with arguments: %d %d\n", (void *)this -> g_cmd_handler[exe_index], arg[0], arg[1]);
						break;
					}
					case 3:
					{
						//Create a pointer to function with no arguments and no return
						void (*my_function_ptr)(uint8_t, uint8_t, uint8_t);
						//promote the pointer to the right kind
						my_function_ptr = (void(*)(uint8_t, uint8_t, uint8_t))this -> g_cmd_handler[exe_index];
						//execute handler
						(*my_function_ptr)( arg[0], arg[1], arg[2] );
						DPRINT("Executing function >%p< with arguments: %d %d %d\n", (void *)this -> g_cmd_handler[exe_index], arg[0], arg[1], arg[2]);
						break;
					}
					case 4:
					{
						//Create a pointer to function with no arguments and no return
						void (*my_function_ptr)(uint8_t, uint8_t, uint8_t, uint8_t);
						//promote the pointer to the right kind
						my_function_ptr = (void(*)(uint8_t, uint8_t, uint8_t, uint8_t))this -> g_cmd_handler[exe_index];
						//execute handler
						(*my_function_ptr)( arg[0], arg[1], arg[2], arg[3] );
						DPRINT("Executing function >%p< with arguments: %d %d %d %d\n", (void *)this -> g_cmd_handler[exe_index], arg[0], arg[1], arg[2], arg[3]);
						break;
					}
					default:
					{
						this -> g_err = Err_codes::ERR_GENERIC;
						DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
						return true;	//fail
					}
				}	//end switch number of arguments

				break;
			} //end case: ARG_U8
			//case: ARG_S16
			case (Arg_descriptor::ARG_S16):
			{
				//argument vector
				int16_t arg[ UNIPARSER_MAX_ARGS ];

					//! Fetch all arguments
				//initialize argument index
				index = 0;
				//for all arguments
				for (t = 0;t < num_arg;t++)
				{
					//if too many arguments have been deduced
					if ((UNIPARSER_PENDANTIC_CHECKS) && (this -> g_arg[index] != Arg_descriptor::ARG_S16))
					{
						DPRINT("ERR: argument is not >0x%x<. it's instead: >0x%x<\n", Arg_descriptor::ARG_S16, this -> g_arg[index]);
						this -> g_err = Err_codes::ERR_GENERIC;
						DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
						return true;	//fail
					}
					//get first argument from argument vector
					arg[t] = get_s16( index );
					//advance index
					index += Arg_size::ARG_DESCRIPTOR_SIZE +Arg_size::ARG_S16_SIZE;
					//if too many arguments have been deduced
					if ((UNIPARSER_PENDANTIC_CHECKS) && ((index >= UNIPARSER_ARG_VECTOR_SIZE) || (index > UNIPARSER_MAX_ARG_INDEX)))
					{
						DPRINT("ERR: index is out of range: %d\n", index);
						this -> g_err = Err_codes::ERR_GENERIC;
						DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
						return true;	//fail
					}
				}

					//! Call appropriate function
				//switch number of arguments
				switch( num_arg )
				{
					case 1:
					{
						//Create a pointer to function with no arguments and no return
						void (*my_function_ptr)(int16_t);
						//promote the pointer to the right kind
						my_function_ptr = (void(*)(int16_t))this -> g_cmd_handler[exe_index];
						//execute handler
						(*my_function_ptr)( arg[0]);
						DPRINT("Executing function >%p< with arguments: %d\n", (void *)this -> g_cmd_handler[exe_index], arg[0]);
						break;
					}
					case 2:
					{
						//Create a pointer to function with no arguments and no return
						void (*my_function_ptr)(int16_t, int16_t);
						//promote the pointer to the right kind
						my_function_ptr = (void(*)(int16_t, int16_t))this -> g_cmd_handler[exe_index];
						//execute handler
						(*my_function_ptr)( arg[0], arg[1] );
						DPRINT("Executing function >%p< with arguments: %d %d\n", (void *)this -> g_cmd_handler[exe_index], arg[0], arg[1]);
						break;
					}
					case 3:
					{
						//Create a pointer to function with no arguments and no return
						void (*my_function_ptr)(int16_t, int16_t, int16_t);
						//promote the pointer to the right kind
						my_function_ptr = (void(*)(int16_t, int16_t, int16_t))this -> g_cmd_handler[exe_index];
						//execute handler
						(*my_function_ptr)( arg[0], arg[1], arg[2] );
						DPRINT("Executing function >%p< with arguments: %d %d %d\n", (void *)this -> g_cmd_handler[exe_index], arg[0], arg[1], arg[2]);
						break;
					}
					case 4:
					{
						//Create a pointer to function with no arguments and no return
						void (*my_function_ptr)(int16_t, int16_t, int16_t, int16_t);
						//promote the pointer to the right kind
						my_function_ptr = (void(*)(int16_t, int16_t, int16_t, int16_t))this -> g_cmd_handler[exe_index];
						//execute handler
						(*my_function_ptr)( arg[0], arg[1], arg[2], arg[3] );
						DPRINT("Executing function >%p< with arguments: %d %d %d %d\n", (void *)this -> g_cmd_handler[exe_index], arg[0], arg[1], arg[2], arg[3]);
						break;
					}
					default:
					{
						this -> g_err = Err_codes::ERR_GENERIC;
						DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
						return true;	//fail
					}
				}	//end switch number of arguments

				break;
			} //end case: ARG_S16
			//case: ARG_U16
			case (Arg_descriptor::ARG_U16):
			{
				//argument vector
				uint16_t arg[ UNIPARSER_MAX_ARGS ];

					//! Fetch all arguments
				//initialize argument index
				index = 0;
				//for all arguments
				for (t = 0;t < num_arg;t++)
				{
					//if too many arguments have been deduced
					if ((UNIPARSER_PENDANTIC_CHECKS) && (this -> g_arg[index] != Arg_descriptor::ARG_U16))
					{
						DPRINT("ERR: argument is not >0x%x<. it's instead: >0x%x<\n", Arg_descriptor::ARG_U16, this -> g_arg[index]);
						this -> g_err = Err_codes::ERR_GENERIC;
						DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
						return true;	//fail
					}
					//get first argument from argument vector
					arg[t] = get_u16( index );
					//advance index
					index += Arg_size::ARG_DESCRIPTOR_SIZE +Arg_size::ARG_U16_SIZE;
					//if too many arguments have been deduced
					if ((UNIPARSER_PENDANTIC_CHECKS) && ((index >= UNIPARSER_ARG_VECTOR_SIZE) || (index > UNIPARSER_MAX_ARG_INDEX)))
					{
						DPRINT("ERR: index is out of range: %d\n", index);
						this -> g_err = Err_codes::ERR_GENERIC;
						DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
						return true;	//fail
					}
				}

					//! Call appropriate function
				//switch number of arguments
				switch( num_arg )
				{
					case 1:
					{
						//Create a pointer to function with no arguments and no return
						void (*my_function_ptr)(uint16_t);
						//promote the pointer to the right kind
						my_function_ptr = (void(*)(uint16_t))this -> g_cmd_handler[exe_index];
						//execute handler
						(*my_function_ptr)( arg[0]);
						DPRINT("Executing function >%p< with arguments: %d\n", (void *)this -> g_cmd_handler[exe_index], arg[0]);
						break;
					}
					case 2:
					{
						//Create a pointer to function with no arguments and no return
						void (*my_function_ptr)(uint16_t, uint16_t);
						//promote the pointer to the right kind
						my_function_ptr = (void(*)(uint16_t, uint16_t))this -> g_cmd_handler[exe_index];
						//execute handler
						(*my_function_ptr)( arg[0], arg[1] );
						DPRINT("Executing function >%p< with arguments: %d %d\n", (void *)this -> g_cmd_handler[exe_index], arg[0], arg[1]);
						break;
					}
					case 3:
					{
						//Create a pointer to function with no arguments and no return
						void (*my_function_ptr)(uint16_t, uint16_t, uint16_t);
						//promote the pointer to the right kind
						my_function_ptr = (void(*)(uint16_t, uint16_t, uint16_t))this -> g_cmd_handler[exe_index];
						//execute handler
						(*my_function_ptr)( arg[0], arg[1], arg[2] );
						DPRINT("Executing function >%p< with arguments: %d %d %d\n", (void *)this -> g_cmd_handler[exe_index], arg[0], arg[1], arg[2]);
						break;
					}
					case 4:
					{
						//Create a pointer to function with no arguments and no return
						void (*my_function_ptr)(uint16_t, uint16_t, uint16_t, uint16_t);
						//promote the pointer to the right kind
						my_function_ptr = (void(*)(uint16_t, uint16_t, uint16_t, uint16_t))this -> g_cmd_handler[exe_index];
						//execute handler
						(*my_function_ptr)( arg[0], arg[1], arg[2], arg[3] );
						DPRINT("Executing function >%p< with arguments: %d %d %d %d\n", (void *)this -> g_cmd_handler[exe_index], arg[0], arg[1], arg[2], arg[3]);
						break;
					}
					default:
					{
						this -> g_err = Err_codes::ERR_GENERIC;
						DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
						return true;	//fail
					}
				}	//end switch number of arguments

				break;
			} //end case: ARG_U16
			//case: ARG_S32
			case (Arg_descriptor::ARG_S32):
			{
				//argument vector
				int32_t arg[ UNIPARSER_MAX_ARGS ];

					//! Fetch all arguments
				//initialize argument index
				index = 0;
				//for all arguments
				for (t = 0;t < num_arg;t++)
				{
					//if too many arguments have been deduced
					if ((UNIPARSER_PENDANTIC_CHECKS) && (this -> g_arg[index] != Arg_descriptor::ARG_S32))
					{
						DPRINT("ERR: argument is not >0x%x<. it's instead: >0x%x<\n", Arg_descriptor::ARG_S32, this -> g_arg[index]);
						this -> g_err = Err_codes::ERR_GENERIC;
						DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
						return true;	//fail
					}
					//get first argument from argument vector
					arg[t] = get_s32( index );
					//advance index
					index += Arg_size::ARG_DESCRIPTOR_SIZE +Arg_size::ARG_S32_SIZE;
					//if too many arguments have been deduced
					if ((UNIPARSER_PENDANTIC_CHECKS) && ((index >= UNIPARSER_ARG_VECTOR_SIZE) || (index > UNIPARSER_MAX_ARG_INDEX)))
					{
						DPRINT("ERR: index is out of range: %d\n", index);
						this -> g_err = Err_codes::ERR_GENERIC;
						DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
						return true;	//fail
					}
				}

					//! Call appropriate function
				//switch number of arguments
				switch( num_arg )
				{
					case 1:
					{
						//Create a pointer to function with no arguments and no return
						void (*my_function_ptr)(int32_t);
						//promote the pointer to the right kind
						my_function_ptr = (void(*)(int32_t))this -> g_cmd_handler[exe_index];
						//execute handler
						(*my_function_ptr)( arg[0] );
						DPRINT("Executing function >%p< with arguments: %d\n", (void *)this -> g_cmd_handler[exe_index], arg[0]);
						break;
					}
					case 2:
					{
						//Create a pointer to function with no arguments and no return
						void (*my_function_ptr)(int32_t, int32_t);
						//promote the pointer to the right kind
						my_function_ptr = (void(*)(int32_t, int32_t))this -> g_cmd_handler[exe_index];
						//execute handler
						(*my_function_ptr)( arg[0], arg[1] );
						DPRINT("Executing function >%p< with arguments: %d %d\n", (void *)this -> g_cmd_handler[exe_index], arg[0], arg[1]);
						break;
					}
					default:
					{
						this -> g_err = Err_codes::ERR_GENERIC;
						DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
						return true;	//fail
					}
				}	//end switch number of arguments

				break;
			} //end case: ARG_S32
			//Function type not recognized
			default:
			{
				this -> g_err = Err_codes::ERR_GENERIC;
				DRETURN_ARG("ERR%d: ERR_GENERIC in line: %d\n", this -> g_err, __LINE__ );
				return true;	//fail
			}
		}	//end switch: decode the argument descriptor
	}	//end If: at least one argument has been deduced

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	//Trace Return vith return value
	DRETURN();

	return false; //OK
}	//end method: exe_handler | uint8_t

/****************************************************************************
**	NAMESPACES
****************************************************************************/

} //End Namespace
