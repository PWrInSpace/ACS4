/*
 * ACS4 Flight Computer — ChibiOS Shell Configuration
 */

#ifndef SHELLCONF_H
#define SHELLCONF_H

#define SHELL_MAX_LINE_LENGTH    128
#define SHELL_MAX_ARGUMENTS      8
#define SHELL_PROMPT_STR         "acs4> "

#define SHELL_USE_HISTORY        TRUE
#define SHELL_USE_COMPLETION     FALSE
#define SHELL_USE_ESC_SEQ        TRUE

#define SHELL_CMD_EXIT_ENABLED   FALSE
#define SHELL_CMD_INFO_ENABLED   TRUE
#define SHELL_CMD_ECHO_ENABLED   TRUE
#define SHELL_CMD_SYSTIME_ENABLED TRUE
#define SHELL_CMD_TEST_ENABLED   FALSE

#define SHELL_HISTORY_DEPTH      4

#endif /* SHELLCONF_H */
