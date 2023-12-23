#ifndef __DEBUG_H__
#define __DEBUG_H__

#include "utils/utils_logger.h"

#ifdef DEBUG
#define DMSG(args...)      	utils_log_raw(args)
#define DMSG_CHAR(character)      	utils_log_raw("%c", character)
#define DMSG_STR(str)       utils_log_raw("%s", str)
#define DMSG_HEX(num)       utils_log_raw(' '); utils_log_raw("%x", num)
#define DMSG_INT(num)       utils_log_raw(' '); utils_log_raw("%d", num)
#else
#define DMSG(args...)
#define DMSG_STR(str)
#define DMSG_HEX(num)
#define DMSG_INT(num)
#endif

#endif
