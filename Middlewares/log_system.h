#ifndef __LOG_SYSTEM_H
#define __LOG_SYSTEM_H

#include "stdint.h"

typedef enum
{
  LOG_LEVEL_DEBUG,
  LOG_LEVEL_INFO,
  LOG_LEVEL_WARNING,
  LOG_LEVEL_ERROR
} Log_Level_t;

void Log_Init(void);
void Log_Message(Log_Level_t level, const char *format, ...);
void Log_ToBuffer(Log_Level_t level, const char *message);

#endif /* __LOG_SYSTEM_H */
