/*
 * enum_cmd.h - Enumeration initialization entry point
 */
#ifndef LAT_ENUM_CMD_H
#define LAT_ENUM_CMD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*EnumLogFn)(const char *line);

void enum_cmd_init(EnumLogFn log);

#ifdef __cplusplus
}
#endif

#endif /* LAT_ENUM_CMD_H */
