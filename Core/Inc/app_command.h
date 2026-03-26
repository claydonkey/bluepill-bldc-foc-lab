/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Anthony Campbell (claydonkey)
 */

#ifndef APP_COMMAND_H
#define APP_COMMAND_H

#include <stdint.h>

void AppCommand_Process(const char *cmd_buf, uint32_t len);

#endif
