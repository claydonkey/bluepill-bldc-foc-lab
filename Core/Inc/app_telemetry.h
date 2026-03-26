/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Anthony Campbell (claydonkey)
 */

#ifndef APP_TELEMETRY_H
#define APP_TELEMETRY_H

void AppTelemetry_SendReady(void);
void AppTelemetry_UpdateCache(void);
void AppTelemetry_SendFocSnapshot(void);
void AppTelemetry_SendDiagSnapshot(void);

#endif
