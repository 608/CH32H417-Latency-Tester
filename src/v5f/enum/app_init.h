/*
 * app_init.h - Interrupt-OUT application-layer initialization (Switch Pro, GIP)
 */
#ifndef LAT_APP_INIT_H
#define LAT_APP_INIT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_SW_SDL_OPEN        ((uint8_t)2u)
#define APP_SW_INPUT_MINIMAL   ((uint8_t)1u)

typedef struct AppTarget
{
    uint16_t vid;
    uint16_t pid;
    uint8_t  in_pipe;
    uint8_t  out_pipe;
    uint16_t out_len;          /* Switch: min(64, wMaxPacketSize) */
} AppTarget;

void app_reset(void);

void app_switch_start(const AppTarget *t, uint8_t mode, uint64_t now);
void app_gip_start(const AppTarget *t, uint64_t now);

void app_on_report(uint8_t pipe, const uint8_t *d, uint16_t len, uint64_t t1);

void app_on_out_done(uint8_t pipe, uint64_t t);

void app_poll(uint64_t now);

/* Returns ENUM_APP_* (enum_api.h). */
uint8_t app_state(void);

/* For unit tests: current Switch Pro step (1..13, 14 = steady state). */
uint8_t app_switch_step(void);

#ifdef __cplusplus
}
#endif

#endif /* LAT_APP_INIT_H */
