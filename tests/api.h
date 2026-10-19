/* Thin layer so test_core.c compiles against the current API. */
#ifndef API_H
#define API_H
#include <modbus/modbus_rtu.h>
static inline int mb_rtu_init(modbus_rtu_t *r, uint8_t id, const modbus_backend_t *b)
{ return modbus_rtu_init(r, id, b); }
#endif
