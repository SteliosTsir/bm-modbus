/*
 * Picks what this image does: the Modbus slave (default) or the Modbus master.
 *   make ROLE=slave     (default)   slave()  in modbus_slave.c
 *   make ROLE=master                master() in modbus_master.c
 * The Makefile turns ROLE=master into -DMODBUS_ROLE_MASTER.
 */
void slave(void);
void master(void);

void main(void)
{
#ifdef MODBUS_ROLE_MASTER
	master();
#else
	//slave();
#endif
}