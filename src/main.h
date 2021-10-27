#ifndef MAIN_H_
#define MAIN_H_

void read_registers(uint8_t, size_t);
uint8_t read_register(uint8_t);

void write_registers(uint8_t, size_t);
void write_register(uint8_t, uint8_t);

void setupSPI();
void setupLIS3DH();

void stop_collecting();

void saveConfigCallback ();

#endif