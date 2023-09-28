#ifndef MOSFETS_H_
#define MOSFETS_H_

void disable_mosfets(void);
void enable_mosfets(void);
void set_active_mosfets_index(int8_t index);
uint8_t get_mosfets_enabled(void);
uint8_t get_active_mosfets_index(void);

#endif
