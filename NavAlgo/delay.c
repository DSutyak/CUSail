//#include "config.h"
//#include "delay.h"
//
//void delay_ms(unsigned int ms) {
//    delay_us(ms * 1000);
//}
//
//void delay_us(unsigned int us) {
//    us *= sys_clock / 1000000 / 2;
//    _CP0_SET_COUNT(0); // Set Core Timer count to 0
//    while (us > _CP0_GET_COUNT());
//}