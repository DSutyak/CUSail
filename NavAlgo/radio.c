/* ************************************************************************** */
/** Radio Communication to Base Station
/* ************************************************************************** */
#define _SUPPRESS_PLIB_WARNING // removes outdated plib warning
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING

#include <plib.h>

void initRadio(void) {
    // check this for length, parity bits
    OpenUART2(UART_EN | UART_NO_PAR_8BIT, UART_RX_ENABLE, 9600);
    PPSInput(2, U2RX, RPA1);
    PPSOutput(4, RPB10, U2TX);
}