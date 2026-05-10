/** C++ includes */
#include "utilities.hpp"

/** C includes */
extern "C"
{
    #include <stdint.h>
    #include <stdio.h>
    #include <stdarg.h> 
    #include "main.h"
    #include "usbd_cdc_if.h"
    uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
}

/** main c/c++ implementation */

/** definitions */
#define USB_BUFLEN 128

/** variables */
uint8_t usbTxBuf[USB_BUFLEN];
uint16_t usbTxBufLen;

/** functions */
// logger 
extern "C" 
{
    void logger(const char* tag, const char* format, ...)
    {
        va_list args;
        va_start(args, format);
        char msgBuf[USB_BUFLEN];
        vsnprintf(msgBuf, sizeof(msgBuf), format, args);
        va_end(args);
        usbTxBufLen = snprintf((char*) usbTxBuf, USB_BUFLEN, "[%s]: %s\r\n", tag, msgBuf);
        CDC_Transmit_FS(usbTxBuf, usbTxBufLen);
    }
}

// rgb_set
extern "C"
{
    void rgb_set(uint8_t r, uint8_t g, uint8_t b) 
    {
    HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, r ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, g ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, b ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}