#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif
    void logger(const char* tag, const char* format, ...);  
    void rgb_set(uint8_t r, uint8_t g, uint8_t b);
#ifdef __cplusplus
}
#endif

// Modulo (as opposed to remainder), per https://stackoverflow.com/a/19288271
int mod(const int dividend, const int divisor);

#endif /* UTILITIES_HPP */