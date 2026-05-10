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

#endif /* UTILITIES_HPP */