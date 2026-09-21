#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include "main.h"
#include <cstdint>

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

class SPI_manager {
public:
    enum class Owner {
        NONE,
        ENCODER,
        MOTOR
    };

    bool set_owner(Owner owner)
    {
        if (owner_ != Owner::NONE)
            return false;

        owner_ = owner;
        return true;
    }

    void release() {owner_ = Owner::NONE;}

    Owner get_owner() const {return owner_;}

    bool busy() const   {return owner_ != Owner::NONE;}

private:
    Owner owner_ = Owner::NONE;
};
#endif /* UTILITIES_HPP */