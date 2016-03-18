# Carrier phase observation here is represented as a 40-bit fixed
# point number with Q32.8 layout (i.e. 32-bits of whole cycles and 8-bits of fractional cycles).

int32 i # Carrier phase whole cycles [cycles]
uint8 f # Carrier phase fractional part [cycles/256]
