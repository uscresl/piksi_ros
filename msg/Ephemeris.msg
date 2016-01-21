Header header

float64 tgd # Group delay differential between L1 and L2 [s]

float64 c_rs # Amplitude of the sine harmonic correction term to the orbit radius [m]
float64 c_rc # Amplitude of the cosine harmonic correction term to the orbit radius [m]
float64 c_uc # Amplitude of the cosine harmonic correction term to the argument of latitude [rad]
float64 c_us # Amplitude of the sine harmonic correction term to the argument of latitude [rad]
float64 c_ic # Amplitude of the cosine harmonic correction term to the angle of inclination [rad]
float64 c_is # Amplitude of the sine harmonic correction term to the angle of inclination [rad]

float64 dn

float64 m0

float64 ecc
float64 sqrta

float64 omega0
float64 omegadot

float64 w

float64 inc
float64 inc_dot

float64 af0
float64 af1
float64 af2

float64 toe_tow
uint16 toe_wn

float64 toc_tow
uint16 toc_wn

uint8 valid
uint8 healthy

piksi_ros/Sid sid

uint8 iode
uint16 iodc





#
#
#
# Mean motion difference [rad/s]
# m0
# Mean anomaly at reference time [rad]
# ecc
# Eccentricity of satellite orbit
# sqrta
# Square root of the semi-major axis of orbit [m^(1/2)]
# omega0
# Longitude of ascending node of orbit plane at weekly epoch [rad]
# omegadot
# Rate of right ascension [rad/s]
# w
# Argument of perigee []
# inc
# Inclination
# inc dot
# Inclination first derivative
# af0
# Polynomial clock correction coefficient (clock
# bias)
# af1
# Polynomial clock correction coefficient (clock
# drift)
# af2
# Polynomial clock correction coefficient (rate
# of clock drift)
# toe tow
# Time of week
# toe wn
# Week number
# toc tow
# Clock reference time of week
# toc wn
# Clock reference week number
# valid
# Is valid?
# healthy
# Satellite is healthy?
# sid.sat
# Constellation-specific satellite identifier
# sid.band
# Signal band
# sid.constellation
# Constellation to which the satellite belongs
# iode
# Issue of ephemeris data
# iodc
# Issue of clock data
# reserved
# Reserved field
