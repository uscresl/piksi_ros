Header header

float64 tgd # Group delay differential between L1 and L2 [s]

float64 c_rs # Amplitude of the sine harmonic correction term to the orbit radius [m]
float64 c_rc # Amplitude of the cosine harmonic correction term to the orbit radius [m]
float64 c_uc # Amplitude of the cosine harmonic correction term to the argument of latitude [rad]
float64 c_us # Amplitude of the sine harmonic correction term to the argument of latitude [rad]
float64 c_ic # Amplitude of the cosine harmonic correction term to the angle of inclination [rad]
float64 c_is # Amplitude of the sine harmonic correction term to the angle of inclination [rad]

float64 dn # Mean motion difference [rad/s]

float64 m0 # Mean anomaly at reference time [rad]

float64 ecc # Eccentricity of satellite orbit
float64 sqrta # Square root of the semi-major axis of orbit [m^(1/2)]

float64 omega0 # Longitude of ascending node of orbit plane at weekly epoch [rad]
float64 omegadot # Rate of right ascension [rad/s]

float64 w # Argument of perigee []

float64 inc # Inclination
float64 inc_dot # Inclination first derivative

float64 af0 # Polynomial clock correction coefficient (clock bias)
float64 af1 # Polynomial clock correction coefficient (clock drift)
float64 af2 # Polynomial clock correction coefficient (rate of clock drift)

float64 toe_tow # Time of week
uint16 toe_wn # Week number

float64 toc_tow # Clock reference time of week
uint16 toc_wn # Clock reference week number

uint8 valid # Is valid?
uint8 healthy # Satellite is healthy?

piksi_ros/Sid sid

uint8 iode # Issue of ephemeris data
uint16 iodc # Issue of clock data
