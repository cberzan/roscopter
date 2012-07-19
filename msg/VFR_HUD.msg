# Metrics typically displayed on a HUD for fixed wing aircraft        
#airspeed                  : Current airspeed in m/s (float)
#groundspeed               : Current ground speed in m/s (float)
#heading                   : Current heading in degrees, in compass units (0..360, 0=north) (int16_t)  
#throttle                  : Current throttle setting in integer percent, 0 to 100 (uint16_t)  
#alt                       : Current altitude (MSL), in meters (float)
#climb                     : Current climb rate in meters/second (float)             

float32 airspeed 
float32 groundspeed 
int16 heading
uint16 throttle
float32 alt 
float32 climb
