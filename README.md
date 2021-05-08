# **PIC DC MOTOR CONTROLLER**

This project was built in 2019. 

It contains two codes, one for Arduino and another for PIC. The idea is simple, the PIC controller works as a slave receiving params from the Arduino to control a DC Motor. This project aims to turn cheap DC motors into good motors for robotics through effective PID control. 

## ID SPECIFICATIONS
                         
>#### | 0xF0 -> Read Setpoint | 0xE0 -> Write Setpoint |
>#### | 0xF1 -> Period |
>#### | 0xF2 -> Read PWM | 0XE2 -> Write PWM |
>#### | 0xF3 -> Read KP | 0xE3 -> Write KP |
>#### | 0xF4 -> Read KI | 0xE4 -> Write KI |
>#### | 0xF5 -> Read KD | 0xE5 -> Write KD |
>#### | 0xAA -> Set Open Loop 0 | 0xFF -> Set Closed Loop 1 |
>#### | 0xFA -> Read Loop, 0 Open 1 Closed |

### You can see better details on the article and on the presentation (in Portuguese).