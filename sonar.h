#ifndef SONAR_H_
#define SONAR_H_

#define UNIT_COMMAND 0
#define SEND_CM  50
#define SEND_INC 51
#define SEND_MILLIS 52
//#define SONAR_COMMAND 1
#define SONAR_1 60
#define SONAR_2 61
#define SONAR_3 62
#define SONAR_4 63
#define SONAR_5 64
#define CHANGE_ADD_COMAND 70
#define EEPROM_I2C_ADD 0 
#define EEPROM_ADD_VALIDATE 1
#define EEPROM_DAT_VALIDATE 0xAA
#define MAX_RANGE_ALT 400
#define PULSE_TIMEOUT 22600 // 11600 us for srf04 max range 400 cm 
#define SONAR_LEFT 60
#define SONAR_RIGHT 61
#define SONAR_FRONT 62
#define SONAR_REAR 64

#ifndef OUTPUT_LIMIT_MIN
#define	OUTPUT_LIMIT_MIN 0
#endif OUTPUT_LIMIT_MIN

#ifndef OUTPUT_LIMIT_MAX
#define	OUTPUT_LIMIT_MAX 127
#endif OUTPUT_LIMIT_MAX
//#define debug
#endif
long microsecondsToCentimeters(long microseconds);
long microsecondsToInches(long microseconds);
void set_i2c_add(void);
void receiveEvent(int size);
void requestEvent();
void get_data(void);
