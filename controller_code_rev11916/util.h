#ifndef UTIL
#define UTIL
/* Header file to put miscellaneous operations that are done with the arduino. */
void wait_for_serial_data();
double read_ambient_temperature();
void wait_for_eeprom();
double adc_to_voltage(double adc_value);



#endif
