void wait_for_serial_data()
{
  while(!Serial.available())
  {}
}

void wait_for_eeprom()
{
  while(!EEPROM.isReady())
  { }
}

double adc_to_voltage(double adc_value)
{
  /* This function provides a map from the adc value given to the volatage
   *  value that it is mapped to.
   * This will return a value in V (not mV)
   */
   
   double vrefh = 5; //5 V
   double vrefl = 0; //0 V
   double adc_res = 1024; //2^10 (10 bits)
   double adc_volt = vrefl + (adc_value/adc_res)*(vrefh - vrefl);
   //double adc_volt = map(adc_value, 0, adc_res, vrefl, vrefh);
   return adc_volt;
}

double read_ambient_temperature()
{
  /* This function will read in a value from the ADC and then calculate the actual
   *  value of the temperature read in.
   */
   int temp_adc_value = analogRead(TEMP_SENSOR_PIN);
   double temp_sensor_out_voltage = adc_to_voltage((double)temp_adc_value); //in V
   temp_sensor_out_voltage = temp_sensor_out_voltage * 1000; //mV
   double lower_temp = 0; //Celcius
   double lower_mv = 1034; //mV
   double higher_temp = 32; //Celcius 
   double higher_mv = 860; //mV
   /* The below function is obtained from the LMT84LP data sheet and is the linear approximation of a value when finding
    *  temperatures within a particular range (0-32 C) for the average Ocean temperatures across time of year.
    */
   double approx_temp = (temp_sensor_out_voltage - lower_mv)*((higher_temp - lower_temp)/(higher_mv - lower_mv))+lower_temp;
   return approx_temp;
}
