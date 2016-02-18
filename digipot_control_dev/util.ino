void wait_for_serial_data()
{
  while(!Serial.available())
  {}
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

double adc_to_voltage(double adc_value)
{
  /* This function provides a map from the adc value given to the volatage
   *  value that it is mapped to.
   * This will return a value in V (not mV)
   */
   
   double vrefh = 5; //5 V
   double vrefl = 0; //0 V (GND)
   double adc_res = 1024; //2^10 (10 bits)
   double adc_volt = vrefl + (adc_value/adc_res)*(vrefh - vrefl);
   //double adc_volt = map(adc_value, 0, adc_res, vrefl, vrefh);
   return adc_volt;
}

void wait_for_data_and_continue()
{
      Serial.println(F("\n[INFO] Please enter anything to continue"));
      wait_for_serial_data();
      int number_of_bytes_received = Serial.available();
      char serial_input;
      while(Serial.available() > 0)
      {
        serial_input = Serial.read();
      }
}

void power_on_pump()
{
  /* This function will trigger on the relay controlling the pump */
  digitalWrite(PUMP_RELAY, RELAY_ON);
}

void power_on_everything()
{
  /* This provide a function for turning on all of the relays on teh board
   *  THis is typically used after the u-chamber has been filled with a fresh sample
   */
   digitalWrite(RELAYS, RELAY_ON);
   digitalWrite(PUMP_RELAY, RELAY_ON);
}

double demo_calibrate_frequency()
{
  /* This function will run through and calibrate the frequency with input from a 
   *  Temperature Sensor. The effect of temperature on teh resonance frequency is a known
   *  relationship with some statistical error. Idea is to sweep within 2-3 std. deviations
   *  of the new projected resonance frequency. Will use a stored step size as well.
   */
   double calibrated_frequency = EEPROM.readDouble(CALIBRATED_FREQ_ADDR);
   double temperature = read_ambient_temperature();
   Serial.print(F("[INFO] Temperature Read: "));
   Serial.print(temperature);
   Serial.print(F("\n"));
   double temp_calibrated_frequency = map_to_temperature_frequency(calibrated_frequency, temperature);
   Serial.print("[INFO] New Temperature Mapped Calibrated Frequency: ");
   Serial.print(temp_calibrated_frequency);
   Serial.print(F("\n"));
   double std_dev = EEPROM.readDouble(TEMP_ADJUST_STD_DEV_ADDR);
   double sweep_low_freq = temp_calibrated_frequency - ( 3 * std_dev );
   double sweep_high_freq = temp_calibrated_frequency + (3 * std_dev );
   Serial.print("[INFO] Low Sweep Frequency: ");
   Serial.print(sweep_low_freq);
   Serial.print(F("\n"));
   Serial.print("[INFO] High Sweep Frequency: ");
   Serial.print(sweep_high_freq);
   Serial.print(F("\n"));
   double step_size = EEPROM.readDouble(FREQ_SWEEP_STEP_ADDR);
   wait_for_data_and_continue();
   double adjusted_freq = demo_sweep_frequency(sweep_low_freq, sweep_high_freq, temp_calibrated_frequency, step_size);
   Serial.print("[INFO] Adjusted Calibrated Frequency: ");
   Serial.print(adjusted_freq);
   Serial.print(F("\n"));
   return adjusted_freq;
}

double map_to_temperature_frequency(double orig_freq, double ambient_temp)
{
  /* THis function will serve to map the original frequency of the microchamber
   *  to the predicted resonance freuqency of the microchamber at another temperature
   */
   /* TODO: Find this relationship out and then replace the inside of this function with what it should
    *  be
    */
    return orig_freq;
}

double demo_sweep_frequency(double low_freq,double high_freq,double nom_freq,double step_size)
{
  /* This function will sweep the frequency of the PZTs on the system from the low frequency to higher frequency, 
   *  using the passed in arguments and step size.
   *  The function will then return the value of frequency that gives the maximal PMT output value, this is assumed to be
   *  the adjusted resonance frequency of the u-chamber
   *  INPUTS:
   *    low_freq - frequency to start sweep at
   *    high_freq - frequency to end sweep at
   *    nom_freq - the "actual" frequency computed by temperature model, or a middle frequency
   *    step_size - the size of frequency steps used to find the adjusted resonance frequency
   *  RETURNS:
   *    double representing the adjusted calibration frequency
   */
   double max_freq = -1;
   double current_freq = low_freq;
   double max_adc_value = -1;
   /* intially run the experiment at low frequency */
   digitalWrite(PZT_DRIVER_RELAY, RELAY_ON);
   initialize_fgen();
   while(current_freq < (high_freq+step_size))
   {
      if ((nom_freq < current_freq) && (nom_freq > (current_freq - step_size)))
      {
        current_freq = nom_freq;
      }
      else if (current_freq > high_freq)
      {
        current_freq = high_freq;
      }

      
      adjust_frequency(current_freq);
      Serial.print("[INFO] Current Frequency:");
      Serial.print(current_freq);
      Serial.print(F("\n"));
      /* wait for the PZT's to be able to trap */
      /* Obtained this value from previous research paper on this topic, used plot */
      delay(500);
      int readings[4];
      readings[0] = analogRead(PMT_IN);
      readings[1] = analogRead(PMT_IN);
      readings[2] = analogRead(PMT_IN);
      readings[3] = analogRead(PMT_IN);
      double avg = (readings[0] + readings[1] + readings[2] + readings[3])/4;
      Serial.print(F("[INFO] ADC Value: "));
      Serial.print(avg);
      Serial.print(F("\n"));
      if (avg > max_adc_value)
      {
        max_adc_value = avg;
        max_freq = current_freq;
      }
      current_freq = current_freq + step_size;
      //Serial.println(F("[INFO] Adjust voltage for PMT input if necessary"));
      Serial.flush();
      wait_for_data_and_continue();
      //digitalWrite(PZT_DRIVER_RELAY, RELAY_OFF);
      
   }
   return max_freq;
}

void initialize_fgen()
{ 
  /* This function will initialize the fgen after turning it on */
  pulse_high_signal(RESET);
  pulse_high_signal(W_CLK);
  pulse_high_signal(FQ_UD);
}

void adjust_frequency(double new_frequency)
{
  /* This function will lower or raise the frequency needed in order to 
   *  calibrate the device
   *  INPUTS:
   *    double frequency - frequency to adjust the function generator to
   */
   /* Code based off code on internet, written by Andrew Smallbone at
    *  www.rocketnumbernine.com
    *  
    */
    /* 4294967295 is the frequency tuning word */
   int32_t freq_to_send = new_frequency * 4294967295/125000000; //125 MHz clock on fgen
   for (int b = 0; b<4; b++, freq_to_send >>=8) //unfamiliar with this syntax
   {
      transfer_byte_to_fgen(freq_to_send & 0xFF);
   }
   transfer_byte_to_fgen(0x000); //final control byte, all 0
   pulse_high_signal(FQ_UD); //Done now, frequency changed
}

void pulse_high_signal(int pin)
{
  digitalWrite(pin, HIGH);
  digitalWrite(pin, LOW);
}

void transfer_byte_to_fgen(byte data)
{
  /* This function will transfer a byte of data, one bit at a time
   *  to the Function generator as a means to program it.
   *  INPUT:
   *    byte data-byte of data to transmit
   */
   for (int i = 0; i<8; i++, data>>=1)
   {
      digitalWrite(DATA, data & 0x01);
      pulse_high_signal(W_CLK); //after each bit is sent pulse the clock high
   }
}

double read_in_pmt(double frequency)
{
  /* This function will kick off the FGEn at the correct frequency and will take a 
   *  number of samples and average them.
   */
   double adc_values[NUMBER_OF_MEASUREMENTS];
   //digitalWrite(PZT_DRIVER_RELAY, RELAY_ON);
   initialize_fgen();
   adjust_frequency(frequency);
   delay(500);
   //delay(TRAP_DELAY);
   for(int i = 0; i < NUMBER_OF_MEASUREMENTS; i++)
   {
      adc_values[i] = analogRead(PMT_IN);
   }
   //digitalWrite(PZT_DRIVER_RELAY, RELAY_OFF);
   delay(500);
   
   double avg_value = 0;
   for (int i = 0; i < NUMBER_OF_MEASUREMENTS; i++)
   {
      avg_value = avg_value + ((1.0/NUMBER_OF_MEASUREMENTS) * adc_values[i]);
   }
   return avg_value;
}

void power_off_components()
{
  digitalWrite(PUMP_RELAY, RELAY_OFF);
  digitalWrite(RELAYS, RELAY_OFF);
  digitalWrite(PZT_DRIVER_RELAY, RELAY_OFF);
}

double convert_adc_to_oil(double adc_value)
{
    double adc_voltage = adc_to_voltage(adc_value);
    Serial.print(F("[INFO] PMT Voltage Read (V): "));
    Serial.println(adc_voltage);
    /* TODO: Here is where you would put the Voltage->Measurement value.
     *  For now, just return old value.
     */
    double oil_measure = pmt_volt_to_oil(adc_voltage);
    return oil_measure;
}

double pmt_volt_to_oil(double volt)
{
  /* THis function will convert a voltage to an oil measurement.
   *  Right now this is just a placeholder for some 
   *  experimental map that has yet to be determined
   */

   /* TODO: Replace with actual map*/
   return volt;
}

void transfer_value_to_pot(int reg, int value)
{
  /* This function provides a utility to shift data into the digital potentiometer
   *  
   */
   
   boolean a0, a1;
   boolean data[8];
   a1 = 0;
   if (reg == 1)
   {
    a0 = 0;
   }
   else
   {
    a0 = 1;
   }
   int j = 0;
   for (int i = 128; i >=1; i = i/2) // 7 -bit data space
   {
      if ((value & i) != 0)
      {
        data[j] = 1;
      }
      else
      {
        data[j] = 0;
      }
      ++j;
   }
   //while(1)
   //{
   /* Send Address First */
   int wait_delay = 1; //need 50 ns, so be safe with >=100ns=1us
   //digitalWrite(POT_CSIN, HIGH); //this is active low pin
   digitalWrite(POT_CSIN, LOW);
   delayMicroseconds(wait_delay);
   //Serial.print(F("[DEBUG] A1: "));
   //Serial.println(a1); 
   //Serial.print(F("[DEBUG] A0: "));
   //Serial.println(a0); 
   if(a1)
   {
      
      digitalWrite(POT_SDI, HIGH);
   }
   else
   {
      digitalWrite(POT_SDI, LOW);
   }
   delayMicroseconds(wait_delay);
   digitalWrite(POT_CLK, HIGH);
   delayMicroseconds(wait_delay);
   digitalWrite(POT_CLK, LOW);
   digitalWrite(POT_SDI, LOW);
   if(a0)
   {
    digitalWrite(POT_SDI, HIGH);
   }
   else
   {
    digitalWrite(POT_SDI, LOW);
   }
   delayMicroseconds(wait_delay);
   digitalWrite(POT_CLK, HIGH);
   delayMicroseconds(wait_delay);
   digitalWrite(POT_CLK, LOW);
   digitalWrite(POT_SDI, LOW);
   for(int i = 0; i < 8; i = i+1)
   {
    //Serial.print(F("[DEBUG] B"));
    //Serial.print(7-i);
    //Serial.print(": ");
    //Serial.println(data[i]);
    if (data[i])
      digitalWrite(POT_SDI, HIGH);
    else
      digitalWrite(POT_SDI, LOW);
    delayMicroseconds(wait_delay);
    digitalWrite(POT_CLK, HIGH);
    delayMicroseconds(wait_delay);
    digitalWrite(POT_CLK, LOW);
    digitalWrite(POT_SDI, LOW);
   }
   //digitalWrite(POT_CSIN, LOW);
   digitalWrite(POT_CSIN, HIGH);
   delayMicroseconds(wait_delay);
//digitalWrite(POT_CSIN, HIGH); //should this be reset to high or not?
//notice the diagram on Datasheet page 10
   
   //}
}

int calculate_tuning_word_bw(double resistance)
{
  /* This function calculates the tuning word needed to get the required resistance 
   *  for R_BW
   */
   double resistance_0 = 45.34;
   double resistance_255 = 9370.0;
   Serial.print(F("[DEBUG] Value of Resistance in calculate_tuning_word_bw(): "));
   Serial.println(resistance);
   double resistance_step = (resistance_255 - resistance_0)/256;
   Serial.print(F("[DEBUG] Resistance Step: "));
   Serial.println(resistance_step);
   int i = 0;
   if ( resistance < resistance_0)
   {
    return 0;
   }
   for (i = 0; i < 256; i++)
   {
      double next_resistance = resistance_0 + resistance_step;
      if ((resistance >= resistance_0) && (resistance < next_resistance))
      {
        return i;
      }
      else
      {
        resistance_0 = next_resistance;
      }
   }
   if (resistance == resistance_255)
   {
      return 255;
   }
   return -1; //error occured
}

double read_in_adc(int adc_pin, int number_of_samples)
{
  /* This function will kick off the FGEn at the correct frequency and will take a 
   *  number of samples and average them.
   */
   //int number_of_averages = 5;
   double adc_values[number_of_samples];
   for(int i = 0; i < number_of_samples; i++)
   {
      adc_values[i] = analogRead(PMT_IN);
   }
   
   
   double avg_value = 0;
   for (int i = 0; i < number_of_samples; i++)
   {
      avg_value = avg_value + ((1.0/number_of_samples) * adc_values[i]);
   }
   return avg_value;
}


void adjust_pmt_sensitivity_voltage(double voltage)
{
  /* This will use a feedback routine to make sure the sensitivity voltage is correct */
  boolean adjusted = 0;
  double pmt_vref_value = 1.1; /* TODO: CHECK WHAT THIS OUTPUT IS ACTUALLY */
  double digipot_error = .1; //10 % error

  
  transfer_value_to_pot(PMT_ADJUST_CHANNEL, 0); //make sure the pot is reset to 0

  if ((voltage < .4) || (voltage > 1.1))
  {
    Serial.println(F("[ERROR] Voltage Passed in to adjust_pmt_sensitivity() below .4 or above 1.1V"));
    return;
  }
  
  double tuning_resistance = 10000*voltage/1.1;
  /* up to 10% error in the potentiometer */
  tuning_resistance = (1-digipot_error) * tuning_resistance;
  int tuning_word = calculate_tuning_word_bw(tuning_resistance);
  Serial.print(F("[DEBUG] Tuning Word: "));
  Serial.println(tuning_word);
  if (tuning_word == -1)
  {
    /* Error Occured */
    Serial.println(F("[ERROR] Error occurred here...Tuning Word bad, resistance out of range"));
  }
  double adc_value, adc_voltage, prev_voltage;
  adc_value = read_in_adc(PMT_SENSITIVITY_VOLTAGE_PIN, 5);
  prev_voltage = adc_to_voltage(adc_value);
  if(prev_voltage > voltage)
  {
    /* ERROR */
    Serial.println(F("[ERROR] Voltage Not acheivable"));
    return;
  }
  transfer_value_to_pot(PMT_ADJUST_CHANNEL, tuning_word);
  adc_value = read_in_adc(PMT_SENSITIVITY_VOLTAGE_PIN, 5);
  prev_voltage = adc_to_voltage(adc_value);
  if ((prev_voltage > voltage) || (prev_voltage > 1.1))
  {
    //Serial.println(F("[ERROR] Voltage Not Acheivable"));
    //return;
    /* If this happens, abandon the speed up, and start from 0 */
    tuning_word = -1; //-1 because adding 1 below
  }
  
  tuning_word = tuning_word + 1;
  if(tuning_word > 255)
  {
    Serial.println(F("[ERROR] Voltage Not Acheivable"));
    return;
  }
  
  while(adjusted == 0)
  {
    transfer_value_to_pot(PMT_ADJUST_CHANNEL, tuning_word);
    adc_value = read_in_adc(PMT_SENSITIVITY_VOLTAGE_PIN, 5);
    adc_voltage = adc_to_voltage(adc_value);
    if ((adc_voltage < voltage) && (prev_voltage > voltage))
    {
      double dist_below = abs(prev_voltage - voltage);
      double dist_above = abs(adc_voltage - voltage);
      if ( dist_below <= dist_above)
      {
        transfer_value_to_pot(PMT_ADJUST_CHANNEL, tuning_word-1);
        adjusted = 1;
      }
      else
      {
        transfer_value_to_pot(PMT_ADJUST_CHANNEL, tuning_word);
        adjusted = 1;
      }
    }
    else
    {
      prev_voltage = adc_voltage;
      tuning_word = tuning_word + 1;
      if (tuning_word > 255)
      {
        Serial.println(F("[ERROR] Voltage Not Acheivable"));
        transfer_value_to_pot(PMT_ADJUST_CHANNEL, 0);
        adjusted = 1;
      }
    }
  }
}

double get_voltage_from_adc(int analog_pin, double vrefl, double vrefh)
{
  int adc_value = analogRead(analog_pin);
  double voltage = vrefl + (double)adc_value * ((vrefh - vrefl)/1024.0);
  return voltage;
}

