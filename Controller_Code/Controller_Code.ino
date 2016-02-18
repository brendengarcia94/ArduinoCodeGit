//#include <EEPROM.h>

#include <EEPROMex.h>
#include <EEPROMVar.h>
#include <math.h>


/* Microcontroller Code */

/* Pin to turn on the relays */
#define RELAYS 12
#define PUMP_RELAY 11
#define PZT_DRIVER_RELAY 10

/* LED Pin number for debugging */
#define LED 13

/* Pins Allocated to the Function Generator */
#define W_CLK 6
#define FQ_UD 7
#define DATA 8
#define RESET 9

/* PMT Input Pin Number */
#define PMT_IN A0

/* ADC Input Pin Number */
#define TEMP_SENSOR_PIN A1

/* defines to help Readability */
#define RELAY_OFF HIGH
#define RELAY_ON LOW
#define DEBUG_DELAY 500
#define LOWER_FREQ 0
#define RAISE_FREQ 1

/* defines to help with Memory IO */
#define MEM_SIZE EEPROMSizeMicro/2
/* Give the addresses for the different information stored in the EEPROM */
#define CALIBRATED_FREQ_ADDR 0
/* Probably wont be used but adding for support in case */
#define CALIBRATED_TEMP_ADDR CALIBRATED_FREQ_ADDR + sizeof(double)
#define TEMP_ADJUST_STD_DEV_ADDR CALIBRATED_TEMP_ADDR + sizeof(double)
#define FREQ_SWEEP_STEP_ADDR TEMP_ADJUST_STD_DEV_ADDR + sizeof(double)
#define WAIT_DELAY_ADDR FREQ_SWEEP_STEP_ADDR + sizeof(double)
#define NEXT_DATA_ADDR WAIT_DELAY_ADDR + sizeof(long)
#define DATA_START_ADDR NEXT_DATA_ADDR + sizeof(unsigned int)

/* Because of limitations with teh EEPROMex library, will declare a 
 *  clear number to write to
 */

#define CLEAR -100

/* The below will define delays for the project */
#define PUMP_FILL_DELAY 500
/* trap delay should be ~10-13 seconds */
#define TRAP_DELAY 500
/* escape delay should be ~1-3 seconds */
#define ESCAPE_DELAY 500

/* this define gives and easy way to let us change the number of averaged sample from the 
 *  ADC
 */
 #define NUMBER_OF_MEASUREMENTS 10

/*function definitions */

void power_off_routine();
void monitor_chamber_fill();
void wait_for_next_sample();
void separate_oil();
void adjust_frequency();
void pulse_high_signal();
void transfer_byte_to_fgen(byte data); //what is the byte data type

void fill_chamber();

void wait_for_eeprom();
void wait_for_serial_data();

void memory_check();
void check_double();
void power_on_pump();

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

void wait_for_eeprom()
{
  while(!EEPROM.isReady())
  { }
}

void wait_for_serial_data()
{
  while(!Serial.available())
  {}
}

void check_double(int address, String data_name)
{
    /* This function provides a way to check for a double value in the memory at the specified address
    *  and then if not there prompts someone or something to enter in the value.
    *  INPUT:
    *    int address - address of where the double is stored
    *    String data_name - name of the piece of information trying to be retrieved from the device
    */
    
    /* wait for the EEPROM to be ready to read from */
    wait_for_eeprom();
    double value_read = EEPROM.readDouble(address);
    if ((value_read < 0) || isnan(value_read))
    {
      Serial.print("[INFO] No value stored for ");
      Serial.print(data_name);
      Serial.print("\n[INFO] Please Enter Value for data\n");
      wait_for_serial_data();
      int number_of_bytes_received = Serial.available();
      char serial_input[number_of_bytes_received];
      int i = 0;
      while(Serial.available() > 0)
      {
        serial_input[i] = Serial.read();
        ++i;
      }
      char **_;
      value_read = strtod(serial_input, _);
      wait_for_eeprom();
      EEPROM.writeDouble(address, value_read);
      Serial.println("[DEBUG] Validating Data is Written to EEPROM");
      wait_for_eeprom();
      double validate_read = EEPROM.readDouble(address);
      if (validate_read != value_read)
      {
        Serial.print("[ERROR] Error validating back the value for ");
        Serial.print(data_name);
        Serial.print(" to EERPROM\n");
        Serial.print("[ERROR]\tCorrect Value\tValue Read\n");
        Serial.print("[ERROR]\t");
        Serial.print(value_read);
        Serial.print("\t");
        Serial.print(validate_read);
        Serial.print("\n");
      }
      else
      {
        Serial.println("[INFO] Value written to EERPROM validated.");
        Serial.print("[INFO]\tCorrect Value\tValue Read\n");
        Serial.print("[INFO]\t");
        Serial.print(value_read);
        Serial.print("\t");
        Serial.print(validate_read);
        Serial.print("\n");
      }
      
    }
    else
    {
      Serial.print("[DEBUG] ");
      Serial.print(data_name);
      Serial.print(" value: \n");
      Serial.print(value_read);
      Serial.print("\n");
    }
    
}

void check_long(int address, String data_name)
{
    /* This function provides a way to check for a double value in the memory at the specified address
    *  and then if not there prompts someone or something to enter in the value.
    *  INPUT:
    *    int address - address of where the double is stored
    *    String data_name - name of the piece of information trying to be retrieved from the device
    */
    
    /* wait for the EEPROM to be ready to read from */
    wait_for_eeprom();
    long value_read = EEPROM.readLong(address);
    if ((value_read < 0) || isnan(value_read))
    {
      Serial.print("[INFO] No value stored for ");
      Serial.print(data_name);
      Serial.print("\n[INFO] Please Enter Value for data\n");
      wait_for_serial_data();
      int number_of_bytes_received = Serial.available();
      char serial_input[number_of_bytes_received];
      int i = 0;
      while(Serial.available() > 0)
      {
        serial_input[i] = Serial.read();
        ++i;
      }
      char **_;
      value_read = strtol(serial_input, _, 10);
      wait_for_eeprom();
      EEPROM.updateLong(address, value_read);
      Serial.println("[DEBUG] Validating Data is Written to EEPROM");
      wait_for_eeprom();
      long validate_read = EEPROM.readLong(address);
      if (validate_read != value_read)
      {
        Serial.print("[ERROR] Error validating back the value for ");
        Serial.print(data_name);
        Serial.print(" to EERPROM\n");
        Serial.print("[ERROR]\tCorrect Value\tValue Read\n");
        Serial.print("[ERROR]\t");
        Serial.print(value_read);
        Serial.print("\t");
        Serial.print(validate_read);
        Serial.print("\n");
      }
      else
      {
        Serial.println("[INFO] Value written to EERPROM validated.");
        Serial.print("[INFO]\tCorrect Value\tValue Read\n");
        Serial.print("[INFO]\t");
        Serial.print(value_read);
        Serial.print("\t");
        Serial.print(validate_read);
        Serial.print("\n");
      }
      
    }
    else
    {
      Serial.print("[DEBUG] ");
      Serial.print(data_name);
      Serial.print(" value: \n");
      Serial.print(value_read);
      Serial.print("\n");
    }
    
}

void check_next_data_address(int address)
{
    /* This function provides a way to check for a double value in the memory at the specified address
    *  and then if not there prompts someone or something to enter in the value.
    *  INPUT:
    *    int address - address of where the double is stored
    *    String data_name - name of the piece of information trying to be retrieved from the device
    */
    
    /* wait for the EEPROM to be ready to read from */
    wait_for_eeprom();
    int value_read = EEPROM.readInt(address);
    if ((value_read < DATA_START_ADDR) || isnan(value_read) || (value_read > MEM_SIZE))
    {
      Serial.print("[INFO] No valid value stored for Next Data Address");
      Serial.print("\n[INFO] Resetting Next Data Address to \n");
      wait_for_eeprom();
      EEPROM.updateInt(address, DATA_START_ADDR);
      Serial.println("[DEBUG] Validating Data is Written to EEPROM");
      wait_for_eeprom();
      int validate_read = EEPROM.readInt(address);
      if (validate_read != DATA_START_ADDR)
      {
        Serial.print("[ERROR] Error validating back the value for Next Data Address");
        Serial.print(" to EERPROM\n");
        Serial.print("[ERROR]\tCorrect Value\tValue Read\n");
        Serial.print("[ERROR]\t");
        Serial.print(DATA_START_ADDR);
        Serial.print("\t");
        Serial.print(validate_read);
        Serial.print("\n");
      }
      else
      {
        Serial.println("[INFO] Value written to EERPROM validated.");
        Serial.print("[INFO]\tCorrect Value\tValue Read\n");
        Serial.print("[INFO]\t");
        Serial.print(DATA_START_ADDR);
        Serial.print("\t");
        Serial.print(validate_read);
        Serial.print("\n");
      }
      
    }
    else
    {
      Serial.print("[DEBUG] ");
      Serial.print("Next Data Address ");
      Serial.print(" value: \n");
      Serial.print(value_read);
      Serial.print("\n");
    }
    
}

void memory_check()
{
  /* This function will make sure the correct values are in the EEPROM
   *  memory before the program begins. If the values are not in the 
   *  EEPROM then the Chip will ping serial communication for a value to
   *  use.
   */
   Serial.println("[DEBUG] Running memory check routine");
   check_double(CALIBRATED_FREQ_ADDR, "Microchamber Calibration Frequency");
   check_double(TEMP_ADJUST_STD_DEV_ADDR, "Frequency Std. Dev or Fit number");
   check_double(FREQ_SWEEP_STEP_ADDR, "Amount of frequency to step with during Calibration Routine");
   check_long(WAIT_DELAY_ADDR, "Number of ms to wait in between measurements if no manual trigger");
   check_next_data_address(NEXT_DATA_ADDR);
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

void wait_for_pump()
{
  /* This function waits for the selected amount of time for the pump to fill
   *  the microchamber with the water around the buoy
   *  The time to wait for pump to fill will be in the PUMP_FILL_DELAY macro
   *  defined at the top of this sketch.
   */
   delay(PUMP_FILL_DELAY);
}

double adc_to_voltage(double adc_value)
{
  /* This function provides a map from the adc value given to the volatage
   *  value that it is mapped to.
   * This will return a value in V (not mV)
   */
   
   double vrefh = 5; //5 V
   double vrefl = 0; //0 V
   int adc_res = 1024; //2^10 (10 bits)
   double adc_volt = map(adc_value, 0, adc_res, vrefl, vrefh);
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
   int lower_temp = 0; //Celcius
   int lower_mv = 1034; //mV
   int higher_temp = 32; //Celcius 
   int higher_mv = 860; //mV
   /* The below function is obtained from the LMT84LP data sheet and is the linear approximation of a value when finding
    *  temperatures within a particular range (0-32 C) for the average Ocean temperatures across time of year.
    */
   double approx_temp = (temp_sensor_out_voltage - lower_mv)*((higher_temp - lower_temp)/(higher_mv - lower_mv))+lower_temp;
   return approx_temp;
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

void initialize_fgen()
{ 
  /* This function will initialize the fgen after turning it on */
  pulse_high_signal(RESET);
  pulse_high_signal(W_CLK);
  pulse_high_signal(FQ_UD);
}

double sweep_frequency(double low_freq,double high_freq,double nom_freq,double step_size)
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
      digitalWrite(PZT_DRIVER_RELAY, RELAY_ON);
      initialize_fgen();
      adjust_frequency(current_freq);
      /* wait for the PZT's to be able to trap */
      /* Obtained this value from previous research paper on this topic, used plot */
      delay(TRAP_DELAY);
      int readings[4];
      readings[0] = analogRead(PMT_IN);
      readings[1] = analogRead(PMT_IN);
      readings[2] = analogRead(PMT_IN);
      readings[3] = analogRead(PMT_IN);
      double avg = (readings[0] + readings[1] + readings[2] + readings[3])/4;
      if (avg > max_adc_value)
      {
        max_adc_value = avg;
        max_freq = current_freq;
      }
      current_freq = current_freq + step_size;
      digitalWrite(PZT_DRIVER_RELAY, RELAY_OFF);
      delay(ESCAPE_DELAY);
   }
   return max_freq;
}

double calibrate_frequency()
{
  /* This function will run through and calibrate the frequency with input from a 
   *  Temperature Sensor. The effect of temperature on teh resonance frequency is a known
   *  relationship with some statistical error. Idea is to sweep within 2-3 std. deviations
   *  of the new projected resonance frequency. Will use a stored step size as well.
   */
   double calibrated_frequency = EEPROM.readDouble(CALIBRATED_FREQ_ADDR);
   double temperature = read_ambient_temperature(); 
   double temp_calibrated_frequency = map_to_temperature_frequency(calibrated_frequency, temperature);
   double std_dev = EEPROM.readDouble(TEMP_ADJUST_STD_DEV_ADDR);
   double sweep_low_freq = temp_calibrated_frequency - ( 3 * std_dev );
   double sweep_high_freq = temp_calibrated_frequency + (3 * std_dev );
   double step_size = EEPROM.readDouble(FREQ_SWEEP_STEP_ADDR);
   double adjusted_freq = sweep_frequency(sweep_low_freq, sweep_high_freq, temp_calibrated_frequency, step_size);
   return adjusted_freq;
   
}


double read_in_pmt(double frequency)
{
  /* This function will kick off the FGEn at the correct frequency and will take a 
   *  number of samples and average them.
   */
   double adc_values[NUMBER_OF_MEASUREMENTS];
   digitalWrite(PZT_DRIVER_RELAY, RELAY_ON);
   initialize_fgen();
   adjust_frequency(frequency);
   delay(TRAP_DELAY);
   for(int i = 0; i < NUMBER_OF_MEASUREMENTS; i++)
   {
      adc_values[i] = analogRead(PMT_IN);
   }
   digitalWrite(PZT_DRIVER_RELAY, RELAY_OFF);
   delay(ESCAPE_DELAY);
   
   double avg_value = 0;
   for (int i = 0; i < NUMBER_OF_MEASUREMENTS; i++)
   {
      avg_value = avg_value + ((1/NUMBER_OF_MEASUREMENTS) * adc_values[i]);
   }
   return avg_value;
}

void power_off_components()
{
  digitalWrite(PUMP_RELAY, RELAY_OFF);
  digitalWrite(RELAYS, RELAY_OFF);
  digitalWrite(PZT_DRIVER_RELAY, RELAY_OFF);
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

double convert_adc_to_oil(double adc_value)
{
    double adc_voltage = adc_to_voltage(adc_value);
    /* TODO: Here is where you would put the Voltage->Measurement value.
     *  For now, just return old value.
     */
    double oil_measure = pmt_volt_to_oil(adc_voltage);
}

int get_next_addr(int prev_addr, int mem_size)
{
  /* This function will calculate the next address in which 
   *  to store data. This function will consider the size of the memory and will
   *  adhere to wrap the data around in the correct manner.
   *  INPUTS:
   *    prev_addr - the address where the previous data is stored
   *    mem_size - the total number of addresses for the board
   *  RETURNS:
   *    location to save next thing of data
   */
   int next_addr;
   if ((prev_addr + sizeof(double)) > (mem_size - sizeof(double)))
   {
      next_addr = DATA_START_ADDR;
   }
   else
   {
      next_addr = prev_addr + sizeof(double);
   }
   return next_addr;
}

void save_oil_measurement(double measurement)
{
  /* This function is charged with storing the oil measurement in the EEPROM
   *  memory. I it will need to read in the next address memory entry and then store
   *  the data in that entry and then will update all appropriate memory values.
   */
   wait_for_eeprom();
   int address_to_save_to = EEPROM.readInt(NEXT_DATA_ADDR);
   EEPROM.updateDouble(address_to_save_to, measurement);
   int next_address = get_next_addr(address_to_save_to, MEM_SIZE);
   EEPROM.updateInt(NEXT_DATA_ADDR, next_address);
}

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600); //This line for [DEBUG] to terminal 
  
  pinMode(RELAYS, OUTPUT); //set up the relays output pin
  pinMode(PUMP_RELAY, OUTPUT); //set the pump relay for output
  pinMode(LED, OUTPUT); // set the LED up for outputting light
  pinMode(PZT_DRIVER_RELAY, OUTPUT);
  
  digitalWrite(RELAYS, RELAY_OFF); //Power off the relays 
  digitalWrite(PUMP_RELAY, RELAY_OFF);
  digitalWrite(PZT_DRIVER_RELAY, RELAY_OFF);
  
  pinMode(PMT_IN, INPUT); //define the analog pin to read PMT as input
  pinMode(TEMP_SENSOR_PIN, INPUT);
  
  /* Set up Function Generator Pins */
  pinMode(FQ_UD, OUTPUT);
  pinMode(W_CLK, OUTPUT);
  pinMode(DATA, OUTPUT);
  pinMode(RESET, OUTPUT);

  while(!Serial) //wait until you can actually read and write from the serial port connected
  {}
  
}

void loop() 
{
  Serial.println(DATA_START_ADDR);
  memory_check();
  /*
  power_on_pump();
  wait_for_pump();
  power_on_everything();
  double calibrated_resonance_freq = calibrate_frequency();
  double adc_reading = read_in_pmt(calibrated_resonance_freq);
  power_off_components();
  double oil_ppm_measurement = convert_adc_to_oil(adc_reading);
  save_oil_measurement(oil_ppm_measurement);
  long delay_time = EEPROM.readLong(WAIT_DELAY_ADDR);
  delay(delay_time);
  */
  delay(5000);
}
