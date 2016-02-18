//#include <EEPROMex.h>
//#include <EEPROMVar.h>
#include <WProgram.h>
#include <math.h>
#include <EEPROMex.h>
#include <EEPROMVar.h>


#include "pin.h"
#include "parser.h"
#include "util.h"
#include "memory.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //This line for [DEBUG] to terminal 
  EEPROM.setMaxAllowedWrites(10000);
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

void loop() {
  
  // put your main code here, to run repeatedly:
  //Serial.println(DATA_START_ADDR);
  int get_command = parse_start_command();
  if (get_command == RUN_MODE)
  {
    Serial.println(F("\n[INFO] Running the Memory Check Routine to see if initial data is stored in memory"));
    memory_check();
    wait_for_data_and_continue();
    Serial.println(F("\n[INFO] Turning on Pump to fill Microchamber"));
    power_on_pump();
    Serial.println(F("[INFO] Pump On. Normally would wait for a ~ 15 seconds for chamber to fill."));
    wait_for_data_and_continue();
    Serial.println(F("\n[INFO] Turning on LED, and other peripherals"));
    power_on_everything();
    Serial.println(F("[INFO] Running the demo Calibration Routine."));
    Serial.println(F("[INFO] In order to do manual demo, the voltage being sampled needs to be manually maximized"));
    double calibrated_resonance_freq = demo_calibrate_frequency();
    wait_for_data_and_continue();
    Serial.print(F("\n[INFO] Reading in PMT Values at frequency (Hz): "));
    Serial.print(calibrated_resonance_freq);
    Serial.print(F("\n"));
    Serial.flush();
    double adc_reading = read_in_pmt(calibrated_resonance_freq);
    Serial.print(F("[INFO] ADC Value Read in: "));
    Serial.print(adc_reading);
    Serial.print(F("\nTurning off components...\n"));
    power_off_components();
    wait_for_data_and_continue();
    Serial.println(F("\n[INFO] Calculating actual measured value"));
    double oil_ppm_measurement = convert_adc_to_oil(adc_reading);
    Serial.print(F("[INFO] PPM Measurement: "));
    Serial.println(oil_ppm_measurement);
    Serial.println(F("[INFO] Saving Measurement Value"));
    save_oil_measurement(oil_ppm_measurement);
    Serial.println(F("[INFO] Demo Complete!")); 
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
    
  }
  else if (get_command == CLEAR_MODE)
  {
   
    clear_memory();
    Serial.flush();
  }
  else if (get_command == DUMP_MODE)
  {
    Serial.println(F("\n"));
    Serial.flush();
    memory_dump();
    Serial.println(F("\n"));
    Serial.flush();
    wait_for_data_and_continue();
    
  }
  else
  {
    Serial.println(F("[ERROR] Command not recognized. Please enter command again."));
    Serial.flush();
  }
}
