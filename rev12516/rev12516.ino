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
#define DATA_START_ADDR NEXT_DATA_ADDR + sizeof(int)

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

/* DEMO DEFINITIONS */
#define RUN_MODE 0
#define CLEAR_MODE 1
#define DUMP_MODE 2
#define UNRECOGNIZED_CMD -1

/* During Compilation, the functions defined in the tabs will be 
 *  declared here before compilation begins.
 */
 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //This line for [DEBUG] to terminal 
  EEPROM.setMaxAllowedWrites(10000); //TODO: Find real way to make this work, this is a workaround
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
  // 1.) Get the start command
  int get_command = parse_start_command();
  //2.) Decide what to do with the parsed command
  if (get_command == RUN_MODE) //run mode means run the demo program
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
