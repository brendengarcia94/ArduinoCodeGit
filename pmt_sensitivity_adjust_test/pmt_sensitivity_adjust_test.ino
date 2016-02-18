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

/* ADC Input for PMT Sensitivity Adjust */
#define PMT_SENSITIVITY_VOLTAGE_PIN A2

/* Digital Potentiometer Control */
#define POT_CLK 5
#define POT_SDI 4
#define POT_CSIN 3
#define PMT_ADJUST_CHANNEL 1

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
  pinMode(PMT_SENSITIVITY_VOLTAGE_PIN, INPUT);
  
  /* Set up Function Generator Pins */
  pinMode(FQ_UD, OUTPUT);
  pinMode(W_CLK, OUTPUT);
  pinMode(DATA, OUTPUT);
  pinMode(RESET, OUTPUT);

  /* Set up Digital Pot Bits */
  pinMode(POT_CLK, OUTPUT);
  pinMode(POT_SDI, OUTPUT);
  pinMode(POT_CSIN, OUTPUT);

  transfer_value_to_pot(PMT_ADJUST_CHANNEL, 0); //to make sure voltage is low
  
  while(!Serial) //wait until you can actually read and write from the serial port connected
  {}

}

void loop() {
  //analogReference(EXTERNAL); //for this test, vref will be ~1.4V
  delay(1);
  Serial.println(F("Voltage Value Passed In,Voltage Value Read\n"));
  double i = 0.4;
  double step_size = 0.001;
  double vrefh = 5;
  while(i<=1.1)
  {
    adjust_pmt_sensitivity_voltage(i, 1.1);
    delay(1);
    double voltage = get_averaged_voltage_from_adc(PMT_SENSITIVITY_VOLTAGE_PIN, 0, vrefh);
    Serial.print(i,3);
    Serial.print(",");
    Serial.print(voltage,5);
    Serial.print("\n");
    Serial.flush();
    i = i + step_size;
  }
  Serial.println(F("Data Collection Complete"));
  //while(true)
  //{;}
  wait_for_data_and_continue();
}
