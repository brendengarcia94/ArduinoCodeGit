#include "memory.h"

void memory_check()
{
  /* This function will make sure the correct values are in the EEPROM
   *  memory before the program begins. If the values are not in the 
   *  EEPROM then the Chip will ping serial communication for a value to
   *  use.
   */
   Serial.println(F("[DEBUG] Running memory check routine"));
   check_double(CALIBRATED_FREQ_ADDR, "Microchamber Calibration Frequency");
   Serial.print(F("[INFO] Current Temperature (Celcius): "));
   Serial.println(read_ambient_temperature());
   check_double(CALIBRATED_TEMP_ADDR, "Temperature Calibrated Frequency at in Celcius");
   check_double(TEMP_ADJUST_STD_DEV_ADDR, "Frequency Std. Dev or Fit number");
   check_double(FREQ_SWEEP_STEP_ADDR, "Amount of frequency to step with during Calibration Routine");
   check_long(WAIT_DELAY_ADDR, "Number of ms to wait in between measurements if no manual trigger");
   check_next_data_address(NEXT_DATA_ADDR);
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
      Serial.print(F("[DEBUG] "));
      Serial.print(data_name);
      Serial.print(F(" value: "));
      Serial.print(value_read);
      Serial.print(F("\n"));
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
      Serial.print(F("[DEBUG] "));
      Serial.print(data_name);
      Serial.print(F(" value: "));
      Serial.print(value_read);
      Serial.print(F("\n"));
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
    if ((value_read < DATA_START_ADDR) || isnan(value_read) || (value_read > MEM_SIZE) || (value_read == CLEAR))
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
      Serial.print(F("[DEBUG] "));
      Serial.print(F("Next Data Address "));
      Serial.print(F(" value: "));
      Serial.print(value_read);
      Serial.print(F("\n"));
    }
    
}
