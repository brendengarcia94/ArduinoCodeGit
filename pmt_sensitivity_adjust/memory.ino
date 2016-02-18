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

void wait_for_eeprom()
{
  while(!EEPROM.isReady())
  { }
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

void clear_memory()
{
  for(int i = 0; i < MEM_SIZE; i = i + 2)
    {
      //Serial.println(i);
      Serial.print(F("[DEBUG] Clearing Address Location "));
      Serial.println(i);
      Serial.flush();
      EEPROM.updateInt(i, CLEAR);
    }
    Serial.println(F("[INFO] MEMORY CLEARED!\n"));
}

void memory_dump()
{
  /* This function will go through all of the memory in the Arduino and will print out
   *  the stored values in the device to show user in Serial console.
   */
   double temp_read_double;
   long temp_read_long;
   int temp_read_int;

   /* Just print out the "MetaData" of the project, calibration information */
   Serial.println(F("[INFO] Below is the result of the memory dump."));
   Serial.println(F("[INFO]\tAddress(Hex)\tAddress(Dec)\tName of Data\tValue"));
   print_double_meta_data(CALIBRATED_FREQ_ADDR, "Calibrated Resonance Frequency");
   print_double_meta_data(CALIBRATED_TEMP_ADDR, "Temperature In Celcius Calibrated At.");
   print_double_meta_data(TEMP_ADJUST_STD_DEV_ADDR, "Amount of error in Temperature adjust model for resonance.");
   print_double_meta_data(FREQ_SWEEP_STEP_ADDR, "Step size to use in calibration routine in lab (in Hz)");
   print_long_meta_data(WAIT_DELAY_ADDR, "Number of (ms) to wait between measurements");
   print_int_meta_data(NEXT_DATA_ADDR, "The address to store the next data entry");
   dump_data();
}

void print_double_meta_data(int addr, String data_name)
{
    double temp_read_double;  
   temp_read_double = EEPROM.readDouble(addr);
   Serial.print(F("[INFO]\t"));
   Serial.print(addr, HEX);
   Serial.print(F("\t"));
   Serial.print(addr);
   Serial.print(F("\t"));
   Serial.print(data_name);
   Serial.print(F("\t"));
   Serial.println(temp_read_double);
   Serial.flush();
}

void print_long_meta_data(int addr, String data_name)
{
    double temp_read;  
   temp_read = EEPROM.readLong(addr);
   Serial.print(F("[INFO]\t"));
   Serial.print(addr, HEX);
   Serial.print(F("\t"));
   Serial.print(addr);
   Serial.print(F("\t"));
   Serial.print(data_name);
   Serial.print(F("\t"));
   Serial.println(temp_read);
   Serial.flush();
}

void print_int_meta_data(int addr, String data_name)
{
   int temp_read;  
   temp_read = EEPROM.readInt(addr);
   Serial.print(F("[INFO]\t"));
   Serial.print(addr, HEX);
   Serial.print(F("\t"));
   Serial.print(addr);
   Serial.print(F("\t"));
   Serial.print(data_name);
   Serial.print(F("\t"));
   Serial.println(temp_read);
   Serial.flush();
}


void dump_data()
{
  /* This function will go and dump the data out to the screen */
  Serial.println(F("\n[INFO] Below is the measurement data stored in device."));
  Serial.println(F("[INFO]\tAddress(Hex)\tAddress(Dec)\tData Number\tValue"));
  int amount_of_data = 0;
  int oldest_entry_addr = EEPROM.readInt(NEXT_DATA_ADDR);
  int addr = oldest_entry_addr;
  if (addr == CLEAR)
  {
    Serial.print(F("\n[INFO] Total Number of Data Measurements: "));
    Serial.println(amount_of_data);
    Serial.flush();
    return;
  }
  int read_int = EEPROM.readInt(addr);
  double read_double;
  if (read_int != CLEAR && !isnan(read_int))
  {
    ++amount_of_data;
    read_double = EEPROM.readDouble(addr);
    Serial.print(F("[INFO]\t"));
    Serial.print(addr, HEX);
    Serial.print(F("\t"));
    Serial.print(addr, DEC);
    Serial.print(F("\t"));
    Serial.print(amount_of_data);
    Serial.print(F("\t"));
    Serial.println(read_double);
  }
  addr = get_next_addr(addr, MEM_SIZE);
  while(addr != oldest_entry_addr)
  {
    read_int = EEPROM.readInt(addr);
    read_double;
    if (read_int != CLEAR && !isnan(read_int))
    {
      ++amount_of_data;
      read_double = EEPROM.readDouble(addr);
      Serial.print(F("[INFO]\t"));
      Serial.print(addr, HEX);
      Serial.print(F("\t"));
      Serial.print(addr, DEC);
      Serial.print(F("\t"));
      Serial.print(amount_of_data);
      Serial.print(F("\t"));
      Serial.println(read_double);
    }
    addr = get_next_addr(addr, MEM_SIZE);
  }
  Serial.print(F("\n[INFO] Total Number of Data Measurements: "));
  Serial.println(amount_of_data);
  Serial.flush();
}
