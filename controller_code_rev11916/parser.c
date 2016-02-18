int parse_start_command()
{
  /* DEMO FUNCTION -
   *  This function will be used to determine how to start the program
   *  Returns 0 for run mode
   */
   Serial.println(F("\n[INFO] Enter Command to run: "));
   Serial.println(F("[INFO] Commands are 'run' or 'clear' or 'dump'"));
   wait_for_serial_data();
   int number_of_bytes_received = Serial.available();
   char serial_input[number_of_bytes_received];
   int i = 0;
   while(Serial.available() > 0)
   {
     serial_input[i] = Serial.read();
     ++i;
   }   
   if (serial_input[0] == 'r')
   {
      return RUN_MODE;
   }
   else if (serial_input[0] == 'c')
   {
      return CLEAR_MODE;
   }
   else if (serial_input[0] == 'd')
   {
      return DUMP_MODE;
   }
   else
   {
      return UNRECOGNIZED_CMD;
   }
}
