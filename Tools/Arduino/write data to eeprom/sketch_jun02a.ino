//Include the Wire I2C Library
#include <Wire.h>

/*This address is determined by the way your address pins are wired.
In the diagram from earlier, we connected A0 and A1 to Ground and 
A2 to 5V. To get the address, we start with the control code from 
the datasheet (1010) and add the logic state for each address pin
in the order A2, A1, A0 (100) which gives us 0b1010100, or in 
Hexadecimal, 0x54*/

#define EEPROM_ADR 0x50

/*Theoretically, the 24LC256 has a 64-byte page write buffer but 
we'll write 16 at a time to be safe*/ 

#define MAX_I2C_WRITE 16 

byte tempStore[MAX_I2C_WRITE];

void setup()
{

//Start the I2C Library
  Wire.begin();
  Wire.setClock(400000);

//Start the serial port
  Serial.begin(19200);

//Here is where we'll keep track of where in the memory we're writing
  long currentSpot = 0;
  long timerReset = 0;
  long lastWroteTime = 0;
  int lastWrotenDataLength = 0; 
  byte counter = 0;
  bool wrotenLastByte = false;

  //Here we listen for bytes on the serial port and increment
  //the counter as we store them in our tempStore variable
  while (1)
  {
    while (Serial.available())
    {
      tempStore[counter++] = Serial.read(); //Read this byte into the array

      if (counter == MAX_I2C_WRITE)
      {
      //Once we've collected a page worth, go ahead and do 
      //a page write operation
        writeEEPROMPage(currentSpot);
        counter = 0; //Reset
        currentSpot += MAX_I2C_WRITE;

        lastWrotenDataLength = (sizeof(tempStore) / sizeof(tempStore[0]));
        lastWroteTime = millis();
      }

      timerReset = millis();
    }

      if(millis() - lastWroteTime > 10000 && !wrotenLastByte)
      {
          tempStore[counter++] = 'E';
          tempStore[counter++] = 'G';
          tempStore[counter++] = 'E';
          
          writeEEPROMPage(currentSpot);
          counter = 0; //Reset
          currentSpot += MAX_I2C_WRITE;
          wrotenLastByte = true;
      }

    if (millis() - timerReset > 2000)
    {
      Serial.println(currentSpot);
      Serial.println(wrotenLastByte);
      timerReset = millis();
    }
  }

}

void loop()
{
    // Don't do anything here
}

/* This is the 3 step memory writing procedure that
we talked about. First we send the MSB of the address. 
Then we send the LSB of the address. Then we send the 
data that we want to store. */

void writeEEPROMPage(long eeAddress)
{

  Wire.beginTransmission(EEPROM_ADR);

  Wire.write((int)(eeAddress >> 8)); // MSB
  Wire.write((int)(eeAddress & 0xFF)); // LSB

  //Write bytes to EEPROM
  for (byte x = 0 ; x < MAX_I2C_WRITE ; x++)
    Wire.write(tempStore[x]); //Write the data

  Wire.endTransmission(); //Send stop condition
}
