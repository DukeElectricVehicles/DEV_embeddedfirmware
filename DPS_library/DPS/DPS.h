/*
  Library for communicating with DPS50xx converters over Serial1
  Jack Proudfoot
  March 19, 2019
*/

// ensure this library description is only included once
#ifndef DPS_h
#define DPS_h

// library interface description
class DPS
{
  public:
    DPS();

    float* get();
    float get_voltage();
    float get_current();
    float setVoltage(float);

  private:
    char* transmit(char*, int);
    char* receive_bytes();

    //crc function taken from https://stackoverflow.com/questions/19347685/calculating-modbus-rtu-crc-16
    unsigned int crc(char *bufin, int len) {
      unsigned char* buf = (unsigned char*) bufin;
      unsigned int crc = 0xFFFF;
      for (int pos = 0; pos < len; pos++) {
        crc ^= (unsigned int)buf[pos];    // XOR byte into least sig. byte of crc

        for (int i = 8; i != 0; i--) {    // Loop over each bit
          if ((crc & 0x0001) != 0) {      // If the LSB is set
            crc >>= 1;                    // Shift right and XOR 0xA001
            crc ^= 0xA001;
          }
          else                            // Else LSB is not set
            crc >>= 1;                    // Just shift right
        }
      }

      return crc;
    }
};

#endif
