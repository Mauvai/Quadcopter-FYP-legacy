/*
  SD Minimal.side

  Create test.txt, write characters in, read back out, display.
*/

#include "simpletools.h"                      // Include simpletools header    

int MOSI = 22, SCLK = 23, MISO = 24, SS = 25;      // SD card pins on Propeller BOE

// DO  = MOSI
// DI  = MISO
// CLK = SCLK
// CS  = SS

// grey wire is MISO:     pin 24
// brown wire is SCLK:    pin 23 
// black wire is SS:      pin 25
// yellow wire is MOSI:   pin 22

int main(void)                                // main function
{
  sd_mount(MOSI, SCLK, MISO, SS);                  // Mount SD card

  FILE* fp = fopen("test.txt", "w");          // Open a file for writing
  fwrite("Testing 123...\n", 1, 15, fp);      // Add contents to the file
  fclose(fp);                                 // Close the file
 
  char s[15];                                 // Buffer for characters
  fp = fopen("test.txt", "r");                // Reopen file for reading
  fread(s, 1, 15, fp);                        // Read 15 characters
  fclose(fp);                                 // Close the file

  print("First 15 chars in test.txt:\n");     // Display heading
  print("%s", s);                             // Display characters
  print("\n");                                // With a newline at the end
}  