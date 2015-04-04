#include "simpletools.h"                      // Include simpletools header    

int MISO = 0;
int SCLK = 2;
int MOSI = 4;
int SS   = 6;

/*
int MISO = 12;
int SCLK = 13;
int MOSI = 14;
int SS   = 15;
*/  
  
     
// SD card pins on Propeller BOE

// DO  = MISO
// DI  = MOSI
// CLK = SCLK
// CS  = SS

// red    wire is DO   :    pin 12 (MISO)
// black  wire is SCLK :    pin 13 (SCLK)
// yellow wire is DI   :    pin 14 (MOSI)
// green  wire is CS   :    pin 15 (SS)
 
  
int main(void)                                // main function
{
   
 char data [8] = {'2', '3' , '5', '7', '11', '13'};
 
 int mout = sd_mount (MISO, SCLK, MOSI, SS);
  
/*

 print("%d \n", mout);  
  FILE* fp = fopen("test.txt", "w");          // Open a file for writing
  //fwrite("Hello World....", 1, 15, fp);     // Add contents to the file


int ax = 1;
int ay = 2;
int az = 3;

int gx = 5;
int gy = 7;
int gz = 11;
fprintf(fp,"A,%i,%i,%i,G,%d,%d,%d\n",ax,ay,az,gx,gy,gz);  

//fwrite("a,1,2,3,g,12,23,56", 1, 18, fp);                // Add contents to the file
//fwrite(&ax, 4, 1, fp);                                  // Add contents to the file
   
  fclose(fp);                                             // Close the file



  char s[8];                                  // Buffer for characters
  fp = fopen("test.txt", "r");                // Reopen file for reading
  fread(&s, 4, 8, fp);                        // Read 15 characters
  fclose(fp);                                 // Close the file

  print("First 15 chars in test.txt:\n");     // Display heading
  print("%s", s);                             // Display characters
  print("\n");  
  pause(500);
*/


  char c = 1;
  
FILE * pFile = fopen ("alphabet.txt","w");

      fputc ( c , pFile );

    fclose (pFile);
  



while (1)
{ 

}
}  