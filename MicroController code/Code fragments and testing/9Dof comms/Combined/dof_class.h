#include <stdio.h>            // Recommended over iostream for saving space
#include <propeller.h>        // Propeller-specific functions
#include "simpletools.h"



struct xyz  //basic struct which stores 3 dimensional data point
{
  float x;
  float y;
  float z;
} ;


class dof_class
{
  public: 
	dof_class(void);                  //constructor, calls setup functions
	void update();               //main function - updates the xyz structs, calculates new attitudes and angles and so on


   /*
    *The following variables have been deliberately made public to allow fast access to them - 
    *THEY SHOULD NOT BE WRITTEN TO OUSIDE OF THE DOF.UPDATE FUNCTION
    */
  	float theta;            //pitch angle
  	float phi;              //roll angle	
   xyz Gyr_data;           //xyz struct containing gyroscope data
   float Heading;          //Compass reading, relative to north
 
  private:
  	

  

  	float mag_X, mag_Y;     //xMag_h and so on are the magntic componants  in the  and y 
                          //directions and a plane perpendicular to the earths surface. 
  	

  	void read_accel();           //read the accelerometer chip
	void read_gyro();            //read the gyroscope chip
	void read_mag();             //read the magnetometer chip

   xyz Acc_data;             //xyz struct containing accelerometer data
   xyz Mag_data;            //gyr data is public while these are private, as gyr data is
                           //sued directly, while these are converted
   

	void Accel_setup();          //setup the accelerometer
	void Gyro_setup();           //etc.
	void Mag_setup();
	inline int twos2signed(char MSB, char LSB);  
	//function which converts two 8 bit numbers, Most Significant Byte and LSB into a signed integer (efficiently);

};  