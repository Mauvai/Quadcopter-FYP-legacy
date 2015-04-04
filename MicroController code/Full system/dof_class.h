#include <stdio.h>            // Recommended over iostream for saving space
#include <propeller.h>        // Propeller-specific functions
#include "simpletools.h"
#include "fixed.h"


struct xyz  //basic struct which stores 3 dimensional data point
{
  fixed x;
  fixed y;
  fixed z;
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
  	fixed theta;            //pitch angle
  	fixed phi;              //roll angle	
   xyz Gyr_data;           //xyz struct containing gyroscope data
   xyz Mag_data;
   xyz Acc_data;
   
   fixed temp; 
   
  private:
  	

  

  	short x, y, z; //temporary things to store register values, lets compiler handle 2s compliment conversion
  	

  	void read_accel();           //read the accelerometer chip
	void read_gyro();            //read the gyroscope chip
	void read_mag();             //read the magnetometer chip

                //xyz struct containing accelerometer data
                            //gyr and mag  data are public while this is private, as that data is
                           //used externally, while this is converted
   

	void Accel_setup();          //setup the accelerometer
	void Gyro_setup();           //etc.
	void Mag_setup();
	inline int twos2signed(char MSB, char LSB);  
	//function which converts two 8 bit numbers, Most Significant Byte and LSB into a signed integer (efficiently);

};  