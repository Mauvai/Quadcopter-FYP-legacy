/* 
CURRENTLY INCOMPETE control system for a quadcopter

Uses a 
* 9dof sensor stick
* second order complementary filter
* state space control model
*/

//prop lib files
#include <stdio.h>            // Recommended over iostream for saving space
#include <propeller.h>        // Propeller-specific functions
#include "simpletools.h"
#include "pwm32.h"


//custom files
#include "dof_class.h"
#include "filters.h"
#include "control.h"
#include "efficient_math.h"
#include "fixed.h"
#include "ultrasonics.h"
#include "ADF7024.h"


#define _RADIO_   

 //Comment this out to deactivate radio commands 
//(will switch to 0 set point bno input control)


//#define _PWM_   

 //Comment this out to deactivate PWM commands
//(Used for debugging - can run control/filters with no motor output)


//Globals - these no longer need to be volatile, as the multicoreing for these sections is gone
fixed theta, phi, Heading, mag_x, mag_y, mag_z, gyr_x, gyr_y, gyr_z;  //raw dof readings

fixed filt_theta, filt_phi;   //filtered pitch and roll angles

fixed height;  //height form distance sensor, in metres
volatile int _height, _ftheta, _fphi; //these are the volatiles that get transfered through cores
                                    //for the height sensor - no volatile version of fixed!
int echopin = 13;
int triggerpin = 12;  //pins for HC-SR04 ultrasonic sensor




fixed yaw_set_point, pitch_set_point, roll_set_point, height_set_point;



int m1_pw, m2_pw, m3_pw, m4_pw;    //motor pulsewidths in mm
fixed u1,    u2,    u3,    u4;       //channel values




extern float _sample_period = 0.02;  
fixed sample_period(_sample_period);
//30ms sample period
//declare as extern so it is visible in all files that declare the following:
//extern const float sample_period;

const fixed sample_freq = 1/sample_period.f();

extern fixed pi(3.14159265359);
extern fixed degree2rad(pi/180);
extern fixed rad2degree(180/pi.f());
extern fixed degree2rad_squared = degree2rad*degree2rad;

volatile int cmd_word = 0;
volatile int cmd_direction = 0;  //directional changes
volatile int run = 0;            //tells it whether to go to kill motors, low power, or full control
char data_in[10];

//function declarations

void run_height_sensor(void *par);

void run_filter(void *par);
void init_setpoints(void);
inline fixed pwm_clamp(fixed pwm);

volatile int packet_count = 0;  //count of radio packets that have been recieved
volatile int previous_packet_count = 0;


#ifdef _RADIO_

void run_radio(void *par);
char interpret_radio(void);


int comm_pin0 = 0;
int comm_pin1 = 1;

int rad_CS   = 20;
int rad_MOSI = 19;
int rad_CLK  = 18;
int rad_MISO = 17;
volatile char rad_status;
//radio function defs

#endif 
//_RADIO_

#ifdef _PWM_
//start the pwm32 library
void pwm2();
int motor_1  = 4;  //motor 1
int motor_2  = 5;  //motor 2
int motor_3  = 7;  //motor 3
int motor_4  = 6;  //motor 4
const int pwm_period = 15000;
#endif
//_PWM_


using namespace libpropeller;


int main()
{
  print("Mark\n");

  cog_run(&run_height_sensor, 35);
  
  #ifdef _RADIO_
  cog_run(&run_radio, 100);
  pause(1000);
  print("Radio Status: %x\n", rad_status);  
  #endif
  
  
  
  #ifdef _PWM_  
  PWM32 pwm1; //PWM32 object constructor
  pwm1.Start();
  
     //Motor startup routine
  for (int i = motor_1; i<=motor_3; i = i+1)
  {
    pwm1.Servo(i, 900); //send arming signal
    pause(250);           //wait for arm
   
  } 
  pause(2000);//wait for arm
  #ifndef _RADIO_    //only finish arming the motors if the radio code isnt running - 
                    //the radio controller switch sequence takes care of this by itself
  for (int i = motor_1; i<=motor_3; i = i+1)
  {
             
    pwm1.Servo(i, 1200); //spin low
    pause(250);
  }   
  #endif  
  //_RADIO_
  #endif
  //_PWM_
  pause(100);
   
  
  
  int dt; volatile int t;          //stores timing values: t holds a clock value, dt is compared to it
  //t needs to be volatile, as the compiler does some stupid optimisations and it breaks sometimes
  
  dt = sample_period.f()*CLKFREQ;    // 2Hz internal sampling  (CLKFREQ = no of clock cycles in 1 second)
  //dt = CLKFREQ/1.5;
  
  int count = 0;
  fixed mag_Xh, mag_Yh;    //xMag_h and so on are the magnetic componants  in the  and y 
                          //directions and a plane perpendicular to the earths surface. 
  
  


  dof_class dof; //build a sensor object called dof 
  pause(100);
  
  
  init_setpoints();
  dt = sample_period.f()*CLKFREQ;   
  //print("\ndt : %d", dt);
  
  
  
  while(1)
  {
    if (count == 1)
    {print("\n Mark");}
    
    t = CNT;
    count++;
    dof.update();
    
    /* 
     * The below is ugly but deliberate - fastest way to assign variables
     * directly out of the class, which is non volatile, into volatile 
     * variables which are read elsewhere
     * 
     * The dof.whatever variables should NEVER BE WRITTEN TO (outside the
     * dof.update function) - they not be read from either, as they will not 
    
     * update. Only the volatile floats (theta, phi, gyr_x etc.) here should
     * ever be read from. 
     */
    theta = dof.theta;
    phi   = dof.phi;
    mag_x = dof.Mag_data.x;
    mag_y = dof.Mag_data.y;
    mag_z = dof.Mag_data.z;
    gyr_x = dof.Gyr_data.x*degree2rad;   //be very careful - gyr_x is rotation AROUND x, and is phi rate, not theta rate!!!
    gyr_y = dof.Gyr_data.y*degree2rad;   //similarly here - gyr_y is theta rate!
    gyr_z = dof.Gyr_data.z*degree2rad;
    
    filt_theta = pitch_comp_filter(theta, gyr_y);  //run pitch filter
    filt_phi   =  roll_comp_filter(phi,   gyr_x); //run roll filter 
    
    _ftheta = filt_theta.i();
    _fphi   = filt_phi.i();
    
    height.set(_height); //need this trickery to essentially pass fixed's as volatiles - through an int. 
                        //also the hieght data will very likely be out of date....
    
    //linearised magnetometer functions. See Simon's logbook, week 4. 
    //Too much to explain here
    /*
    mag_Xh = mag_x + mag_y*filt_theta*filt_phi + mag_z*filt_theta;
    mag_Yh =         mag_y                      -mag_z*filt_phi;  
    */
    
    /*
    mag_Yh = mag_x;
    mag_Xh = mag_y-filt_theta*filt_phi*mag_x;
    */

    mag_Yh = mag_x;
    mag_Xh = mag_y -filt_phi*mag_z - mag_x;

    Heading = atan2f_cordic_rad(mag_Yh.i(), mag_Xh.i());  
    fixed Heading2 = atan2f_cordic_rad(mag_y.i(), mag_x.i()); //calculate heading assuming sensor is flat
    
    #ifdef _RADIO_
    if (packet_count > previous_packet_count)
    {
        previous_packet_count = packet_count;
        cmd_word = interpret_radio(); //also sets all the set points, which are globals
    }        
    #endif
    

    #ifdef _RADIO_   //remove command word processing if radios arent included
    cmd_word=0x01;
    if (cmd_word == 0x03)
    {
        #ifdef _PWM_
        pwm1.PWM(motor_1, 900, pwm_period-900);  //these functions require an
        pwm1.PWM(motor_2, 900, pwm_period-900);  //integer number of microseconds
        pwm1.PWM(motor_3, 900, pwm_period-900);
        pwm1.PWM(motor_4, 900, pwm_period-900);
        #endif
    }    
    else if (cmd_word == 0x02)
    {
        #ifdef _PWM_
        pwm1.PWM(motor_1, 1200, pwm_period-1200);  //these functions require an
        pwm1.PWM(motor_2, 1200, pwm_period-1200);  //integer number of microseconds
        pwm1.PWM(motor_3, 1200, pwm_period-1200);
        pwm1.PWM(motor_4, 1200, pwm_period-1200);
        #endif
    }      
    else if (cmd_word == 0x01)
    {   
    #endif   
        //u1 = control_height(height,   height_set_point); 
        u1 = 1.500;  //hovering setpoint - no height control
        u2 = control_roll( filt_phi,   gyr_x,  roll_set_point, u2);  
        u3 = control_pitch(filt_theta, gyr_y, pitch_set_point, u3); 
        u4 = control_yaw(  Heading,    gyr_z,   yaw_set_point);     
        //u3 = 0;
        //u4 = 0;
    
    
        m1_pw = pwm_clamp(u1   -u3+u4).i()*1000/(1<<16);
        m2_pw = pwm_clamp(u1-u2   -u4).i()*1000/(1<<16);
        m3_pw = pwm_clamp(u1   +u3+u4).i()*1000/(1<<16); 
        m4_pw = pwm_clamp(u1+u2   -u4).i()*1000/(1<<16);
    
        #ifdef _PWM_
        pwm1.PWM(motor_1, m1_pw, pwm_period-m1_pw);  //these functions require an
        pwm1.PWM(motor_2, m2_pw, pwm_period-m2_pw);  //integer number of microseconds
        pwm1.PWM(motor_3, m3_pw, pwm_period-m3_pw);
        pwm1.PWM(motor_4, m4_pw, pwm_period-m4_pw);
        #endif
    #ifdef _RADIO_
    }    
    #endif

    
    if (count == 30)
    {
      //print("\nTime: %f", (float)(CNT-t)/(float)CLKFREQ);
      count = 0;
      
      //print("\nAccx: %f, , Accy: %f, Accz: %f", dof.Acc_data.x.f(), dof.Acc_data.y.f(), dof.Acc_data.z.f());  
      print( "\ntheta: %f, phi: %f, Heading: %f, Unfiltered heading: %f",  (theta*rad2degree).f(), (phi*rad2degree).f(),  (Heading*rad2degree).f(), (Heading2*rad2degree).f());
      print( "\ngyr_x: %f, gyr_y: %f, gyr_z, %f", gyr_x.f(), gyr_y.f(), gyr_z.f());
      
      //print("Height: %d", height.f());
      //print("\nm1_pw: %d, m2_pw: %d, m3_pw: %d, m4_pw: %d", m1_pw, m2_pw , m3_pw, m4_pw); 
      
      //print("\nftheta: %f, fphi: %f", (filt_theta*rad2degree).f(), (filt_phi*rad2degree).f());
      
      //print("\nSetpoints: %f, %f", pitch_set_point.f(), roll_set_point.f(), height_set_point.f(), yaw_set_point.f());
      //print("\ncmd word: %x, dir: %x", cmd_word, cmd_direction);
      

      //print("\nmag_x: %f, mag_y: %f, mag_z: %f", mag_x.f(), mag_y.f(), mag_z.f());
      
    }
    while(CNT-t < dt);
    
  }
  
  
  
  
  
  
  return 0;
}

void init_setpoints(void)
{
  //measure height, set height
  height_set_point = fixed(0.15);
  yaw_set_point = 0;
  roll_set_point = 0;
  pitch_set_point = 0;
}


void run_height_sensor(void *par)
{
    int temp;
    fixed phi_internal, theta_internal, height_internal;
    while(1)
    {
        temp = ping_cm_hcsr04(echopin, triggerpin);
        height = fixed(temp)/100; //cm to m conversion
        theta_internal.set(_ftheta);
        phi_internal.set(_fphi);
        height_internal  = height*cosf_cordic(theta_internal)*cosf_cordic(phi_internal);
        //tilt compensation
        _height = height_internal.i();  //output to volatile int
    }        

}

#ifdef _RADIO_
void run_radio(void *par)
{
  ADF7024 radio(rad_CS, rad_MOSI, rad_CLK, rad_MISO);
  rad_status = radio.setup();
  pause(2000);
  pause(100);
  radio.issue_cmd(CMD_PHY_ON);
  pause(100);
  
  rad_status = radio.read_config_reg(0x2B);
  
  char data_in[10];
  char data_out[] = {0x00};
  while(1)
  {
      
    radio.issue_cmd(CMD_PHY_RX);
    pause(100);
    radio.read_packet_reg(data_in);
    rad_status = data_in[0];
    
    radio.set_packet_reg(data_out); //reset first byte to 0, so that a new packet will be properly detected
    //rad_status = radio.read_interrupt_reg();
    if (data_in[0] == 0x01 || data_in[0] == 0x02 || data_in[0] == 0x03)
    {
      packet_count++;
      cmd_word = data_in[0];
      cmd_direction = data_in[1];
    }    
    
    pause(100);
  }
}
#endif

#ifdef _RADIO_
char interpret_radio(void)
{
    bool cmd_bool[8];

    low(comm_pin1);  //reset so that it activates every cycle
    low(comm_pin0);
    
    if (cmd_word == 0x02 || cmd_word == 0x03)
    {
        
      //reset pitch/roll
      pitch_set_point = 0;
      roll_set_point = 0;
      high(comm_pin1);  //indicates comms
    }
    else 
    {
        high(comm_pin1); //indicates comms
       high(comm_pin0);  //indicates incoming direction commands - full control on
      for (int i = 0; i<8; i++)
      {
        cmd_bool[i] = cmd_direction & (1<<i);
      }

    
      if (cmd_bool[0] != cmd_bool[1])  //pitch controller
      {
          
        if (cmd_bool[0])
        {
          //print("  FW True"); //dont print - runs too fast!
          pitch_set_point = fixed(15)*degree2rad;  //5 degrees positive
        }
        else // (cmd_bool[1])
        {
           
          //print("  BW True");
          pitch_set_point = -fixed(15)*degree2rad; //5 degrees negative
        }  
      }   
      else
      {
          
          pitch_set_point = 0;
      }           

      if (cmd_bool[2] != cmd_bool[3])  //roll controller
      {
        if (cmd_bool[2])
        {
          //print("  SL True");
          roll_set_point = -fixed(15)*degree2rad; //5 degrees positive
        }
        else // (cmd_bool[3])
        {  
          //print("  SR True");
          roll_set_point = fixed(15)*degree2rad; //5 degrees negative
        }   
      }
      else
      {
          roll_set_point = 0;
      }   


      if (cmd_bool[4] != cmd_bool[5]) //height controller
      {
        if (cmd_bool[4])
        {
          //print("  YL True");
          height_set_point += fixed(0.05); //5cm degrees addition
          if (height_set_point > 2)
            height_set_point = 2;
        }
        else // (cmd_bool[5])
        {  
          //print("  YR True");
          height_set_point -= fixed(0.05); //5 degrees negative subtraction
           if (height_set_point <0.1)
            height_set_point = fixed(0.1);
        }   
      }

      if (cmd_bool[6] != cmd_bool[7]) //yaw
      {
        if (cmd_bool[6])
        {
          //print("  YL True");
          yaw_set_point += fixed(5)*degree2rad; //5 degrees negative addition
          if (yaw_set_point > fixed(135)*rad2degree)
            yaw_set_point = fixed(135)*rad2degree;
        }
        else // (cmd_bool[7])
        {  
          //print("  YR True");
          yaw_set_point -= fixed(5)*degree2rad; //5 degrees negative subtraction
          if (yaw_set_point < -fixed(135)*rad2degree)
            yaw_set_point = -fixed(135)*rad2degree;
        }
      }
    //cmd_word = 0;
    //cmd_direction = 0;  //reset cmd after processing - prevent double processing ov same command
    }
    return cmd_word;
}
#endif

  
  
inline fixed pwm_clamp(fixed pwm)
{ //clamps the pwm output 
  if (pwm>fixed(1.9)) 
    return fixed(1.9);
  else if (pwm<fixed(1.2)) 
    return fixed(1.2);
  else 
    return pwm;
}  