/*
  Blank Simple Project.c
  http://learn.parallax.com/propeller-c-tutorials 
*/
#include "simpletools.h"                      // Include simple tools
#include "ADF7024.h"



const int FW = 13;  //forward
const int BW = 15;  //backwards
const int SL = 17;  //Strafe left
const int SR = 19;  //Strafe right


const int UP = 21;  //increase height
const int DN = 23;  //decrease height
const int YL = 25;  //Yaw left
const int YR = 27;  //Yaw right

const int cmd_pins[] = {FW, BW, SL, SR, UP, DN, YL, YR};
char data[10] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};

char data_in[10];
//unsigned char reg = 0x00;

int main()                                    // Main function
{
   



  ADF7024 radio1(4, 6, 8, 10);
  pause(100);
  radio1.setup();
  
  pause(100);
  radio1.issue_cmd(CMD_PHY_ON);
  pause(100);
  
  const int sw1_pin = 12;
  const int sw2_pin = 14;

  char cmd = 0;
  bool cmd_bool[8];
  char data_in[10];
  
  int sw1 = 0;
  int sw2 = 0;
  while(1)
  {
    
    
    sw1 = input(sw1_pin);
    sw2 = input(sw2_pin);
    
    if (!sw1)
    {
        //power off
        data[0] = 0x03;
    }
    else
    {
        //power on
        if (!sw2)
        {
            data[0] = 0x02;  //1200 us pwm - idle
        }    
        else
        {
            for (int i = 0; i<8; i++)
            {
                cmd |= (input(cmd_pins[i])<<i); //read cmd buttons
            }
            data[0] = 0x01;
            data[1] = cmd;
        }         
    }
    
    
    
    
    print("\ncmd word: %x, dir: %x", data[0], data[1]);
    
    
    
    
    radio1.set_packet_reg(data);
    radio1.read_packet_reg(data_in);
    //print("data: %x", data_in[1]);
    pause(100);


    
    radio1.issue_cmd(CMD_PHY_TX);
    cmd = 0;
    pause(100);
  } 
  
  
}
