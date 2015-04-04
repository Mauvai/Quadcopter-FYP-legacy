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
   












  ADF7024 radio1(20, 19, 18, 17);
  pause(100);
  ADF7024 radio2(12, 13, 14, 15);
  pause(100);
  print("Status word, radio1: %x\n", radio1.setup());
  print("Status word, radio2: %x\n", radio2.setup());
    
  
  radio1.set_packet_reg(data);
  pause(100);
  //radio1.read_packet_reg(data_in);
  //radio1.read_interrupt_reg();
  
  
  //print("Config reg 0x13: %2x\n", radio2.read_config_reg(0x13));
  
  //print("\nStatus word: %x\n", radio1.issue_cmd(CMD_PHY_ON));
  radio2.issue_cmd(CMD_PHY_ON);
  radio2.issue_cmd(CMD_PHY_RX);
  
  print("\nStatus word: %x\n", radio1.issue_cmd(CMD_PHY_ON));
  print(  "Status word: %x\n", radio1.issue_cmd(CMD_PHY_TX));
  pause(100);
  print("Status word, radio1: %x\n", radio1.status_word());
  
  radio2.read_interrupt_reg();
  radio2.read_packet_reg(data_in);
  //radio1.read_packet_reg(data_in);
  
  print(  "\nStatus word: %x\n", radio1.issue_cmd(CMD_PHY_ON));






  char cmd = 0;
  bool cmd_bool[8];
  while(1)
  {

    for (int i = 0; i<4; i++)
    {
        cmd |= (input(cmd_pins[i])<<i); //read cmd buttons
    }
    print("\ncmd word: %x", cmd);

    data[0] = 0x01;
    data[1] = cmd;
    radio1.set_packet_reg(data);
    pause(100);


    radio2.issue_cmd(CMD_PHY_RX);
    radio1.issue_cmd(CMD_PHY_TX);
    pause(100);
    radio2.read_packet_reg(data_in);

    if (data_in[0] == 0x01)
    {
      for (int i = 0; i<4; i++)
      {
      cmd_bool[i] = data_in[1] & (1<<i);
      }
    
      if (cmd_bool[0] != cmd_bool[1])
      {
        if (cmd_bool[0])
        print("  FW True");
        if (cmd_bool[1])
        print("  BW True");
      }    
      if (cmd_bool[2] != cmd_bool[3])
      {
        if (cmd_bool[2])
        print("  SL True");
        if (cmd_bool[3])
        print("  SR True");
      }
    }    
        
  

    
    cmd = 0;
    pause(100);
  } 
  
  
}
