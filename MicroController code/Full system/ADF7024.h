
#include "simpletools.h"
#include "ADF7024_defs.h"
#include "ADF7024_config_registers.h"




extern const char config_reg[];   //contains data array, found in "ADF7024_config_registers.h"

class ADF7024
{
public:

 
	ADF7024(const int _CS, const int _MOSI, const int _CLK, const int _MISO) : CS(_CS), MOSI(_MOSI), CLK(_CLK), MISO(_MISO)
	{
		high(CS);
		pause(1);
		low(CS);
		shift_in(MISO, CLK, MSBPRE, 8);   //dummy - return is garbage
    	      _status_word = shift_in(MISO, CLK, MSBPRE, 8);   //read from Miso the status word - assumming 0x00 has been shifted out
           //print("Status word: %x\n", _status_word);
            //cant return status word from constructor - if needed, do so manually
    	      high(CS);
	}

	char status_word(void)
	{
            pause(1);
		low(CS);
		_status_word = shift_in(MISO, CLK, MSBPRE, 8);   //dummy - return is garbage
    	      _status_word = shift_in(MISO, CLK, MSBPRE, 8);   //read from Miso the status word - assumming 0x00 has been shifted out
		high(CS);
		pause(1);
		return _status_word;
	}


	char issue_cmd(char CMD)
	{
		pause(1);
		low(CS);                                //select radio
		shift_out(MOSI, CLK, MSBFIRST, 8, CMD);  //shift out only the 8 bit command
		high(CS);                               //deselect radio
		pause(1);
		return status_word();
	}



	char setup()
	{
		low(CS);
            pause(1);
		shift_out(MOSI, CLK, MSBFIRST, 8, 0x19); //SPI write to config registers
		shift_out(MOSI, CLK, MSBFIRST, 8, 0x00); //start address
		for (int i = 0; i<64; i++)
		{
                 pause(1);
			shift_out(MOSI, CLK, MSBFIRST, 8, config_reg[i]);  //write to all 64 config registers
		}
           pause(1);
	     high(CS);
	     pause(1);


		return issue_cmd(CMD_CONFIG_DEV);  //issues cmd and returns status word
	}

	char read_config_reg(char reg)
	{
		low(CS);
		shift_out(MOSI, CLK, MSBFIRST, 8, 0x39);
		shift_out(MOSI, CLK, MSBFIRST, 8, reg);
            shift_in(MISO, CLK, MSBPRE, 8);  //need dummy in which status is returned
		temp = shift_in(MISO, CLK, MSBPRE, 8);  
            high(CS);
            pause(1);
            return temp; 
		//dont need to return status word as spi reads dont affect anything
	}

      char set_packet_reg(char *data, char start_reg = 0x10)
      {
          low(CS);
          shift_out(MOSI, CLK, MSBFIRST, 8, 0x18); //SPI write to packet registers
	    shift_out(MOSI, CLK, MSBFIRST, 8, start_reg); //start address - up to 0x0F are reserved
	    for (int i = 0; i<sizeof(data); i++) //use sizeof because char is one byte
	    {
               pause(1);
               shift_out(MOSI, CLK, MSBFIRST, 8, data[i]);  //write to all packet ram
	    }
          high(CS);
          pause(1);

          return status_word();
      }
      
      char read_packet_reg(char *data_in, char start_reg = 0x10)
      {
          low(CS);
          shift_out(MOSI, CLK, MSBFIRST, 8, 0x38); //SPI write to packet registers
	      shift_out(MOSI, CLK, MSBFIRST, 8, start_reg); //start address - up to 0x0F are reserved
          shift_in(MISO, CLK, MSBPRE, 8);  //need dummy in which status is returned
	    for (int i = 0; i<10; i++)
	    {
               pause(1);
               data_in[i] = shift_in(MISO, CLK, MSBPRE, 8);
               //print("\nByte %d: %2x", i, data_in[i]);
	    }
          high(CS);
          pause(1);
          return status_word();
      }          

      char read_interrupt_reg(void)
      {
          
          //readfrom the interrupt registers - only 2 of the mac (0x36) interrupts are enabled
          //none of the phy interrupts are enabled (0x37), but they set in this register anyway
          low(CS);
          shift_out(MOSI, CLK, MSBFIRST, 8, 0x3B); //SPI write to Aux registers
	    shift_out(MOSI, CLK, MSBFIRST, 8, 0x36); //start address - first interrupt source register
          shift_in(MISO, CLK, MSBPRE, 8);  //need dummy in which status is returned
	    pause(1);
          //print("\nMac interrupts: %2x", shift_in(MISO, CLK, MSBPRE, 8));  //read 2 interrupt registers
          char int1 = shift_in(MISO, CLK, MSBPRE, 8); //the useful interuppts
          pause(1);
          //print("\nPhy interrupts: %2x", shift_in(MISO, CLK, MSBPRE, 8));
          high(CS);
          pause(1);
          return int1;
      }  


private:
	char _status_word;
	char temp;
      
      
      int CS; 
      int MOSI;
      int CLK;
      int MISO;

};