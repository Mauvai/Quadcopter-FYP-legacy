//See logbook, notebook, and UG-698 REF manual

extern const char config_reg[] = {  //define registers here to avoid cluttering class file
0x07, //0x100  trigger interrupt on preamble or CRC
0x00, //0x101  no phy interrupts
0x00, //0x102  no swm
0x00, //0x103   |
0x00, //0x104   |
0x00, //0x105   |
0x00, //0x106   |
0x00, //0x107   |
0x00, //0x108  \/
0xEC, //0x109    >>
0x44, //0x10A      >> Set radio freq to 865MHz
0x21, //0x10B    >>
0xE8, //0x10C  Radio Profile D
0x03, //0x10D   |
0xFA, //0x10E   |
0x20, //0x10F   |
0x26, //0x110   | 
0x00, //0x111   |
0x00, //0x112  \/
0xC0, //0x113  AGC on, lock after preamble 
0x05, //0x114  16 code/bit PA ramp DEPENDS ON PA LEVEL see data sheet 
0x00, //0x115  Profile D 
0x0B, //0x116  Lock AFC after preamble
0x37, //0x117  Profile D 
0x00, //0x118  No img reject
0x00, //0x119  No img reject
0x40, //0x11A  Enable BB cal, Tx/Rx auto turnaround OFF - this may be changed!
0x0C, //0x11B  no preamble errors allowed 
0x20, //0x11C  No alternative symbol modes
0x14, //0x11D  20 bytes of preamble
0x80, //0x11E  CRC error checking polynomial (0)
0x05, //0x11F  CRC error checking polynomial (1)
0x18, //0x120  24 bit sync word, no errors allowed
0x12, //0x121  sync_byte_0
0x34, //0x122  sync_byte_1
0x56, //0x123  sync_byte_2
0x10, //0x124  Tx base address - change this if want to store multiple packets!
0x10, //0x125  Rx base address - change this if want to store multiple packets!
0xE0, //0x126  Packets are MSB first, fixed length, CRC on, no offset
0x0E, //0x127  10 byte packet - should be 0x0A, but this seems to be necessary - need to find out why!
0x2B, //0x128  Profile D
0x00, //0x129  Reserved
0x00, //0x12A  Reserved
0x2F, //0x12B  Profile D
0x12, //0x12C   | 
0x07, //0x12D  \/
0x03, //0x12E  Pa level  = 0x03 (lowest setting, -20dBm transmit power - highest is 63d)
0x00, //0x12F  Profile D
0x00, //0x130   |
0x00, //0x131   |
0x00, //0x132   |
0x00, //0x133   |
0x00, //0x134   |
0x00, //0x135   |
0x00, //0x136   |
0x00, //0x137   |
0xA7, //0x138  \/  //something to do with timings? can check adf7023 datasheet            
0x00, //0x139  No testmodes enabled
0x04, //0x13A  Reserved
0x00, //0x13B  Reserved
0x00, //0x13C  Reserved
0x00, //0x13D  Reserved
0x00, //0x13E  Reserved
0x00  //0x13F  Reserved
};  //end of config register array 