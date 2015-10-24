#include "SPI.h"
#include "cmd_strobes.h"
#include "registers.h"

//define SPI pins
#define pin_SS 8
#define pin_MOSI 11
#define pin_MISO 12
#define pin_SCK 13

#define pin_greenLED 3
#define pin_redLED 5
#define pin_yellowLED 6

//define crystal oscillator frequency to 32MHz
#define f_xosc 32000000;

//declare variables to print status on time intervals
unsigned long previousTime = 0;
unsigned long currentTime = millis();
const int interval = 1000;

//************************************************************ SETUP ********************************************************//

void setup() {                
  // initialize the digital pin as an output.
  Serial.begin(9600); 
  Serial.print("hello world");
  
  //Configure SPI
  pinMode(pin_SS, OUTPUT);
  set_CSn(1);
  delay(1000);
  SPI.begin();
  
  //This is the correct setting for the CC1120
  SPI.setBitOrder(MSBFIRST);
  
  //**********CONFIGURE CHIP***************//
      byte test;
      byte value;
      
      //Set SS
      set_CSn(0);
      
      while(digitalRead(pin_MISO))
        delay(100);
     
      //RESET 
      cmd_str(SRES, 0);             //SRES                  reset chip
      
      //Reset RX FIFO
      cmd_str(SFRX, 1);             //SFRX                  flush RX FIFO
      
      //Reset RX FIFO
      cmd_str(SFTX, 1);             //SFTX                  flush TX FIFO
  //********** END CONFIGURE CHIP**********//
  
  //**************SET UP TX****************//

 
  
  //high performance settings
  reg_write2F(0x12, 0x00);          //FS_DIG1: 0x00         Frequency Synthesizer Digital Reg. 1
  reg_write2F(0x13, 0x5F);          //FS_DIG0: 0x5F         Frequency Synthesizer Digital Reg. 0
  reg_write2F(0x16, 0x40);          //FS_CAL1: 0x40         Frequency Synthesizer Calibration Reg. 1
  reg_write2F(0x17, 0x0E);          //FS_CAL0: 0x0E         Frequency Synthesizer Calibration Reg. 0
  reg_write2F(0x19, 0x03);          //FS_DIVTWO: 0x03       Frequency Synthesizer Divide by 2
  reg_write2F(0x1B, 0x33);          //FS_DSM0: 0x33         FS Digital Synthesizer Module Configuration Reg. 0
  reg_write2F(0x1D, 0x17);          //FS_DVCO: 0x17         Frequency Synthesizer Divider Chain Configuration ..
  reg_write2F(0x1F, 0x50);          //FS_PFD: 0x50          Frequency Synthesizer Phase Frequency Detector Con..
  reg_write2F(0x20, 0x6E);          //FS_PRE: 0x6E          Frequency Synthesizer Prescaler Configuration
  reg_write2F(0x21, 0x14);          //FS_REG_DIV_CML: 0x14  Frequency Synthesizer Divider Regulator Configurat..
  reg_write2F(0x22, 0xAC);          //FS_SPARE: 0xAC        Set up Frequency Synthesizer Spare
  reg_write2F(0x27, 0xB4);          //FS_VCO0: 0xB4         FS Voltage Controlled Oscillator Configuration Reg..
  reg_write2F(0x32, 0x0E);          //XOSC5: 0x0E           Crystal Oscillator Configuration Reg. 5
  reg_write2F(0x36, 0x03);          //XOSC1: 0x03           Crystal Oscillator Configuration Reg. 0
  
  //For test purposes only, use values from SmartRF for all bits
  reg_write2F(0x00, 0x04);          //
  reg_write2F(0x03, 0x00);          //
  reg_write2F(0x09, 0x00);          //
  reg_write2F(0x0F, 0x02);          //
  reg_write2F(0x10, 0xA6);          //
  reg_write2F(0x11, 0x04);          //
  reg_write2F(0x1A, 0x00);          //
  reg_write2F(0x1C, 0xFF);          //
  reg_write2F(0x1E, 0x00);          //
  reg_write2F(0x20, 0x6E);          //
  reg_write2F(0x23, 0x14);          //
  reg_write2F(0x24, 0x00);          //
  reg_write2F(0x28, 0x00);          //
  reg_write2F(0x29, 0x02);          //
  reg_write2F(0x2A, 0x00);          //
  reg_write2F(0x2B, 0x00);          //
  reg_write2F(0x2C, 0x10);          //
  reg_write2F(0x2D, 0x00);          //
  reg_write2F(0x2E, 0x00);          //
  reg_write2F(0x2F, 0x01);          //
  reg_write2F(0x30, 0x01);          //
  reg_write2F(0x31, 0x01);          //
  reg_write2F(0x33, 0xA0);          //
  reg_write2F(0x34, 0x03);          //
  reg_write2F(0x38, 0x00);          //
  reg_write2F(0x39, 0x00);          //
  reg_write2F(0x68, 0x00);          //
  reg_write2F(0x7B, 0x00);          //
  reg_write2F(0x7C, 0x3F);          //
  reg_write2F(0x96, 0x00);          //
  reg_write2F(0x97, 0x00);          //
  reg_write2F(0x98, 0x00);          //
  reg_write2F(0x99, 0x00);          //
  reg_write2F(0x9A, 0x00);          //
  reg_write2F(0x9B, 0x0B);          //
  reg_write2F(0x9C, 0x40);          //
  reg_write2F(0x9D, 0x00);          //
  reg_write2F(0x9E, 0x00);          //
  reg_write2F(0x9F, 0x3C);          //
  reg_write2F(0xA0, 0x00);          //
  
  //For test purposes only, use values from SmartRF for some bits
  reg_write(0x08, 0x0A);            //*Changed on line 152
  reg_write(0x13, 0x0D);            //
  reg_write(0x26, 0x04);            //*Changed on line 144
  reg_write(0x28, 0x00);            //*Changed on line 145
  reg_write(0x2B, 0x7F);            //
  reg_write2F(0x05, 0x00);          //
  reg_write2F(0x14, 0x00);          //
  reg_write2F(0x26, 0x00);          //
  reg_write2F(0x35, 0x04);          //
  reg_write2F(0x7A, 0xD1);          //
  reg_write2F(0x8D, 0x01);          //
  
  //High performance TX
  reg_write(0x08, 0x0B);            //
  reg_write(0x0C, 0x1C);            //
  reg_write(0x10, 0x00);            //
  reg_write(0x11, 0x04);            //
  reg_write(0x13, 0x05);            //
  reg_write(0x1C, 0xA9);            //
  reg_write(0x1D, 0xCF);            //
  reg_write(0x1E, 0x00);            //
  reg_write(0x20, 0x03);            //
  reg_write(0x2E, 0x00);            //
  reg_write2F(0x00, 0x00);          //
  
  //modulation and freq deviation settings
  reg_write(0x0A, B01001000);       //DEVIATION_M: 0x48      set DEV_M to 72 which sets freq deviation to 20.019531kHz (with DEV_M=5)
  reg_write(0x0B, B00000101);       //MODCFG_DEV_E: 0x05     set up modulation mode and DEV_E to 5 (see DEV_M register)
  reg_write(0x21, B00000100);       //FS_CFG: B00010100      set up LO divider to 8 (410.0 - 480.0 MHz band), out of lock detector disabled
  
  //set preamble
  reg_write(0x0D, 0x00);            //PREAMBLE_CFG1: 0x00    No preamble
  reg_write_bit(0x0E, 5, 0);        //PQT_EN: 0x00           Preamble detection disabled
  
  //TOC_LIMIT
  reg_write_bit2F(0x02, 7, 0);        //TOC_LIMIT: 0x00      Using the low tolerance setting (TOC_LIMIT = 0) greatly reduces system settling times and system power consumption as no preamble bits are needed for bit synchronization or frequency offset compensation (4 bits preamble needed for AGC settling).
  reg_write_bit2F(0x02, 6, 0);        //TOC_LIMIT: 0x00      Using the low tolerance setting (TOC_LIMIT = 0) greatly reduces system settling times and system power consumption as no preamble bits are needed for bit synchronization or frequency offset compensation (4 bits preamble needed for AGC settling).
  
  //set SYNC word
  reg_write_bit(0x08, 6, 0);        //PQT_GATING_EN: 0       PQT gating disabled (preamble not required)
  reg_write(0x09, B00010111);       //SYNC_CFG0: B00010111   32 bit SYNC word. Bit error qualifier disabled. No check on bit errors
  reg_write(0x04, 0x93);            //SYNC3: 0x93            Set SYNC word bits 31:24
  reg_write(0x05, 0x0B);            //SYNC2: 0x0B            Set SYNC word bits 23:16
  reg_write(0x06, 0x51);            //SYNC1: 0x51            Set SYNC word bits 15:8
  reg_write(0x07, 0xDE);            //SYNC0: 0xDE            Set SYNC word bits 7:0
  
  //set packets
  reg_write_bit(0x12, 6, 1);        //FIFO_EN: 0             FIFO enable set to true
  reg_write_bit(0x13, 6, 0);        //TRANSPARENT_MODE_EN: 0 Disable transparent mode
  reg_write(0x26, 0x00);            //PKT_CFG2: 0x00         set FIFO mode
  reg_write(0x28, 0x00);            //PKT_CFG0: 0x00         set fixed packet length
  reg_write(0x2E, 0xFF);            //PKT_LEN: 0xFF          set packet length to 0xFF (max)
  
  //set power level
  reg_write(0x2B, B01111111);       //PA_CFG2: 0x7F          set POWER_RAMP to 64 (output power to 14.5dBm, equation 21)
  
  //frequency offset setting
  reg_write2F(0x0A, 0);             //FREQOFF1: 0x00         set frequency offset to 0
  reg_write2F(0x0B, 0);             //FREQOFF0: 0x00
  
  //Frequency setting
  reg_write2F(0x0C, 0x6C);          //FREQ2: 0x6C            set frequency to 434MHz (sets Vco, see equation from FREQ2 section of user guide)
  reg_write2F(0x0D, 0x80);          //FREQ1: 0x80
  reg_write2F(0x0E, 0x00);          //FREQ0: 0x00

  //************************************************************ MAIN PROGRAM *****************************************************//
   //turn on green LED when main code starts running
   Serial.println("\n*** Beginning of Main Program ***\n");
   
   //turn on green LED on breadboard and setup other LED pins as outputs
   pinMode(pin_greenLED, OUTPUT);
   pinMode(pin_redLED, OUTPUT);
   pinMode(pin_yellowLED, OUTPUT);
   digitalWrite(pin_greenLED, HIGH);
   
   //Fill the TX FIFO with a message to send periodically
   int message[0x100];
   for(int i=0x00; i<0x100; i++)
     message[i]=0x41;
     
//   message[128] = 0x48;  // This puts "HELLO!" at 0x80.
//   message[129] = 0x45;
//   message[130] = 0x4C;
//   message[131] = 0x4C;
//   message[132] = 0x4F;
//   message[133] = 0x21;  
     
   //fill the TX FIFO
//      
   for(int i=0x00; i<0x100; i++)
   dir_FIFO_write(i, message[i]);
   
   //set up TX FIFO pointers
   reg_write2F(0xD3, 0x00);            //set TX FIRST to 0
   reg_write2F(0xD5, 0x7F);            //set TX LAST to 0xFF (maximum)
   
   //strobe commands to start TX
   cmd_str(SCAL, 1);                   //calibrate frequency synthesizer
   delay(250);
   cmd_str(SAFC, 1);
   delay(250);
   cmd_str(STX, 1);                    //put in TX mode
}
   
//************************************************************ MAIN LOOP ********************************************************//
void loop() {
  monitor_LEDs();
  input_cmd_str();
  p_status_interval();
  byte TX_FIRST = reg_read2F(0xD3);
  
  //when TX_FIRST reaches the end, reset
  if(TX_FIRST>0x7E){
    Serial.println("\n**FIFO RESET**");
    cmd_str(SIDLE, 1);
    reg_write2F(0xD3, 0x00);
    reg_write2F(0xD5, 0x7F);
    cmd_str(STX, 1);
    Serial.println("\n");
  }
}



//************************************************************NEW READ AND WRITE FUNCTIONS***************************************//




//************************************************************ FUNCTION DECLARATION**********************************************//


//write to register address addr with data
byte reg_read(byte addr){
  
    addr = addr + B10000000; //add the read bit
    SPI.transfer(addr); //send desired address
    return SPI.transfer(0); //read back

}

//reads register in extended memory space
byte reg_read2F(byte addr){
    SPI.transfer(B10101111); //address extension command
    SPI.transfer(addr); //send the desired address
    return SPI.transfer(0); //read back
}

//write to register address addr with data
byte reg_write(byte addr, byte data){
  
    SPI.transfer(addr); //send desired address
    return SPI.transfer(data); //send desired address}
	
}

//rwrites to register in extended memory space
byte reg_write2F(byte addr, byte data){
    SPI.transfer(B00101111); //address extension command
    SPI.transfer(addr); //send desired address
    return SPI.transfer(data); //send desired address}
}

//writes status information to variables in main loop
void get_status(bool *CHIP_RDYn, byte *state){
  
  byte status_b = cmd_str(SNOP, 0);  
  *CHIP_RDYn = (status_b/128)%2; //7th bit (reading backwards)
  *state = ((status_b/64)%2)*4 + ((status_b/32)%2)*2 + (status_b/16)%2; //6th-4th bits (reading backwards)
  return;
}

//parses chip status byte
void p_chip_status(){
  byte status_b = cmd_str(SNOP, 0);
  byte CHIP_RDYn = (status_b/128)%2; //7th bit (reading backwards)
  byte state = ((status_b/64)%2)*4 + ((status_b/32)%2)*2 + (status_b/16)%2; //6th-4th bits (reading backwards)
  
  Serial.print("Chip status: ");
  
  if(CHIP_RDYn)
    Serial.print("NOT READY (1), ");
  else
    Serial.print("READY (0), ");
  
  //parses state  
  if(state == B000)
    Serial.println("IDLE (000)");
  else if(state == B001)
    Serial.println("RX (001)");
  else if(state == B010)
    Serial.println("TX (010)");
  else if(state == B011)
    Serial.println("FSTXON (011)");
  else if(state == B100)
    Serial.println("CALIBRATE (100)");
  else if(state == B101)
    Serial.println("SETTLING (101)");
  else if(state == B110)
    Serial.println("RX FIFO ERROR (110)");
  else if(state == B111)
    Serial.println("TX FIFO ERROR (111)");  
  
  return;
}

//send command strobe and print info on command strobe if Print bit is true
byte cmd_str(byte addr, bool Print){
  
  //prints command strobe executed if applicable
  if(Print){
    Serial.print("Command strobe(");
    if(addr == SRES)
      Serial.println("0x30): SRES");
    else if(addr == SFSTXON)  
      Serial.println("0x31): SFSTXON");
    else if(addr == SXOFF)
      Serial.println("0x32): SXOFF");
    else if(addr == SCAL)
      Serial.println("0x33): SCAL");
    else if(addr == SRX)
      Serial.println("0x34): SRX");
    else if(addr == STX)
      Serial.println("0x35): STX");
    else if(addr == SIDLE)
      Serial.println("0x36): SIDLE");
    else if(addr == SAFC)
      Serial.println("0x37): SAFC");
    else if(addr == SWOR)
      Serial.println("0x38): SWOR");
    else if(addr == SPWD)
      Serial.println("0x39): SPWD");
    else if(addr == SFRX)
      Serial.println("0x3A): SFRX");
    else if(addr == SFTX)
      Serial.println("0x3B): SFTX");
    else if(addr == SWORRST)
      Serial.println("0x3C): SWORRST");
    else if(addr == SNOP)
      Serial.println("0x3C): SNOP");
  }
    
  return SPI.transfer(addr);
}

//when Serial.available()==0, checks for a command strobe and executes it
void get_cmd_str(){ 
  String cmd_str_in = ""; //set input to empty
  char letter;  
  
  do{  //continue reading input until nothing is input  
    letter = Serial.read();  //set as char
    cmd_str_in += letter;
  }while(Serial.available());
  
  if(cmd_str_in == "SRES")
    cmd_str(SRES, 1);
  else if(cmd_str_in == "SFSTXON")
    cmd_str(SFSTXON, 1);
  else if(cmd_str_in == "SCAL")
    cmd_str(SCAL, 1);
  else if(cmd_str_in == "SRX")
    cmd_str(SRX, 1);
  else if(cmd_str_in == "STX")
    cmd_str(STX, 1);
  else if(cmd_str_in == "SIDLE")
    cmd_str(SIDLE, 1);
  else if(cmd_str_in == "SAFC")
    cmd_str(SAFC, 1);
  else if(cmd_str_in == "SWOR")
    cmd_str(SWOR, 1);
  else if(cmd_str_in == "SPWD")
    cmd_str(SPWD, 1);
  else if(cmd_str_in == "SFRX")
    cmd_str(SFRX, 1);
  else if(cmd_str_in == "SFTX")
    cmd_str(SFTX, 1);
  else if(cmd_str_in == "SWORRST")
    cmd_str(SWORRST, 1);
  else if(cmd_str_in == "SNOP")
    cmd_str(SNOP, 1);
    
  return;
}

//reads FIFO using direct access
byte dir_FIFO_read(byte addr){
  SPI.transfer(B10111110); //direct FIFO read address
  SPI.transfer(addr); //send desired address
  return SPI.transfer(0); //read back
}

//writes in FIFO using direct access
byte dir_FIFO_write(byte addr, byte data){
  SPI.transfer(B00111110); //direct FIFO write address
  SPI.transfer(addr); //send desired FIFO address
  return SPI.transfer(data); //send desired data
}

//sets chip select to either LOW or HIGH
void set_CSn(bool state){
  if(state)
    digitalWrite(pin_SS, HIGH);
  else
    digitalWrite(pin_SS, LOW);
}

//prints the value of each register
void p_all_reg(){
  
  for (int i = 0x00; i<=0x2E; i++){
   Serial.print("0x");
   if(reg_read(i)<0x10) {Serial.print("0");}
   Serial.println(reg_read(i), HEX);
 }
 
 for (int i = 0x00; i<=0x39; i++){
   Serial.print("0x");
   if(reg_read2F(i)<0x10) {Serial.print("0");}
   Serial.println(reg_read2F(i), HEX);
 }
 
 for (int i = 0x64; i<=0xA0; i++){
   Serial.print("0x");
   if(reg_read2F(i)<0x10) {Serial.print("0");}
   Serial.println(reg_read2F(i), HEX);
 }
 
 for (int i = 0xD2; i<=0xD9; i++){
   Serial.print("0x");
   if(reg_read2F(i)<0x10) {Serial.print("0");}
   Serial.println(reg_read2F(i), HEX);
 }
 
 return;
}

//turns on LEDs for different situations
void monitor_LEDs(){
  bool CHIP_RDYn;
  byte state;
  
  //red LED goes on if there is an error
  get_status(&CHIP_RDYn, &state);
  if(CHIP_RDYn || state==B110 || state==B111){
    digitalWrite(pin_redLED, HIGH);
    Serial.println("**FIFO RESET**");
    cmd_str(SIDLE, 1);
    cmd_str(SFTX, 1);
    reg_write2F(0xD3, 0x00);
    reg_write2F(0xD5, 0x7F);
    cmd_str(STX, 1);
    }
  else
    digitalWrite(pin_redLED, LOW);
  
  if(state==B001 || state==B010)
    digitalWrite(pin_yellowLED, HIGH);
  else
    digitalWrite(pin_yellowLED, LOW);
}

//uses the get_cmd_str() function to check for an input command strobe
void input_cmd_str(){
  //inputs command strobes from serial monitor
  if(Serial.available()){
    delay(100);          //delay so that serial input doesn't get erased
    get_cmd_str();       //takes serial input and executes command strobe
 }
}

//uses the p_chip_status() function to print status on regular time intervals
void p_status_interval(){
//print status on time intervals
 currentTime=millis();
 if(currentTime-previousTime>=interval){
   p_chip_status();
   byte TX_FIRST = reg_read2F(0xD3);
   byte TX_LAST = reg_read2F(0xD5);
   Serial.print("TX_FIRST: ");
   Serial.println(TX_FIRST, HEX);
   Serial.print("TX_LAST: ");
   Serial.println(TX_LAST, HEX);
   previousTime=currentTime;
 }
}


//changes the nth bit in register 'reg' to data
void reg_write_bit(int reg, int n, int data){
  int old_value = reg_read(reg);
  int power = pow(2, n);
  int new_value = old_value-(old_value%(2*power)) + data*power + old_value%power;
  reg_write(reg, new_value);
  return;
}

//changes the nth bit in register 'reg' to data (extended register space)
void reg_write_bit2F(int reg, int n, int data){
  int old_value = reg_read2F(reg);
  int power = pow(2, n);
  int new_value = old_value-(old_value%(power)) + data*power + old_value%power;
  reg_write2F(reg, new_value);
  return;
}
