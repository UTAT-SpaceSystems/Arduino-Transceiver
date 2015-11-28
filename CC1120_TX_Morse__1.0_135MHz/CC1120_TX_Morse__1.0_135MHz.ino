#include "SPI.h"
#include "cmd_strobes.h"
#include "registers.h"

//define SPI pins
#define pin_SS 8
#define pin_MOSI 11
#define pin_MISO 12
#define pin_SCK 13
#define WPM 30

#define pin_greenLED 3
#define pin_redLED 5
#define pin_yellowLED 6

//define crystal oscillator frequency to 32MHz
#define f_xosc 32000000;

//declare variables to print status on time intervals
unsigned long previousTime = 0;
unsigned long currentTime = millis();
const int interval = 1000;
bool message[1024] = { 0 };
int message_counter=0;
int position_counter=0;

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
  reg_write(0x0B, B00001100);       //MODCFG_DEV_E: 0x05     set up modulation mode and DEV_E to 5 (see DEV_M register)
  reg_write(0x21, 0x1B);       //FS_CFG: B00010100      set up LO divider to 8 (410.0 - 480.0 MHz band), out of lock detector disabled

  //long long 
  //set symbol rate to 256 symbol per second
  reg_write(0x14, B00100111);       //SRATE_E: 0
  reg_write(0x15, B00000000);       //SRATE_M: 68719
  reg_write(0x16, B00000000);

  //set sync length to 0
  reg_write(0x09, B00000000);

  //disable CRC
  reg_write_bit(0x27,2,0);
  reg_write_bit(0x27,3,0);

  //set preamble
  reg_write(0x0D, 0x00);            //PREAMBLE_CFG1: 0x00    No preamble
  reg_write_bit(0x0E, 5, 0);        //PQT_EN: 0x00           Preamble detection disabled
  
  //TOC_LIMIT
  reg_write_bit2F(0x02, 7, 0);        //TOC_LIMIT: 0x00      Using the low tolerance setting (TOC_LIMIT = 0) greatly reduces system settling times and system power consumption as no preamble bits are needed for bit synchronization or frequency offset compensation (4 bits preamble needed for AGC settling).
  reg_write_bit2F(0x02, 6, 0);        //TOC_LIMIT: 0x00      Using the low tolerance setting (TOC_LIMIT = 0) greatly reduces system settling times and system power consumption as no preamble bits are needed for bit synchronization or frequency offset compensation (4 bits preamble needed for AGC settling).
  
  //set SYNC word
  reg_write_bit(0x08, 6, 0);        //PQT_GATING_EN: 0       PQT gating disabled (preamble not required)

  //set packets
  reg_write_bit(0x12, 6, 1);        //FIFO_EN: 0             FIFO enable set to true
  reg_write_bit(0x13, 6, 0);        //TRANSPARENT_MODE_EN: 0 Disable transparent mode
  reg_write(0x26, 0x00);            //PKT_CFG2: 0x00         set FIFO mode
  reg_write(0x28, 0B01000000);      //PKT_CFG0: 0x00         set fixed packet length
  reg_write(0x2E, 0xFF);            //PKT_LEN: 0xFF          set packet length to 0xFF (max)
  
  //set power level
  reg_write(0x2B, B01111111);       //PA_CFG2: 0x7F          set POWER_RAMP to 64 (output power to 14.5dBm, equation 21)
  
  //frequency offset setting
  reg_write2F(0x0A, 0);             //FREQOFF1: 0x00         set frequency offset to 0
  reg_write2F(0x0B, 0);             //FREQOFF0: 0x00
  
  //Frequency setting
  reg_write2F(0x0C, 0x65);          //FREQ2: 0x6C            set frequency to 434MHz (sets Vco, see equation from FREQ2 section of user guide)
  reg_write2F(0x0D, 0x40);          //FREQ1: 0x80
  reg_write2F(0x0E, 0x00);          //FREQ0: 0x00

  
  //************************************************************ MAIN PROGRAM *****************************************************//
   //turn on green LED when main code starts running
   Serial.println("\n*** Beginning of Main Program ***\n");
   
   //turn on green LED on breadboard and setup other LED pins as outputs
   pinMode(pin_greenLED, OUTPUT);
   pinMode(pin_redLED, OUTPUT);
   pinMode(pin_yellowLED, OUTPUT);
   digitalWrite(pin_greenLED, HIGH);

char text[] = "HELLO WORLD";
  message2morse(text, message);
  for (int i=0;i<1024;i++)
    Serial.print(message[i]);
    Serial.println(); 

 
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
    if(message_counter==8)
       message_counter=0;
       
     for(int i=0x00; i<128; i++){
            dir_FIFO_write(i, message[i+message_counter*128]);
            Serial.print(message[i+message_counter*128]);
         }
      Serial.println();
    
    reg_write2F(0xD3, 0x00);
    reg_write2F(0xD5, 0x7F);
   
    cmd_str(STX, 1);
    message_counter=message_counter+1;
    
  delay(2780);
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
    /*cmd_str(SIDLE, 1);
    cmd_str(SFTX, 1);
    reg_write2F(0xD3, 0x00);
    reg_write2F(0xD5, 0x7F);
    cmd_str(STX, 1);*/
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

//convert one charactor to morse 'code'
void char2code(char data, uint8_t *Length, uint8_t *code1,uint8_t *code2,uint8_t *code3, uint8_t *code4, uint8_t *code5, uint8_t *code6, uint8_t *code7,
      uint8_t *code8, uint8_t *code9, uint8_t *code10, uint8_t *code11, uint8_t *code12, uint8_t *code13, uint8_t *code14, uint8_t *code15, uint8_t *code16){

  switch (data){
case ' ':
*code1 = 0;
*code2 = 0;
*code3 = 0;
*Length = 21;
break;

case 'A':
*code1 = 255;
*code2 = 113;
*Length = 15;
break;

case 'B':
*code1 = 199;
*code2 = 113;
*code3 = 252;
*code4 = 7;
*Length = 27;
break;

case 'C':
*code1 = 199;
*code2 = 127;
*code3 = 28;
*code4 = 255;
*code5 = 1;
*Length = 33;
break;

case 'D':
*code1 = 199;
*code2 = 241;
*code3 = 31;
*Length = 21;
break;

case 'E':
*code1 = 7;
*Length = 3;
break;

case 'F':
*code1 = 199;
*code2 = 127;
*code3 = 28;
*code4 = 7;
*Length = 27;
break;

case 'G':
*code1 = 199;
*code2 = 127;
*code3 = 252;
*code4 = 7;
*Length = 27;
break;

case 'H':
*code1 = 199;
*code2 = 113;
*code3 = 28;
*Length = 21;
break;

case 'I':
*code1 = 199;
*code2 = 1;
*Length = 9;
break;

case 'J':
*code1 = 255;
*code2 = 241;
*code3 = 31;
*code4 = 255;
*code5 = 113;
*Length = 39;
break;

case 'K':
*code1 = 255;
*code2 = 241;
*code3 = 31;
*Length = 21;
break;

case 'L':
*code1 = 199;
*code2 = 241;
*code3 = 31;
*code4 = 7;
*Length = 27;
break;

case 'M':
*code1 = 255;
*code2 = 241;
*code3 = 31;
*Length = 21;
break;

case 'N':
*code1 = 199;
*code2 = 127;
*Length = 15;
break;

case 'O':
*code1 = 255;
*code2 = 241;
*code3 = 31;
*code4 = 255;
*code5 = 1;
*Length = 33;
break;

case 'P':
*code1 = 199;
*code2 = 127;
*code3 = 252;
*code4 = 199;
*code5 = 1;
*Length = 33;
break;

case 'Q':
*code1 = 255;
*code2 = 113;
*code3 = 252;
*code4 = 199;
*code5 = 127;
*Length = 39;
break;

case 'R':
*code1 = 199;
*code2 = 127;
*code3 = 28;
*Length = 21;
break;

case 'S':
*code1 = 199;
*code2 = 113;
*Length = 15;
break;

case 'T':
*code1 = 255;
*code2 = 1;
*Length = 9;
break;

case 'U':
*code1 = 255;
*code2 = 113;
*code3 = 28;
*Length = 21;
break;

case 'V':
*code1 = 255;
*code2 = 113;
*code3 = 28;
*code4 = 7;
*Length = 27;
break;

case 'W':
*code1 = 255;
*code2 = 241;
*code3 = 31;
*code4 = 7;
*Length = 27;
break;

case 'X':
*code1 = 255;
*code2 = 113;
*code3 = 28;
*code4 = 255;
*code5 = 1;
*Length = 33;
break;

case 'Y':
*code1 = 255;
*code2 = 241;
*code3 = 31;
*code4 = 199;
*code5 = 127;
*Length = 39;
break;

case 'Z':
*code1 = 199;
*code2 = 241;
*code3 = 31;
*code4 = 255;
*code5 = 1;
*Length = 33;
break;

case '0':
*code1 = 255;
*code2 = 241;
*code3 = 31;
*code4 = 255;
*code5 = 241;
*code6 = 31;
*code7 = 255;
*code8 = 1;
*Length = 57;
break;

case '1':
*code1 = 255;
*code2 = 241;
*code3 = 31;
*code4 = 255;
*code5 = 241;
*code6 = 31;
*code7 = 7;
*Length = 51;
break;

case '2':
*code1 = 255;
*code2 = 241;
*code3 = 31;
*code4 = 255;
*code5 = 113;
*code6 = 28;
*Length = 45;
break;

case '3':
*code1 = 255;
*code2 = 241;
*code3 = 31;
*code4 = 199;
*code5 = 113;
*Length = 39;
break;

case '4':
*code1 = 255;
*code2 = 113;
*code3 = 28;
*code4 = 199;
*code5 = 1;
*Length = 33;
break;

case '5':
*code1 = 199;
*code2 = 113;
*code3 = 28;
*code4 = 7;
*Length = 27;
break;

case '6':
*code1 = 199;
*code2 = 113;
*code3 = 28;
*code4 = 255;
*code5 = 1;
*Length = 33;
break;

case '7':
*code1 = 199;
*code2 = 113;
*code3 = 252;
*code4 = 199;
*code5 = 127;
*Length = 39;
break;

case '8':
*code1 = 199;
*code2 = 241;
*code3 = 31;
*code4 = 255;
*code5 = 241;
*code6 = 31;
*Length = 45;
break;

case '9':
*code1 = 199;
*code2 = 127;
*code3 = 252;
*code4 = 199;
*code5 = 127;
*code6 = 252;
*code7 = 7;
*Length = 51;
break;





  default:
    break;

  }
}
//convert message to morse code
void message2morse(char message[], bool* morse)
{
  uint8_t current_position = 10;
  uint8_t length = 0;
  uint8_t code = 0;
  uint8_t code2 = 0;
  uint8_t code3 = 0;
  uint8_t code4 = 0;
  uint8_t code5 = 0;
  uint8_t code6 = 0;
  uint8_t code7 = 0;
  uint8_t code8 = 0;
  uint8_t code9 = 0;
  uint8_t code10 = 0;
  uint8_t code11 = 0;
  uint8_t code12 = 0;
  uint8_t code13 = 0;
  uint8_t code14 = 0;
  uint8_t code15 = 0;
  uint8_t code16 = 0;

  for (int i = 0; message[i] != NULL; i++)
  {
    char2code(message[i], &length, &code,&code2,&code3,&code4,&code5,&code6,&code7,&code8,&code9,&code10,&code11,&code12,&code13,&code14,&code15,&code16);
    code2morse(&current_position, code,code2,code3,code4, code5,code6,code7,code8,code9, code10,code11,code12,code13,code14, code15,code16, morse, length);

    Serial.print(code);
    Serial.print(" ");
    Serial.print(code2);
    Serial.print(" ");
    Serial.print(length);
    Serial.print(" ");
    Serial.print(current_position);
    Serial.println();
    int space_between_letter=9;
    
    if(current_position>(127-space_between_letter)){
      position_counter=position_counter+1;
      current_position=current_position-(128-space_between_letter);
    }
    else{
      current_position += space_between_letter;
    }
    Serial.print(length);
    Serial.print(" ");
    Serial.print(current_position);
    Serial.println();
  }

};
void code2morse(uint8_t *current, uint8_t code, uint8_t code2, uint8_t code3, uint8_t code4, uint8_t code5, uint8_t code6, uint8_t code7, uint8_t code8, uint8_t code9, uint8_t code10, uint8_t code11, 
              uint8_t code12, uint8_t code13, uint8_t code14, uint8_t code15, uint8_t code16, bool* morse, uint8_t length)
{
  uint8_t bits[128] = { 0 };
  
  uint8_t remain_bit = (length % 8) == 0 ? 8 : (length % 8);
if (length>120)
  {
    for (uint8_t k = remain_bit-1; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code16 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+120] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code15 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+112] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code14 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+104] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code13 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+96] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code12 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+88] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code11 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+80] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code10 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+72] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code9 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+64] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code8 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+56] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code7 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+48] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code6 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+40] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code5 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+32] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code4 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+24] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code3 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+16] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code2 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+8] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code & mask;
      uint8_t thebit = masked_n >> k;
      bits[k] = thebit;
    }
  }if (length>112)
  {
    for (uint8_t k = remain_bit-1; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code15 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+112] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code14 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+104] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code13 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+96] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code12 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+88] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code11 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+80] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code10 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+72] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code9 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+64] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code8 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+56] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code7 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+48] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code6 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+40] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code5 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+32] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code4 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+24] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code3 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+16] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code2 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+8] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code & mask;
      uint8_t thebit = masked_n >> k;
      bits[k] = thebit;
    }
  }
  else if (length>104)
  {
    for (uint8_t k = remain_bit-1; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code14 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+104] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code13 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+96] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code12 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+88] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code11 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+80] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code10 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+72] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code9 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+64] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code8 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+56] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code7 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+48] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code6 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+40] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code5 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+32] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code4 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+24] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code3 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+16] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code2 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+8] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code & mask;
      uint8_t thebit = masked_n >> k;
      bits[k] = thebit;
    }
  }
  else if (length>96)
  {
    for (uint8_t k = remain_bit-1; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code13 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+96] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code12 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+88] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code11 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+80] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code10 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+72] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code9 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+64] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code8 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+56] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code7 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+48] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code6 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+40] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code5 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+32] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code4 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+24] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code3 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+16] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code2 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+8] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code & mask;
      uint8_t thebit = masked_n >> k;
      bits[k] = thebit;
    }
  }
  else if (length>88)
  {
    for (uint8_t k = remain_bit-1; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code12 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+88] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code11 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+80] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code10 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+72] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code9 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+64] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code8 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+56] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code7 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+48] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code6 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+40] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code5 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+32] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code4 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+24] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code3 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+16] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code2 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+8] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code & mask;
      uint8_t thebit = masked_n >> k;
      bits[k] = thebit;
    }
  }
  else if (length>80)
  {
    for (uint8_t k = remain_bit-1; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code11 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+80] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code10 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+72] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code9 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+64] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code8 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+56] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code7 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+48] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code6 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+40] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code5 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+32] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code4 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+24] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code3 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+16] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code2 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+8] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code & mask;
      uint8_t thebit = masked_n >> k;
      bits[k] = thebit;
    }
  }
  else if (length>72)
  {
    for (uint8_t k = remain_bit-1; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code10 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+72] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code9 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+64] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code8 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+56] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code7 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+48] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code6 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+40] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code5 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+32] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code4 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+24] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code3 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+16] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code2 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+8] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code & mask;
      uint8_t thebit = masked_n >> k;
      bits[k] = thebit;
    }
  }
  else if (length>64)
  {
    for (uint8_t k = remain_bit-1; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code9 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+64] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code8 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+56] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code7 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+48] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code6 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+40] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code5 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+32] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code4 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+24] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code3 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+16] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code2 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+8] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code & mask;
      uint8_t thebit = masked_n >> k;
      bits[k] = thebit;
    }
  }
  else if (length>56)
  {
    for (uint8_t k = remain_bit-1; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code8 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+56] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code7 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+48] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code6 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+40] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code5 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+32] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code4 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+24] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code3 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+16] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code2 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+8] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code & mask;
      uint8_t thebit = masked_n >> k;
      bits[k] = thebit;
    }
  }
  else if (length>48)
  {
    for (uint8_t k = remain_bit-1; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code7 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+48] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code6 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+40] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code5 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+32] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code4 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+24] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code3 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+16] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code2 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+8] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code & mask;
      uint8_t thebit = masked_n >> k;
      bits[k] = thebit;
    }
  }
  
else if (length>40)
  {
    for (uint8_t k = remain_bit-1; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code6 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+40] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code5 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+32] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code4 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+24] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code3 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+16] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code2 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+8] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code & mask;
      uint8_t thebit = masked_n >> k;
      bits[k] = thebit;
    }
  }
  else if (length>32)
  {
    for (uint8_t k = remain_bit-1; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code5 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+32] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code4 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+24] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code3 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+16] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code2 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+8] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code & mask;
      uint8_t thebit = masked_n >> k;
      bits[k] = thebit;
    }
  }
  else if (length>24)
  {
    for (uint8_t k = remain_bit-1; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code4 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+24] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code3 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+16] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code2 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+8] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code & mask;
      uint8_t thebit = masked_n >> k;
      bits[k] = thebit;
    }
  }
  else if (length>16)
  {
    for (uint8_t k = remain_bit-1; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code3 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+16] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code2 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+8] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code & mask;
      uint8_t thebit = masked_n >> k;
      bits[k] = thebit;
    }
  }
  else if (length>8)
  {
    for (uint8_t k = remain_bit-1; k>=0 && k!=255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code2 & mask;
      uint8_t thebit = masked_n >> k;
      bits[k+8] = thebit;
    }
    for (uint8_t k = 7; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code & mask;
      uint8_t thebit = masked_n >> k;
      bits[k] = thebit;
    }
  }
  else
  {
    for (uint8_t k = remain_bit - 1; k>=0 && k != 255; k--) {
      uint8_t mask = 1 << k;
      uint8_t masked_n = code & mask;
      uint8_t thebit = masked_n >> k;
      bits[k] = thebit;
    }
  }


  for (uint8_t i = 0; i<length; i++)
  {
    morse[*current + i+128*position_counter] |= bits[length-1-i];
  }
  //*current += length;

  if(*current>(127-length)){
      position_counter=position_counter+1;
      *current=*current-(128-length);
    }
    else{
      *current += length;
    }

}
