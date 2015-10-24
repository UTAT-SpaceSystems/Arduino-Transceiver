

#include "SPI.h"
#include "cmd_strobes.h"
#include "registers.h"

//define SPI pins
#define pin_SS 10
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
  
  //begin serial
  Serial.begin(9600); 
  
  //Configure SPI
  pinMode(pin_SS, OUTPUT);
  
  // initialize the digital pin as an output.
  SPI.begin();
  
  //This is the correct setting for the CC1120
  SPI.setBitOrder(MSBFIRST);
      
      int msg = 0;
      
      //Set SS
      set_CSn(0);
      
      while(digitalRead(pin_MISO))
        delay(100);
     
      //RESET 
      cmd_str(SRES, 0);             //SRES                  reset chip
      
      //Reset RX FIFO
      cmd_str(SFRX, 1);             //SFRX                  flush RX FIFO
      
      //Reset TX FIFO
      cmd_str(SFTX, 1);             //SFTX                  flush TX FIFO
  
  //**************SET UP RX****************//
  
  //high performance settings
  reg_write2F(0x12, 0x00);          //FS_DIG1: 0x00         Frequency Synthesizer Digital Reg. 1
  reg_write2F(0x13, 0x5F);          //FS_DIG0: 0x5F         Frequency Synthesizer Digital Reg. 0
  reg_write2F(0x16, 0x40);          //FS_CAL1: 0x40         Frequency Synthesizer Calibration Reg. 1
  reg_write2F(0x17, 0x0E);          //FS_CAL0: 0x0E         Frequency Synthesizer Calibration Reg. 0
  reg_write2F(0x19, 0x03);          //FS_DIVTWO: 0x03       Frequency Synthesizer Divide by 2
  reg_write2F(0x1B, 0x33);          //FS_DSM0: 0x33         FS Digital Synthesizer Module Configuration Reg. 0
  reg_write2F(0x1D, 0x17);          //FS_DVCO: 0x17         Frequency Synthesizer Divider Chain Configuration ..
  reg_write2F(0x1F, 0x50);          //FS_PFD: 0x50          Frequency Synthesizer Phase Frequency Detector Con..
//  reg_write2F(0x20, 0x6E);          //FS_PRE: 0x6E          Frequency Synthesizer Prescaler Configuration
//  msg = reg_read2F(0x20);
//  
//  if (msg == 0x6E)
//  {
//       Serial.println("\n*** Register 0x20 is correct. ***\n");
//  }
  
  reg_write2F(0x21, 0x14);          //FS_REG_DIV_CML: 0x14  Frequency Synthesizer Divider Regulator Configurat..
  reg_write2F(0x22, 0xAC);          //FS_SPARE: 0xAC        Set up Frequency Synthesizer Spare
  //reg_write2F(0x27, 0xB4);          //FS_VCO0: 0xB4         FS Voltage Controlled Oscillator Configuration Reg..
  //reg_write2F(0x32, 0x0E);          //XOSC5: 0x0E           Crystal Oscillator Configuration Reg. 5
  //reg_write2F(0x36, 0x03);          //XOSC1: 0x03           Crystal Oscillator Configuration Reg. 0
  
  //For test purposes only, (2nd block, deleted first one) use values from SmartRF for some bits
  reg_write(0x08, 0x0B);            //*Changed on line 152
  reg_write(0x13, 0x0D);            //
  reg_write(0x26, 0x04);            //*Changed on line 144
  
  //High performance RX
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
  reg_write(0x21, B00000100);       //FS_CFG: 0x14           set up LO divider to 8 (410.0 - 480.0 MHz band), out of lock detector enabled
  
  //set preamble
  reg_write(0x0D, 0x00);            //PREAMBLE_CFG1: 0x00    No preamble
  reg_write_bit(0x0E, 5, 0);        //PQT_EN: 0x00           Preamble detection disabled
  
  //TOC_LIMIT
  reg_write_bit2F(0x02, 7, 0);      //TOC_LIMIT: 0x00        Using the low tolerance setting (TOC_LIMIT = 0) greatly reduces system settling times and system power consumption as no preamble bits are needed for bit synchronization or frequency offset compensation (4 bits preamble needed for AGC settling).
  reg_write_bit2F(0x02, 6, 0);      //TOC_LIMIT: 0x00        Using the low tolerance setting (TOC_LIMIT = 0) greatly reduces system settling times and system power consumption as no preamble bits are needed for bit synchronization or frequency offset compensation (4 bits preamble needed for AGC settling).
  
  //set SYNC word
  reg_write_bit(0x08, 6, 0);        //PQT_GATING_EN: 0       PQT gating disabled (preamble not required)
  reg_write(0x09, 0x17);            //SYNC_CFG0: 0x17        32 bit SYNC word. Bit error qualifier disabled. No check on bit errors
  
  //set packets
  reg_write(0x26, 0x00);            //PKT_CFG2: 0x00         set FIFO mode
  reg_write(0x2E, 0xFF);            //PKT_LEN: 0xFF          set packet length to 0xFF (max)  
  
  //Frequency setting
  reg_write2F(0x0C, 0x6C);          //FREQ2: 0x6C            set frequency to 434MHz (sets Vco, see equation from FREQ2 section of user guide)
  reg_write2F(0x0D, 0x80);          //FREQ1: 0x80
  
  //************************************************************ MAIN PROGRAM *****************************************************//
   //turn on green LED when main code starts running
   Serial.println("\n*** Beginning of Main Program ***\n");
   
   //turn on green LED on breadboard and setup other LED pins as outputs
   pinMode(pin_greenLED, OUTPUT);
   pinMode(pin_redLED, OUTPUT);
   pinMode(pin_yellowLED, OUTPUT);
   digitalWrite(pin_greenLED, HIGH);
   
   for(int i = 0x00; i<0xFF; i++){
      Serial.print(dir_FIFO_read(i));
      Serial.print(", ");
    } 
    Serial.println(" ");
   
   
   //strobe commands to start RX
   cmd_str(SCAL, 1);                   //calibrate frequency synthesizer
   delay(250);
   cmd_str(SAFC, 1);
   delay(250);
   cmd_str(SRX, 1);                    //put in RX mode
}
   
//************************************************************ MAIN LOOP ********************************************************//
void loop() {
  byte RX_FIRST = reg_read2F(0xD2);
  bool CHIP_RDYn;
  byte state;
  
  monitor_LEDs();
  input_cmd_str();
  p_status_interval();
  get_status(&CHIP_RDYn, &state);
  print_FIFO();
  
  if(state == B110 || state == B111){
    Serial.println("**FIFO RESET**");
    cmd_str(SIDLE, 0);
    print_FIFO();
    cmd_str(SFRX, 0);
    cmd_str(SRX, 0);
  }   
}

//************************************************************ FUNCTION DECLARATION**********************************************//

byte reg_read(byte addr){
//write to register address addr with data  
  
    addr = addr + B10000000; //add the read bit
    SPI.transfer(addr); //send desired address
    return SPI.transfer(0); //read back

}

byte reg_read2F(byte addr){
//reads register in extended memory space

    SPI.transfer(B10101111); //address extension command
    SPI.transfer(addr); //send the desired address
    return SPI.transfer(0); //read back
}

byte reg_write(byte addr, byte data){
//write to register address addr with data
  
    SPI.transfer(addr); //send desired address
    return SPI.transfer(data); //send desired address}
	
}

byte reg_write2F(byte addr, byte data){
//writes to register in extended memory space

    SPI.transfer(B00101111); //address extension command
    SPI.transfer(addr); //send desired address
    return SPI.transfer(data); //send desired address}
}

void get_status(bool *CHIP_RDYn, byte *state){
//writes status information to variables in main loop

  byte status_b = cmd_str(SNOP, 0);  
  *CHIP_RDYn = (status_b/128)%2; //7th bit (reading backwards)
  *state = ((status_b/64)%2)*4 + ((status_b/32)%2)*2 + (status_b/16)%2; //6th-4th bits (reading backwards)
  return;
}

void p_chip_status(){
//parses chip status byte

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

byte cmd_str(byte addr, bool Print){
//send command strobe and print info on command strobe if Print bit is true
  
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

void get_cmd_str(){ 
//when Serial.available()==0, checks for a command strobe and executes it

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
  else if(cmd_str_in == "FF")
    reg_write(0x0A, 0xFF);
  else if(cmd_str_in == "00")
    reg_write(0x0A, 0x00);
    
  return;
}

byte dir_FIFO_read(byte addr){
//reads FIFO using direct access
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

void set_CSn(bool state){
//sets chip select to either LOW or HIGH

  if(state)
    digitalWrite(pin_SS, HIGH);
  else
    digitalWrite(pin_SS, LOW);
}

void p_all_reg(){
//prints the value of each register
  
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

void monitor_LEDs(){
//turns on LEDs for different situations
  
  bool CHIP_RDYn;
  byte state;
  
  //red LED goes on if there is an error
  get_status(&CHIP_RDYn, &state);
  if(CHIP_RDYn || state==B110 || state==B111)
    digitalWrite(pin_redLED, HIGH);
  else
    digitalWrite(pin_redLED, LOW);
  
  if(state==B001 || state==B010)
    digitalWrite(pin_yellowLED, HIGH);
  else
    digitalWrite(pin_yellowLED, LOW);
}

void input_cmd_str(){
//uses the get_cmd_str() function to check for an input command strobe
  
  //inputs command strobes from serial monitor
  if(Serial.available()){
    delay(100);          //delay so that serial input doesn't get erased
    get_cmd_str();       //takes serial input and executes command strobe
 }
}

void p_status_interval(){
//uses the p_chip_status() function to print status on regular time intervals
  
//print status on time intervals
 currentTime=millis();
 if(currentTime-previousTime>=interval){
   p_chip_status();
   Serial.print("0x0A: ");
   Serial.println(reg_read(0x0A), HEX);
 //  byte RX_FIRST = reg_read2F(0xD2);
 //  byte RX_LAST = reg_read2F(0xD4);
 //  byte NUM_RXBYTES = reg_read2F(0xD7);
 //  int CARRIER_SENSE = reg_read_bit2F(0x72, 2);
 //  Serial.print("RX_FIRST: ");
 //  Serial.println(RX_FIRST, HEX);
 //  Serial.print("RX_LAST: ");
 //  Serial.println(RX_LAST, HEX);
 //  Serial.print("NUM_RXBYTES: ");
 //  Serial.println(NUM_RXBYTES);
 //  Serial.print("CARRIER_SENSE: ");
 //  Serial.println(CARRIER_SENSE);
   previousTime=currentTime;
 }
}

void reg_write_bit(int reg, int n, int data){
//changes the nth bit in register 'reg' to data
  
  int old_value = reg_read(reg);
  int power = pow(2, n);
  int new_value = old_value-(old_value%(2*power)) + data*power + old_value%power;
  reg_write(reg, new_value);
  return;
}

int reg_read_bit(int reg, int n){
//reads the nth bit in register 'reg'
  
  int old_value = reg_read(reg);
  int power = pow(2, n);
  int nth_bit = old_value%(2*power)/power;
  return nth_bit;
}

void reg_write_bit2F(int reg, int n, int data){
//changes the nth bit in register 'reg' to data (extended register space)
  
  int old_value = reg_read2F(reg);
  int power = pow(2, n);
  int new_value = old_value-(old_value%(power)) + data*power + old_value%power;
  reg_write2F(reg, new_value);
  return;
}

int reg_read_bit2F(int reg, int n){
//reads the nth bit in register 'reg'(extended register space)
  
  int old_value = reg_read2F(reg);
  int power = pow(2, n);
  int nth_bit = old_value%(2*power)/power;
  return nth_bit;
}

int get_RSSI(){
//Returns the RSSI
  
  int rssi = 0;
  int eight_MSB = reg_read2F(0x71)*pow(2, 4);
  
  if(eight_MSB>=3968)
    rssi = (-1*eight_MSB+4096)*-1;
  else
    rssi = eight_MSB;
  return rssi;
}

void print_FIFO(){
//Prints what is in the FIFO queue 
  
  for(int i = 0x80; i<0xFF; i++){
     /* Serial.print("(");
        Serial.print(i, HEX);
        Serial.print("): ");*/
        Serial.print(dir_FIFO_read(i), HEX);
        Serial.print(", ");
   }
  // Serial.print("(FF): ");
   Serial.print(dir_FIFO_read(0xFF));
   Serial.println(".");
}
