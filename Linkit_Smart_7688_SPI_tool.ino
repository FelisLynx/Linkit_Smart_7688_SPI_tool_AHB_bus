// MEdiatek         - ATMEGA     - EVB PIN 
// MDI_TN_P1 - CLK  - PB1 - SCLK - S2 
// MDI_RN_P1 - MOSI - PB3 - MISO - S0 
// MDI_TP_P1 - CS   - PB0 - SS   - S3 
// MDI_RP_P1 - MISO - PB2 - MOSI - S1 

#include <SPI.h>
#include "pins_arduino.h"
#include "Linkit_Smart_7688_AHB.h"

#define CS_PIN SS
#define SPI_DELAY 0
#define DUMMY 0xF0

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin();

  // Default mode - SPI_MODE0
  // TRY MSBFIRST or LSBFIRST, no result
  SPI.beginTransaction(SPISettings(1000000, LSBFIRST, SPI_MODE0));
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    while (Serial.available() > 0) Serial.read();
    //testPrintMem();
    print_mem_range(0x10000000,1);
  }
}

byte SPIbusy(){
  return(SPI_standard_transaction(AHB_READ,0x10,DUMMY) & 0b00000001);
}

byte SPI_standard_transaction(AHB_RW A_rw, byte addr, byte value){
  digitalWrite(CS_PIN, LOW);
  delay(SPI_DELAY);
  
  if(A_rw)
    addr = addr | 0b10000000; // MSB=1 - write
  else
    addr = addr & 0b01111111; // MSB=0 - read
    
  SPI.transfer(addr);
  byte result = SPI.transfer(value);
  
  delay(SPI_DELAY);
  digitalWrite(CS_PIN, HIGH);
  return(result);
}

unsigned long SPIread_word(unsigned long addr){

  // write bus addr we wont to read to registers
  SPI_standard_transaction(AHB_WRITE, 0x08, (byte)(addr>>0));
  SPI_standard_transaction(AHB_WRITE, 0x09, (byte)(addr>>8));
  SPI_standard_transaction(AHB_WRITE, 0x0A, (byte)(addr>>16));
  SPI_standard_transaction(AHB_WRITE, 0x0B, (byte)(addr>>24));
  
  // start bus read
  SPI_standard_transaction(AHB_WRITE, 0x0C, AHB_CMD_reg_create(AHB_R_BUS, AHB_WORD, AHB_READ));

  // wait for info to be placed in registers
  while(SPIbusy()){
    Serial.println("SPI busy during SPIread_word");
  }

  unsigned long result = 0;
  
  result |= SPI_standard_transaction(AHB_READ, 0x00, DUMMY);
  result |= SPI_standard_transaction(AHB_READ, 0x01, DUMMY)<<8;
  result |= SPI_standard_transaction(AHB_READ, 0x02, DUMMY)<<16; // potential problem here, solve later
  result |= SPI_standard_transaction(AHB_READ, 0x03, DUMMY)<<24;

  return(result);
}

byte AHB_CMD_reg_create(AHB_assert_bus A_bus, AHB_size A_size, AHB_RW A_rw){
  return( A_bus<<4 | A_size<<1 | A_rw);
}

void print_mem_range (unsigned long addr, unsigned long len){
  Serial.println("Start print_mem_range");
  unsigned long end_addr = addr + len*32;
  while (addr < end_addr){
    
    byte safety_count = 0;
    while(SPIbusy()){
      Serial.println("SPI busy during print_mem_range");
      safety_count++;
      if (safety_count == 100) return;
    }
      printMemAddr(addr);
      printMemVal(SPIread_word(addr));
      addr+=32;
  }
  Serial.println("End print_mem_range");
}

void printHex(unsigned long num, int precision) {
  char tmp[16];
  char format[128];
  
  sprintf(format, "0x%%.%dX", precision);
  sprintf(tmp, format, num);
  Serial.print(tmp);
}

void testPrintMem(){
  Serial.println("Start test print");
  printMemAddr(0x12345678);
  printMemVal(0xA1B2C3D4);
  Serial.println("End test print");
}

void printMemAddr(unsigned long addr){
  char tmp[16];
  sprintf(tmp, "0x%04X%04X: ", (int)(addr>>16), (int)addr);
  Serial.print(tmp);
}

void printMemVal(unsigned long val){
  char tmp[32];
  sprintf(tmp, "%02X %02X %02X %02X\n", (byte)(val>>24), (byte)(val>>16), (byte)(val>>8), (byte)val);
  Serial.print(tmp);
}
