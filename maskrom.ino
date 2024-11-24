#define LED PC13

/*
 * data pins should be 5V tolerant in case
 * we want to run on 5V 
 * 

pinout for 128kb 28-pin using an stm32f103c8t6 blue pill

micro  rom    ---_---   rom   micro
PA7    A15 - |01   28| - +5V  +5V
PA4    A12 - |02   27| - A14  PA6
PC15   A7  - |03   26| - A13  PA5
PC14   A6  - |04   25| - A8   PA0
PB12   A5  - |05   24| - A9   PA1
PB13   A4  - |06   23| - A11  PA3
PB14   A3  - |07   22| - A16  PB0
PB15   A2  - |08   21| - A10  PA2
PA8    A1  - |09   20| - /CE  GND or PB5
PA9    A0  - |10   19| - D7   PA10
PB9    D0  - |11   18| - D6   PA15
PB8    D1  - |12   17| - D5   PB3
PB7    D2  - |13   16| - D4   PB4
GND    GND - |14   15| - D3   PB6

 more pinouts here: https://www.nesdev.org/NES%20ROM%20Pinouts.txt
 * 
 */

// Make sure the data pins are 5v-tolerant. For blue pill, see:
// http://land-boards.com/blwiki/index.php?title=File:STM32F103C8T6-Blue-Pill-Pin-Layout.gif
#define NES_CE PB5 // undefine to just connect directly to ground
#define NES_D0 PB9
#define NES_D1 PB8
#define NES_D2 PB7
#define NES_D3 PB6
#define NES_D4 PB4
#define NES_D5 PB3
#define NES_D6 PA15
#define NES_D7 PA10

#define NES_A00 PA9
#define NES_A01 PA8
#define NES_A02 PB15
#define NES_A03 PB14
#define NES_A04 PB13
#define NES_A05 PB12
#define NES_A06 PC14
#define NES_A07 PC15
#define NES_A08 PA0
#define NES_A09 PA1
#define NES_A10 PA2
#define NES_A11 PA3
#define NES_A12 PA4
#define NES_A13 PA5
#define NES_A14 PA6
#define NES_A15 PA7
#define NES_A16 PB0
#define NES_A17 PB10
#define NES_A18 PB11

#define SMS_NWR NES_A15

#define MAX_ADDRESS_BITS 19

#define QUICK_CHECKSUM_SPACING 17

const uint32_t smsAddress[MAX_ADDRESS_BITS] = {
  NES_A00, NES_A01, NES_A02, NES_A03, NES_A04, NES_A05, NES_A06, NES_A07, 
  NES_A08, NES_A09, NES_A10, NES_A11, NES_A12, NES_A13, NES_A14, NES_A16, NES_A16, NES_A17, NES_A18
};

const uint32_t nesAddress[MAX_ADDRESS_BITS] = {
  NES_A00, NES_A01, NES_A02, NES_A03, NES_A04, NES_A05, NES_A06, NES_A07, 
  NES_A08, NES_A09, NES_A10, NES_A11, NES_A12, NES_A13, NES_A14, NES_A15, NES_A16, NES_A17, NES_A18
};

const uint32_t nesData[8] = {
  NES_D0, NES_D1, NES_D2, NES_D3, NES_D4, NES_D5, NES_D6, NES_D7
};

enum {
  MODE_AUTO,
  MODE_DETECT,
  MODE_NES,
  MODE_SMS,
  MODE_NONE
} readMode = MODE_NONE;

uint32_t address = 0;
unsigned addressBits = 17;
uint32_t romSize = 128*1024;
uint32_t crc = 0;
uint32_t currentBank = 0xFFFFFFFF;
char outBuffer[128];

uint8_t read(uint32_t address) {
  if (readMode == MODE_SMS) {
    uint32_t bank = address / 16384;
    if (currentBank != bank) {
      smsWrite(0xFFFF, bank);
      currentBank = bank;
      delayMicroseconds(16);
    }
    address = 0x8000 + (address % 16384);
  }
  setAddress(address);
  delayMicroseconds(4);
  uint8_t datum = 0;
  for (int i = 7 ; i >= 0 ; i--) {
    datum = (datum << 1) | digitalRead(nesData[i]);
  }
  
  return datum;
}

void getLine(char* p, unsigned length) {
  unsigned pos = 0;

  while (pos < length - 1) {
    int c;
    while (-1 == (c = Serial.read())) ;
    if (c == '\n') {
      p[pos] = 0;
      return;
    }
    else if (c == 8) {
      if (pos > 0) 
        pos--;
    }
    else if (c != '\r') {
      p[pos++] = c;
    }
  }
  p[pos] = 0;
}

// crc32 adapted from https://github.com/aeldidi/crc32/blob/master/src/crc32.c
uint32_t
crc32_for_byte(uint32_t byte)
{
  const uint32_t polynomial = 0xEDB88320L;
  uint32_t       result     = byte;
  size_t         i          = 0;

  for (; i < 8; i++) {
    /* IMPLEMENTATION: the code below always shifts result right by
     * 1, but only XORs it by the polynomial if we're on the lowest
     * bit.
     *
     * This is because 1 in binary is 00000001, so ANDing the
     * result by 1 will always give 0 unless the lowest bit is set.
     * And since XOR by zero does nothing, the other half only
     * occurs when we're on the lowest bit.
     *
     * I didn't leave the above implementation in, despite being
     * faster on my machine since it is a more complex operation
     * which may be slower on less sophisticated processors. It can
     * be added in in place of the loop code below.
     */

    result = (result >> 1) ^ (result & 1) * polynomial;

    /* Here is the code I replaced with the branch I tried to
     * remove:
    if (result & 1) {
      result = (result >> 1) ^ polynomial;
      continue;
    }
    result >>= 1;
     */
  }
  return result;
}

uint32_t crc32(const void *input, size_t size, uint32_t start)
{
  const uint8_t *current = (const uint8_t*)input;
  uint32_t       result  = ~start;
  size_t         i       = 0;

  for (; i < size; i++) {
    result ^= current[i];
    result = crc32_for_byte(result);
  }

  return ~result;
}

void clearAddress() {
  for (unsigned i = 0 ; i < MAX_ADDRESS_BITS ; i++)
    digitalWrite(nesAddress[i], 0);
}

uint8_t setAddress(uint32_t address) {
  const uint32_t* pins = readMode == MODE_SMS ? smsAddress : nesAddress;
  for (unsigned i = 0 ; i < addressBits ; i++, address >>= 1) {
//    
//    if (i==15) digitalWrite(nesAddress[i], !(address & 1));//ARP
//    else 
    digitalWrite(pins[i], address & 1);
  }
}

void smsWrite(uint16_t address, uint8_t value) {
//  sprintf(outBuffer, "SMS write %04lx %02x", address, value);
//  Serial.println(outBuffer);
  for (unsigned i = 0 ; i < 8 ; i++) {
    digitalWrite(nesData[i], value & 1);
    value >>= 1;
  }
  setAddress(address);
  digitalWrite(SMS_NWR,0);
  for (int i=0;i<8;i++)
     pinMode(nesData[i], OUTPUT);
  delayMicroseconds(4);
  digitalWrite(SMS_NWR,1);
  for (int i=0;i<8;i++)
     pinMode(nesData[i], INPUT);
}

uint32_t quickChecksum(uint32_t address, uint32_t count) {
  outBuffer[0]=0;
  uint32_t crc = 0;
  while(count >= QUICK_CHECKSUM_SPACING) {
    uint8_t datum = read(address);
    crc = crc32(&datum, 1, crc);
    address += QUICK_CHECKSUM_SPACING;
    count -= QUICK_CHECKSUM_SPACING;
  }
  return crc;
}

bool detect() {
  clearAddress();
  Serial.println("Scanning for SMS");
  addressBits = 16;
  delayMicroseconds(40);
  smsWrite(0xFFFC,0x80);
  delayMicroseconds(40);
  readMode = MODE_SMS;
  uint32_t banks[512/8]; // for NES
  uint32_t i;
  uint32_t crc2;
  for (i=0;i<8;i++) {
    delayMicroseconds(16);
    read(i*16384);
    clearAddress(); // in case it's NES
    banks[i] = quickChecksum(i*16384,16384);
    crc2 = quickChecksum(i*16384,16384); 
    sprintf(outBuffer, "Bank %d quick checksums %08lx/%08lx", i, banks[i],crc2);
    Serial.println(outBuffer);
    if (banks[i] != crc2 || (i>0 && banks[i] == banks[i-1]))
      break;
  }
  if (i>1) {
    sprintf(outBuffer, "Detected SMS with at least %d banks, assuming 128k",i);
    Serial.println(outBuffer);
    romSize = 128 * 1024;
    readMode = MODE_SMS;
    return true;
  }

  clearAddress();
  readMode = MODE_NES;
  Serial.println("Assuming NES");
  addressBits = MAX_ADDRESS_BITS;
  banks[0] = quickChecksum(0,8192);
  crc2 = quickChecksum(0,8192);
  if (banks[0] != crc2) {
    Serial.println("Unreliable read of first 8k");
    Serial.println("Detection unsuccessful");
    readMode = MODE_NONE;
    return false;
  }
  sprintf(outBuffer,"0:%08lx ",banks[0]);
  Serial.print(outBuffer);
  for (i=1;i<64;i++) {
    banks[i] = quickChecksum(i*8192,8192);
    crc2 = quickChecksum(i*8192,8192);
    if (banks[i] != crc2)
      break;
    sprintf(outBuffer,"%d:%08lx ",i,banks[i]);
    Serial.print(outBuffer);
  }
  Serial.print("\n");
  int maxChunks = i;
  int chunks = 1;
  for (; chunks < 64 ; chunks *= 2) {
    // determine how many 8k chunks there are in the ROM, by checking if data repeats
    bool repeat = true;
    for (unsigned offset = chunks ; offset + chunks <= maxChunks && repeat ; offset += chunks ) {
        for (unsigned j = 0 ; j < chunks ; j++) {
          if (banks[j] != banks[offset+j]) {
            repeat = false;
            clearAddress();
            break;
          }
        }
    }
  }
  sprintf(outBuffer,"Detected NES of size %dk",chunks*8);
  Serial.println(outBuffer);
  romSize = chunks*8192;
  clearAddress();
  return true;
}

void show(unsigned address) {
  Serial.print(address,HEX);
  Serial.print(" ");
  for(int i=0;i<32;i++) {
    sprintf(outBuffer, "%02x", read(address+i));
    Serial.print(outBuffer);
  }
  Serial.print("\n");
}

void getCommand() {
  char input[128];
  Serial.println("Mask ROM dumper ready.");
  do {
    Serial.println("ROM size/mode selection:");
    Serial.println("  [nes]8/16/32/64/128/256/512: dump");
    Serial.println("  sms[8/16/.../128]: dump, default 128");
    Serial.println("  auto: dump");
    Serial.println("  detect: only detects");
    getLine(input, 128);
    char* p = input;
    Serial.println(input);
    if (!strncmp(p,"nes",3)) {
      p += 3;
    }
    if (isdigit(p[0])) {
      romSize = (uint32_t)atol(p);      
      readMode = MODE_NES;
      Serial.println("NES mode");
    }
    else if (!strncmp(p,"sms",3)) {
      romSize = (uint32_t)atol(p+3);
      readMode = MODE_SMS;
      Serial.println("SMS mode");
    }
    else if (!strncmp(p,"auto",4)) {
      readMode = MODE_AUTO;
      Serial.println("Auto mode");
    }
    else if (!strncmp(p,"detect",5)) {
      readMode = MODE_DETECT;
      Serial.println("Detect mode");
    }
    else {
      continue;
    }
    romSize = (uint32_t)atol(input);
    if (readMode == MODE_DETECT || readMode == MODE_AUTO) {
      bool detectOnly = readMode == MODE_DETECT;
      if (!detect() || detectOnly)
        continue;
    }
    else {
      if (romSize == 0 || romSize > 512) {
        Serial.println("Invalid ROM size.");
        continue;
      } 
      romSize *= 1024; 
    }
    crc = 0;
    currentBank = 0xFFFFFFFF;
    if (readMode == MODE_NES) {
      for (unsigned i = 0 ; i < MAX_ADDRESS_BITS ; i++)
        if (romSize <= (1ul << i)) {
          address = 0;
          addressBits = i;
          crc = 0;
          break;
        }    
    }
    else {
      addressBits = 16;
    }
    Serial.print("Addresses will be ");
    Serial.print(addressBits);
    Serial.println(" bits long.");
    return;
  } while(1);    
}

void setup() {
  for (unsigned i = 0 ; i < MAX_ADDRESS_BITS ; i++) {
    pinMode(nesAddress[i], OUTPUT);
    digitalWrite(nesAddress[i], 0);
  }
#ifdef NES_CE
  pinMode(NES_CE, OUTPUT);
  digitalWrite(NES_CE, 0);
#endif
  
  for (unsigned i = 0 ; i < 8 ; i++) {
    pinMode(nesData[i], INPUT);
  }

  while(!Serial);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, 1);
  delay(4000);
  digitalWrite(LED, 0);
  getCommand();
}

void smsLoop() {
  delayMicroseconds(2);
  smsWrite(0xFFFC,0x80);

  uint32_t crc = 0;

  for (unsigned bank = 0; bank < 8 ; bank++) {
    smsWrite(0xFFFF,bank);
    uint32_t fullAddress = bank * 16384;
    uint32_t readAddress = 0x8000;
    while (readAddress < 0xC000) {
      char s[128];
      uint8_t data16[16];
      uint16_t sum = 0;
      sprintf(s, ">%05lx ", fullAddress);
      Serial.print(s);
      for (unsigned i = 0 ; i < 16 ; i++) {
        uint8_t datum = read(readAddress);
        data16[i] = datum;
        sum += datum;
        readAddress++;
        fullAddress++;
        sprintf(s, "%02x", datum);
        Serial.print(s);
      }
      crc = crc32(data16, 16, crc);
      sprintf(s, " %04x", sum);
      Serial.println(s);
    }
  }  
}

void loop() {
  char s[9];
  uint8_t data16[16];
  if (address >= romSize) {
    Serial.print("Dumped: ");
    Serial.print(romSize);
    Serial.print(" bytes\nCRC-32: ");
    sprintf(s,"%08lx", crc);
    Serial.println(s);
    getCommand();
  }
  uint16_t sum = 0;
  sprintf(s, ">%05lx ", address);
  Serial.print(s);
  for (unsigned i = 0 ; i < 16 ; i++) {
    uint8_t datum = read(address);
    data16[i] = datum;
    sum += datum;
    address++;
    sprintf(s, "%02x", datum);
    Serial.print(s);
  }
  crc = crc32(data16, 16, crc);
  sprintf(s, " %04x", sum);
  Serial.println(s);
}
