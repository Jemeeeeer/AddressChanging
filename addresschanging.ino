#include <HardwareSerial.h>

#define REPORTING_PERIOD_MS 1000
#define HardwareSerial
#define ESP32
#define PZEM_DEFAULT_ADDR 0xF8
#define SET_ADDRESS 0xF7
#define READ_TIMEOUT 1000
#define INCREMENT false
#define NEW_ADDRESS 0xF7
#define PZEM_BAUD_RATE  9600

#define REG_VOLTAGE     0x0000
#define REG_CURRENT_L   0x0001
#define REG_CURRENT_H   0X0002
#define REG_POWER_L     0x0003
#define REG_POWER_H     0x0004
#define REG_ENERGY_L    0x0005
#define REG_ENERGY_H    0x0006
#define REG_FREQUENCY   0x0007
#define REG_PF          0x0008
#define REG_ALARM       0x0009


#define READ_TIMEOUT 100
#define CMD_RHR         0x03
#define CMD_RIR         0X04
#define CMD_WSR         0x06
#define CMD_CAL         0x41
#define CMD_REST        0x42
#define WREG_ADDR  0xF7
#define INVALID_ADDRESS 0x00
#define RESPONSE_SUCCESS 0xF8 // Example constant for success response

#define DEBUGLN(x) Serial.println(x)

//byte customAddressCommand[] = {0xF8, 0x04, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00};
Stream* _serial = &Serial;
bool _isSoft;    // Is serial interface software
bool _isConnected = false;  
uint8_t _addr = PZEM_DEFAULT_ADDR;

//function prototypes
uint8_t readtheAddress(bool update);
bool settheAddress(uint8_t addr);
void printReceivedBytes(uint8_t *data, uint16_t length);
bool sendCommand8(uint8_t cmd, uint16_t rAddr, uint16_t val, bool check, uint16_t slave_addr);
void settheCRC(uint8_t *buf, uint16_t len);
uint16_t CRC_16(const uint8_t *data, uint16_t len);
uint16_t receive(uint8_t *resp, uint16_t len);
bool checktheCRC(const uint8_t *buf, uint16_t len);
float getvoltage();
bool updatetheValues();

void initializePZEM004Tv30(uint8_t receivePin, uint8_t transmitPin, uint8_t addr);

struct CurrentValues {
    float getvoltage;
} _currentValues;

unsigned long _lastRead = 0;
const unsigned long UPDATE_TIME = 1000; // Define your desired update time

void setup() {
  Serial.begin(9600);
  _serial = &Serial;
  Serial.print("Starting...");
  initializePZEM004Tv30(16,17,0xF8); // UART0 (Serial0) on pins 3 (RX) and 1 (TX)
}

void loop() {
    //static uint8_t addr = SET_ADDRESS;
    //Serial.print("Previous address:   0x");
    //Serial.println(pzem1.getAddress(), HEX);
    //Serial.println(readtheAddress(true), HEX);

    // Set the custom address
    //Serial.print("Setting address to: 0x");
    //Serial.println(addr, HEX);
    //if(!settheAddress(0xF7))
    //{
      // Setting custom address failed. Probably no PZEM connected
      //Serial.println("Error setting address.");
    //} else {
      // Print out the new custom address
      //Serial.print("Current address:    0x");
      //Serial.println(readtheAddress(true), HEX);
      //Serial.println();
    //}

    // Update address based on specific conditions
    //addr = (addr == SET_ADDRESS) ? NEW_ADDRESS : SET_ADDRESS;  
    //delay(1000);
    Serial.print("Voltage: "); 
    Serial.print(getvoltage());
    Serial.println("V");
    Serial.println("=======================================");
    delay(3000);
  
}

//---------------------------------------init-------------------------
void init(Stream* port, bool isSoft, uint8_t addr){
    if(addr < 0x01 || addr > 0xF8) // Sanity check of address
        addr = PZEM_DEFAULT_ADDR;
    _addr = addr;

    _serial = port;
    _isSoft = isSoft;

    // Set initial lastRed time so that we read right away
    _lastRead = 0;
    _lastRead -= UPDATE_TIME;

    _isConnected = false; // We have not received anything yet...
}

//-------------------------------initializePZEM004Tv30-------------------------------
void initializePZEM004Tv30(uint8_t receivePin, uint8_t transmitPin, uint8_t addr)
{
    Serial.begin(PZEM_BAUD_RATE, SERIAL_8N1, receivePin, transmitPin);
    init((Stream *)&Serial, false, addr);
}


//--------------------------------getvoltage------------------------------------------
float getvoltage()
{
    if(!updatetheValues()) // Update vales if necessary
        return NAN; // Update did not work, return NAN

    return _currentValues.getvoltage;
}


//----------------------UpdateValues)-----------------------
bool updatetheValues()
{
    //static uint8_t buffer[] = {0x00, CMD_RIR, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00};
    static uint8_t response[25];

    // If we read before the update time limit, do not update
    if(_lastRead + UPDATE_TIME > millis()){
        return true;
    }

    DEBUGLN("Updating Values");


    // Read 10 registers starting at 0x00 (no check)
    sendCommand8(CMD_RIR, 0x00, 0x0A, false, 0xF8);


    if(receive(response, 25) != 25){ // Something went wrong
        return false;
    }



    // Update the current values
    _currentValues.getvoltage = ((uint32_t)response[3] << 8 | // Raw voltage in 0.1V
                              (uint32_t)response[4])/10.0;
    Serial.println("currentvalues getvoltage is ");
    Serial.print(_currentValues.getvoltage);

    //_currentValues.getcurrent = ((uint32_t)response[5] << 8 | // Raw current in 0.001A
                              //(uint32_t)response[6] |
                              //(uint32_t)response[7] << 24 |
                              //(uint32_t)response[8] << 16) / 1000.0;

    //_currentValues.getpower =   ((uint32_t)response[9] << 8 | // Raw power in 0.1W
                              //(uint32_t)response[10] |
                              //(uint32_t)response[11] << 24 |
                              //(uint32_t)response[12] << 16) / 10.0;

    //_currentValues.getenergy =  ((uint32_t)response[13] << 8 | // Raw Energy in 1Wh
                              //(uint32_t)response[14] |
                              //(uint32_t)response[15] << 24 |
                              //(uint32_t)response[16] << 16) / 1000.0;

    //_currentValues.getfrequency=((uint32_t)response[17] << 8 | // Raw Frequency in 0.1Hz
                              //(uint32_t)response[18]) / 10.0;

    //_currentValues.getpf =      ((uint32_t)response[19] << 8 | // Raw pf in 0.01
                              //(uint32_t)response[20])/100.0;

    //_currentValues.getalarms =  ((uint32_t)response[21] << 8 | // Raw alarm value
                              //(uint32_t)response[22]);

    // Record current time as _lastRead
    _lastRead = millis();

    return true;
}



//---------------------readtheAddress---------------------------------
uint8_t readtheAddress(bool update)
{
    static uint8_t response[7];
    uint8_t addr = 0;
    // Read 1 register
    if (!sendCommand8(CMD_RHR, WREG_ADDR, 0x01, update,0xF8))
        Serial.println("Invalid Address (ReadAddress)");
        return INVALID_ADDRESS;
        


    if(receive(response, 6) != 6){ // Something went wrong
        Serial.print("Something went wrong in the (ReadAddress)");
        return INVALID_ADDRESS;
        
    }
    printReceivedBytes(response, 6);
    // Get the current address
    addr = ((uint32_t)response[3] << 8 | // Raw address
                              (uint32_t)response[4]);

    // Update the internal address if desired
    if(update){
         _addr = addr;
    }
    return addr;
}

//-----------------settheAddress-------------------------------------

bool settheAddress(uint8_t addr)
{
    if(addr < 0x01 || addr > 0xF7) // sanity check
        return false;

    // Write the new address to the address register
    if(!sendCommand8(CMD_WSR, WREG_ADDR, addr, true,0xF7))
        return false;

    _addr = addr; // If successful, update the current slave address

    return true;
}




void printSendBuffer(uint8_t *buffer, uint16_t length) {
    Serial.print("Send Buffer Contents: ");
    for (uint16_t i = 0; i < length; i++) {
        Serial.print(buffer[i], HEX); // Print each byte in hexadecimal format
        Serial.print(" "); // Add a space for better readability
    }
    Serial.println(); // Add a new line after printing all bytes
}


//------------------------sendCommand8---------------------------------
bool sendCommand8(uint8_t cmd, uint16_t rAddr, uint16_t val, bool check, uint16_t slave_addr){
    uint8_t sendBuffer[8]; // Send buffer
    uint8_t respBuffer[8]; // Response buffer (only used when check is true)

    if((slave_addr == 0xFFFF) ||
       (slave_addr < 0x01) ||
       (slave_addr > 0xF7)){  
        slave_addr = _addr;
    }

    sendBuffer[0] = slave_addr;                   // Set slave address
    sendBuffer[1] = cmd;                     // Set command

    sendBuffer[2] = (rAddr >> 8) & 0xFF;     // Set high byte of register address
    sendBuffer[3] = (rAddr) & 0xFF;          // Set low byte =//=

    sendBuffer[4] = (val >> 8) & 0xFF;       // Set high byte of register value
    sendBuffer[5] = (val) & 0xFF;            // Set low byte =//=

    settheCRC(sendBuffer, 8);                   // Set CRC of frame

    printSendBuffer(sendBuffer, 8); // Print the contents of sendBuffer
    delay(200);
    if (!_serial->write(sendBuffer, 8)){ // send frame
    Serial.println("Unsuccessfully wrote to the serial!(sendCommand8 Function)");
    }else{
      Serial.println("Successfully wrote to the serial!(sendCommand8 Function)");
    }
    delay(200);
    if(check) {
        if(!receive(respBuffer, 8)){ // if check enabled, read the response
            return false;
        }
        
         if (respBuffer[0] != RESPONSE_SUCCESS) { // Replace RESPONSE_SUCCESS with appropriate response code
            return false;
         }
    }
    return true;
}

//--------------------------------settheCRC---------------------------
void settheCRC(uint8_t *buf, uint16_t len){
    if(len <= 2) // Sanity check
        return;

    uint16_t crc = CRC_16(buf, len - 2); // CRC of data

    // Write high and low byte to last two positions
  buf[len - 2] = (crc >> 8) & 0xFF;  //high byte
  buf[len - 1] = crc & 0xFF;  //low byte
}


// Pre computed CRC table
static const uint16_t crcTable[] PROGMEM = {
    0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
    0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
    0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
    0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
    0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
    0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
    0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
    0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
    0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
    0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
    0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
    0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
    0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
    0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
    0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
    0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
    0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
    0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
    0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
    0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
    0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
    0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
    0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
    0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
    0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
    0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
    0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
    0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
    0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
    0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
    0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
    0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040
};


uint16_t CRC_16(const uint8_t *data, uint16_t len)
{
    uint8_t nTemp; // CRC table index
    uint16_t crc = 0xFFFF; // Default value

    while (len--)
    {
        nTemp = *data++ ^ crc;
        crc >>= 8;
        crc ^= (uint16_t)pgm_read_word(&crcTable[nTemp]);
    }
    return crc;
}
//--------------------------------------------------------------------------------------------------------
void printReceivedBytes(uint8_t *data, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        Serial.print(data[i], HEX); // Print each byte as hexadecimal
        Serial.print(" "); // Separate bytes with a space
    }
    Serial.println(); // Print a newline at the end
}

//-----------------------------------------receive---------------------------------------------------------------
uint16_t receive(uint8_t *resp, uint16_t len)
{
      //* This has to only be enabled for Software serial
    unsigned long startTime = millis(); // Start time for Timeout
    uint8_t index = 0; // Bytes we have read
    uint16_t crc = 0; // Initialize CRC
    while((index < len) && (millis() - startTime < READ_TIMEOUT))
    {
        if(Serial.available() > 0 && index < len)
        {
            Serial.print("there are available bytes to read (Receive Function)");
            uint8_t c = (uint8_t)_serial->read();
            resp[index++] = c;
            crc=CRC_16(resp, index);
        }      
        else{
          Serial.println("No available bytes to read (Receive Function)");
        }
        yield();  // do background netw tasks while blocked for IO (prevents ESP watchdog trigger)
        }

        if (index < len) {
        // Handle the timeout condition appropriately
          _isConnected = false; // We are no longer connected
          return 0; // or return an appropriate error code
        }

    // Check CRC with the number of bytes read
        if(!checktheCRC(resp, len)){
          _isConnected = false; // We are no longer connected
          return 0;
        }

         _isConnected = true; // We received a reply 
        return index;
    
}
//------------------checktheCRC---------------------------------------
bool checktheCRC(const uint8_t *buf, uint16_t len)
{
    if(len <= 2) // Sanity check
        return false;

    uint16_t crc = CRC_16(buf, len - 2); // Compute CRC of data
    return ((uint16_t)buf[len-2]  | (uint16_t)buf[len-1] << 8) == crc;
}

 
