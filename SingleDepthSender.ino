#include <Wire.h>
#include <VL6180X.h>

#define RANGE 1



#include <XBee.h>
#include <SoftwareSerial.h>
#include <Printers.h>
#include <elapsedMillis.h>


//XBEE & COMMUNICATIONS
SoftwareSerial xbeeSerial(2, 4); // RX, TX

//Works with Series1 and 2
XBeeWithCallbacks xbee;

uint8_t wobbleIgnore = 3;
uint8_t lastDistance = 0;

// Build a reuseable message packet to send to the Co-Ordinator
XBeeAddress64 coordinatorAddr = XBeeAddress64(0x00000000, 0x00000000);

uint8_t distanceMessagePayload[1] = {0};
ZBTxRequest distanceMessage = ZBTxRequest(coordinatorAddr, distanceMessagePayload, sizeof(distanceMessagePayload));

//TOF sensors
/* List of adresses for each sensor - after reset the address can be configured */
#define address0 0x20

///* These Arduino pins must be wired to the IO0 pin of VL6180x */
int enablePin0 = 5;


///* Create a new instance for each sensor */
VL6180X sensor0;

void setup() {
  Serial.begin(9600);   // Initialize serial communications with the PC
//  SPI.begin();      // Init SPI bus
  Wire.begin();

  
  //Distance Sensors
  SetSensorI2CAddresses();

  // XBEE
  xbeeSerial.begin(9600);
  xbee.setSerial(xbeeSerial);

  // Make sure that any errors are logged to Serial. The address of
  // Serial is first cast to Print*, since that's what the callback
  // expects, and then to uintptr_t to fit it inside the data parameter.
  xbee.onPacketError(printErrorCb, (uintptr_t)(Print*)&Serial);
  xbee.onTxStatusResponse(printErrorCb, (uintptr_t)(Print*)&Serial);
  xbee.onZBTxStatusResponse(printErrorCb, (uintptr_t)(Print*)&Serial);

  // These are called when an actual packet received
  //xbee.onZBRxResponse(zbReceive, (uintptr_t)(Print*)&Serial);

  // Print any unhandled response with proper formatting
  xbee.onOtherResponse(printResponseCb, (uintptr_t)(Print*)&Serial);

  // Enable this to print the raw bytes for _all_ responses before they
  // are handled
  //xbee.onResponse(printRawResponseCb, (uintptr_t)(Print*)&Serial);
}

void loop() {

  // Continuously let xbee read packets and call callbacks.
  //xbee.loop();

  uint8_t distance = sensor0.readRangeSingle();
  if( distance != lastDistance )
  {
    if( wobbleIgnore>0 )
    {
      if( distance > lastDistance  && (distance - lastDistance) < wobbleIgnore  )
        return;
      if( distance < lastDistance  && (lastDistance - distance) < wobbleIgnore  )
        return;
    }
    SendDistancePacket( distance );
    lastDistance = distance;
  }
}



void SendDistancePacket( uint8_t distance )
{
  Serial.print(F("SENDING distance: "));
  Serial.print(distance);
  Serial.println();

  distanceMessagePayload[0] = distance;
  distanceMessage.setFrameId(xbee.getNextFrameId());
  
  xbee.send(distanceMessage);

  /*// Send the command and wait up to N ms for a response.  xbee loop continues during this time.
  uint8_t status = xbee.sendAndWait(distanceMessage, 3000);
  if (status == 0)
  {
    Serial.println(F("SEND ACKNOWLEDGED"));

  } else { //Complain, but do not reset timeElapsed - so that a new packet comes in and tried again immedietly.
    Serial.print(F("SEND FAILED: "));
    printHex(status, 2);
    Serial.println();
    //flashSingleLed(LED_BUILTIN, 3, 500);
  }*/
}


// UTIL FUNCTIONS
void printHex(int num, int precision) {
     char tmp[16];
     char format[128];

     sprintf(format, "0x%%.%dX", precision);

     sprintf(tmp, format, num);
     Serial.print(tmp);
}



void SetSensorI2CAddresses()
{
  // Reset all connected sensors
  pinMode(enablePin0,OUTPUT);

  digitalWrite(enablePin0, LOW);
  
  delay(1000);
  
  SetSensorI2CAddress(0, enablePin0, &sensor0, address0 );

  delay(1000);
 
  Serial.println("Sensors ready! Start reading sensors in 3 seconds ...!");
  delay(3000);

}

void SetSensorI2CAddress( int i, int enablePin, VL6180X *sensor, int address )
{

  Serial.print("Start Sensor: ");
  Serial.print(i);
  Serial.print(" using pin ");
  Serial.print(enablePin);
  Serial.print(" as I2C Address ");
  Serial.print( address);
  Serial.println();
  
  digitalWrite(enablePin, HIGH);
  delay(50);
  sensor->init();
  sensor->configureDefault();
  sensor->setAddress(address);
  Serial.println(sensor->readReg(0x212),HEX); // read I2C address
  sensor->writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor->writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor->setTimeout(500);
  sensor->stopContinuous();
  sensor->setScaling(RANGE); // configure range or precision 1, 2 oder 3 mm
  delay(300);
  sensor->startInterleavedContinuous(100);
  delay(100);
}

