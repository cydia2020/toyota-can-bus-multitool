#include <SPI.h>
#include <mcp_can.h>

// ADAS Bus features, this should only be used when the Arduino is connected
// to the ADAS Bus, either at the front facing camera, the RADAR, the driving
// support ECU, or at the gateway
//
// SAFETY bus is the bus that connects to the steering sensor, the EPS, Airbag,
// and skid control ECU, it is suggested that you DO NOT CONNECT ANYTHING TO 
// THIS BUS, JUST USE THE ADAS Bus.
const bool TOYOTA_TNGA_ENABLE_COMBINATION_METER_DECODER = true; // ADAS Bus
const bool TOYOTA_PRIUS_RAV4_ENABLE_FAKE_IPA_ECU = false; // ADAS Bus, DO NOT ENABLE UNLESS YOU KNOW WHAT YOU ARE DOING, THIS WILL CAUSE IRREVERSIBLE CHANGES TO YOUR CAR'S COMPUTERS
const bool TOYOTA_AUTOMATIC_DOOR_LOCKER = true; // ADAS Bus or SAFETY Bus

// AVN Bus features, this should only be used when the Arduino is connected
// to the AVN Bus, either behind the navigation unit or the combination meter,
// or at the gateway
const bool TOYOTA_HYBRID_ENABLE_GEAR_PACKET_CONVERTER = true; // AVN Bus

// relay pin assignments
const int DIMMER_OUTPUT_RELAY_PIN = A0;

// Define CAN bus pins
const int SPI_CS_PIN = 10;

// Create MCP_CAN object
MCP_CAN CAN(SPI_CS_PIN);

// Define IPAS message IDs
const int MESSAGE_ID_292 = 0x292;
const int MESSAGE_ID_32E = 0x32E;
const int MESSAGE_ID_396 = 0x396;
const int MESSAGE_ID_43A = 0x43A;
const int MESSAGE_ID_43B = 0x43B;
const int MESSAGE_ID_497 = 0x497;
const int MESSAGE_ID_4CC = 0x4CC;

// Define IPAS message rates
const int MESSAGE_RATE_292 = 333;   // 3Hz = 333ms
const int MESSAGE_RATE_32E = 50;    // 20Hz = 50ms
const int MESSAGE_RATE_396 = 10;    // 100Hz = 10ms
const int MESSAGE_RATE_43A = 10;    // 100Hz = 10ms
const int MESSAGE_RATE_43B = 10;    // 100Hz = 10ms
const int MESSAGE_RATE_497 = 10;    // 100Hz = 10ms
const int MESSAGE_RATE_4CC = 10;    // 100Hz = 10ms

// Define IPAS message data
byte MESSAGE_DATA_292[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9E};
byte MESSAGE_DATA_32E[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte MESSAGE_DATA_396[] = {0xBD, 0x00, 0x00, 0x00, 0x60, 0x0F, 0x02, 0x00};
byte MESSAGE_DATA_43A[] = {0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte MESSAGE_DATA_43B[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte MESSAGE_DATA_497[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte MESSAGE_DATA_4CC[] = {0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Define last transmission times
unsigned long lastTransmissionTime_292 = 0;
unsigned long lastTransmissionTime_32E = 0;
unsigned long lastTransmissionTime_396 = 0;
unsigned long lastTransmissionTime_43A = 0;
unsigned long lastTransmissionTime_43B = 0;
unsigned long lastTransmissionTime_497 = 0;
unsigned long lastTransmissionTime_4CC = 0;

// Define variables to remember the last state of the relevant IDs
bool lastMeterDimmedState = false;
bool lastMeterSliderLowBrightnessState = false;
bool meterDimmed = false;
bool meterSliderLowBrightness = false;

// Define relay state machine
enum RelayState { RELAY_OFF, RELAY_ON };
RelayState relayState = RELAY_OFF;

// CAN IDs
#define SPEED_CAN_ID 180
#define ICE_GEAR_CAN_ID 295
#define HYBRID_GEAR_CAN_ID 956
#define BODY_CONTROL_STATE_1_CAN_ID 0x610
#define BODY_CONTROL_STATE_2_CAN_ID 0x620
#define DEBUG_CAN_ID 1872

// Define the bit positions for the gear signals
#define ORIG_GEAR_BIT 47
#define CONVERTED_GEAR_BIT 13

// Speed threshold in km/h for re-locking the doors
#define SPEED_THRESHOLD 15.0

// CAN door lock message data
unsigned char lockCommand[8] = {0x40, 0x05, 0x30, 0x11, 0x00, 0x40, 0x00, 0x00};
unsigned char unlockCommand[8] = {0x40, 0x05, 0x30, 0x11, 0x00, 0x80, 0x00, 0x00};

// Gear values
#define GEAR_P 0
#define GEAR_R 1
#define GEAR_N 2
#define GEAR_D 3

// Variables to keep track of door and gear status
bool doorsOpenedInDrive = false;
bool doorsLocked = false;
unsigned char prevGearValue = GEAR_P;

// Variables to calculate deceleration
unsigned long prevSpeedTime = 0;
unsigned int prevSpeedValue = 0;

// Helper function to convert the original gear value to the desired format
uint8_t convertGear(uint8_t originalGear)
{
  switch (originalGear) {
    case 0: return 32;  // P -> 32
    case 1: return 16;  // R -> 16
    case 2: return 8;   // N -> 8
    case 3: return 0;   // D -> 0
    case 4: return 0;   // B (Mapped to D) -> 0
    default: return 0;  // Unknown value, default to 0
  }
}

void setup()
{
  pinMode(DIMMER_OUTPUT_RELAY_PIN, OUTPUT);
  digitalWrite(DIMMER_OUTPUT_RELAY_PIN, LOW);

  // Initialize CAN bus
  CAN.begin(CAN_500KBPS);
}

void loop()
{
  if (TOYOTA_PRIUS_RAV4_ENABLE_FAKE_IPA_ECU)
  {
    // Send messages over the CAN bus
    sendIpasCANMessages();
  }

  if (TOYOTA_TNGA_ENABLE_COMBINATION_METER_DECODER)
  {
    // Check for relevant CAN messages to control the relay
    checkCombinationMeterDimmerCANMessages();

    // Update the relay state based on the state machine
    updateCombinationMeterDimmerRelayState();
  }

  if (TOYOTA_HYBRID_ENABLE_GEAR_PACKET_CONVERTER)
  {
    // Convert the hybrid gear CAN ID to the ICE format
    gearPacketConverter();
  }

  if (TOYOTA_AUTOMATIC_DOOR_LOCKER)
  {
    // lock and unlock doors automatically
    doorLockController();
  }
}

void sendIpasCANMessages()
{
  // Send message to 0x292 at 3Hz
  unsigned long currentTime = millis();
  if (currentTime - lastTransmissionTime_292 >= MESSAGE_RATE_292)
  {
    lastTransmissionTime_292 = currentTime;
    sendCANMessage(MESSAGE_ID_292, MESSAGE_DATA_292, sizeof(MESSAGE_DATA_292));
  }

  // Send message to 0x32E at 20Hz
  if (currentTime - lastTransmissionTime_32E >= MESSAGE_RATE_32E)
  {
    lastTransmissionTime_32E = currentTime;
    sendCANMessage(MESSAGE_ID_32E, MESSAGE_DATA_32E, sizeof(MESSAGE_DATA_32E));
  }

  // Send message to 0x396 at 100Hz
  if (currentTime - lastTransmissionTime_396 >= MESSAGE_RATE_396)
  {
    lastTransmissionTime_396 = currentTime;
    sendCANMessage(MESSAGE_ID_396, MESSAGE_DATA_396, sizeof(MESSAGE_DATA_396));
  }

  // Send message to 0x43A at 100Hz
  if (currentTime - lastTransmissionTime_43A >= MESSAGE_RATE_43A)
  {
    lastTransmissionTime_43A = currentTime;
    sendCANMessage(MESSAGE_ID_43A, MESSAGE_DATA_43A, sizeof(MESSAGE_DATA_43A));
  }

  // Send message to 0x43B at 100Hz
  if (currentTime - lastTransmissionTime_43B >= MESSAGE_RATE_43B)
  {
    lastTransmissionTime_43B = currentTime;
    sendCANMessage(MESSAGE_ID_43B, MESSAGE_DATA_43B, sizeof(MESSAGE_DATA_43B));
  }

  // Send message to 0x497 at 100Hz
  if (currentTime - lastTransmissionTime_497 >= MESSAGE_RATE_497)
  {
    lastTransmissionTime_497 = currentTime;
    sendCANMessage(MESSAGE_ID_497, MESSAGE_DATA_497, sizeof(MESSAGE_DATA_497));
  }

  // Send message to 0x4CC at 100Hz
  if (currentTime - lastTransmissionTime_4CC >= MESSAGE_RATE_4CC)
  {
    lastTransmissionTime_4CC = currentTime;
    sendCANMessage(MESSAGE_ID_4CC, MESSAGE_DATA_4CC, sizeof(MESSAGE_DATA_4CC));
  }
}

void gearPacketConverter()
{
  // Check if there is a message available in the CAN buffer
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    // Read the incoming CAN message
    uint8_t len;
    uint8_t buf[8];
    CAN.readMsgBuf(&len, buf);

    // Check if the incoming message is for the original signal
    if (buf[0] == ICE_GEAR_CAN_ID && len == 8) {
      // Extract the relevant data from the original signal
      uint8_t originalGear = buf[ORIG_GEAR_BIT / 8] & 0x0F;

      // Convert the original gear value to the desired format
      uint8_t convertedGear = convertGear(originalGear);

      // Create the converted data message with only the GEAR signal
      uint8_t convertedData[8] = {0};
      convertedData[CONVERTED_GEAR_BIT / 8] = convertedGear;

      // Send the converted data back to the CAN network with the converted CAN ID
      sendCANMessage(HYBRID_GEAR_CAN_ID, convertedData, sizeof(convertedData));
    }
  }
}

void doorLockController() {
  // Read the gear value from CAN bus
  unsigned char canData[8];
  unsigned char len = 0;

  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&len, canData);

    if (len > 2) {
      if (canData[0] == HYBRID_GEAR_CAN_ID) {
        unsigned char gearValue = (canData[2] >> 4) & 0x0F;

        // Check if the gear has changed
        if (gearValue != prevGearValue) {
          if (gearValue == GEAR_D) {
            doorsOpenedInDrive = checkDoorsOpened();
            if (doorsOpenedInDrive && !doorsLocked) {
              // Lock the doors if they were opened while in "D" gear and not already locked
              sendCANMessage(DEBUG_CAN_ID, lockCommand, sizeof(lockCommand));
              doorsLocked = true;
            }
          } else if (gearValue == GEAR_P) {
            sendCANMessage(DEBUG_CAN_ID, unlockCommand, sizeof(unlockCommand));
            doorsLocked = false; // Reset the lock state
          }

          // Update the previous gear value
          prevGearValue = gearValue;
        }
      } else if (canData[0] == SPEED_CAN_ID) {
        // Check the vehicle speed when the speed packet is received
        unsigned int speedValue = (canData[5] << 8) | canData[6];
        float speed_kph = speedValue * 0.01;

        // Check if the doors were opened in "D" gear and the vehicle speed is above the threshold
        if (doorsOpenedInDrive && speed_kph > SPEED_THRESHOLD && !doorsLocked) {
          // Lock the doors again if conditions are met and not already locked
          sendCANMessage(DEBUG_CAN_ID, lockCommand, sizeof(lockCommand));
          doorsLocked = true;
        }

        // Check for deceleration if the vehicle speed is above 15 km/h
        if (speed_kph > SPEED_THRESHOLD) {
          unsigned long currentSpeedTime = millis();
          unsigned int currentSpeedValue = speedValue;

          // Calculate acceleration (change in speed over time)
          float acceleration = (currentSpeedValue - prevSpeedValue) * 0.01 / ((currentSpeedTime - prevSpeedTime) * 0.001);
          
          // Check if deceleration exceeds 1G (9.81 m/s^2)
          if (acceleration < -9.81) {
            // Unlock the doors if deceleration exceeds 1G
            sendCANMessage(DEBUG_CAN_ID, unlockCommand, sizeof(unlockCommand));
            doorsLocked = false; // Reset the lock state
          }

          // Update the previous speed and time values
          prevSpeedValue = currentSpeedValue;
          prevSpeedTime = currentSpeedTime;
        }
      }
    }
  }
}

// Function to check if any of the doors were opened while in "D" gear
bool checkDoorsOpened() {
  unsigned char canData[8];
  unsigned char len = 0;

  // Read the body control state message
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&len, canData);
    if (len > 2 && canData[0] == BODY_CONTROL_STATE_2_CAN_ID) {
      // Check if any of the doors are open
      bool doorOpen_FL = (canData[2] >> 5) & 0x01;
      bool doorOpen_RL = (canData[2] >> 2) & 0x01;
      bool doorOpen_RR = (canData[2] >> 3) & 0x01;
      bool doorOpen_FR = (canData[2] >> 4) & 0x01;

      return doorOpen_FL || doorOpen_RL || doorOpen_RR || doorOpen_FR;
    }
  }

  return false;
}

// function to send our CAN message
void sendCANMessage(int messageId, byte* data, byte dataSize)
{
  // Send CAN message
  CAN.sendMsgBuf(messageId, 0, dataSize, data);
}

// function to check if the combination meter is either automatically dimmed or forcefully dimmed
void checkCombinationMeterDimmerCANMessages()
{
  unsigned char len = 0;
  unsigned char buf[8];

  while (CAN_MSGAVAIL == CAN.checkReceive())
  {
    CAN.readMsgBuf(&len, buf);

    if (len == 8 && CAN.getCanId() == BODY_CONTROL_STATE_2_CAN_ID) { // auto dimming function
      meterDimmed = bitRead(buf[4], 6);
    }

    if (len == 8 && CAN.getCanId() == BODY_CONTROL_STATE_1_CAN_ID) { // forced dimming function, occurs when the brightness slider is all the way down
      meterSliderLowBrightness = bitRead(buf[4], 5);
    }
  }
}


void updateCombinationMeterDimmerRelayState()
{
  static bool lastRelayState = LOW; // Initial relay state is OFF (LOW)

  // Check if the relay state should change
  relayState = (meterDimmed || meterSliderLowBrightness) ? RELAY_ON : RELAY_OFF;

  // Update the relay only when there's a change in its state
  if (relayState != lastRelayState)
  {
    digitalWrite(DIMMER_OUTPUT_RELAY_PIN, relayState == RELAY_ON ? HIGH : LOW);
    lastRelayState = relayState;
  }
}