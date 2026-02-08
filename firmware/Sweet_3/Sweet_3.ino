/*
 * 16n Faderbank Firmware
 * (c) 2017,2018,2020 by Brian Crabtree, Sean Hellfritsch, Tom Armitage, and Brendon Cassidy
 * MIT License
 */

/*
 * NOTES:
 * - Hardware MIDI is on pin 1
 * - You **must** also compile this with Tools->USB type set to MIDI or MIDI/Serial (for debugging)
 * - You also should overclock to 120MHz to make it as snappy as possible
 */

/*
 * Most configuration now hpapens via online editor.
 * config.h is mainly for developer configuration.
 */
// modified version for the Tesseract Modular Sweet Sixteen by Mangu Díaz
// changes from the original 16n 2.0.1  firmware from Tom Armitage:
// - calibration of the max fader value is now possible when 'rotated' is active
// - channel numbers are not inverted when 'rotate' is active
// - added pitch bend option (when midiCC 127 is selected)

#include <CD74HC4067.h>
#include <EEPROM.h>
#include <i2c_t3.h>
#include <MIDI.h>
#include <ResponsiveAnalogRead.h>

#include "config.h"
#include "TxHelper.h"

// wrap code to be executed only under DEBUG conditions in D()
#ifdef DEBUG
#define D(x) x
#else
#define D(x)
#endif

#define TO_TR 0x00
#define TO_TR_TOG 0x01
#define TO_TR_TIME 0x02
#define TO_TR_TIME_S 0x03
#define TO_TR_TIME_M 0x04
#define TO_TR_PULSE 0x05
#define TO_TR_POL 0x06

#define TO_CV 0x10
#define TO_CV_SET 0x11
#define TO_CV_SLEW 0x12
#define TO_CV_SLEW_S 0x13
#define TO_CV_SLEW_M 0x14
#define TO_CV_OFF 0x15

// panel led and function button pins:
const int  Led_ = 12;
const int  Butt = 15;
// variables for the functon button
bool _Butt ;
bool old_Butt = HIGH;

MIDI_CREATE_DEFAULT_INSTANCE();

// loop helpers
int i, temp;

// midi write helpers
int q, shiftyTemp, notShiftyTemp, lastMidiActivityAt;
int midiDirty = 0;
const int midiFlashDuration = 50;
int ledPin = 13;

// the storage of the values; current is in the main loop; last value is for midi output
int volatile currentValue[channelCount];
int lastMidiValue[channelCount];

// variables to hold configuration
int usbChannels[channelCount];
int trsChannels[channelCount];
int usbCCs[channelCount];
int trsCCs[channelCount];
int legacyPorts[channelCount]; // for V125 only
int flip;
int ledOn;
int ledFlash;
int i2cMaster;

int faderMin;
int faderMax;

// variables for i2c master mode
  // memory of the last unshifted value
  int lastValue[channelCount];

  // the i2c message buffer we are sending
  uint8_t messageBuffer[4];

  // temporary values
  uint16_t valueTemp;
  uint8_t device = 0;
  uint8_t port = 0;

  // master i2c specific stuff
  const int ansibleI2Caddress = 0x20;
  const int er301I2Caddress = 0x31;
  const int txoI2Caddress = 0x60;
  bool er301Present = false;
  bool ansiblePresent = false;
  bool txoPresent = false;


// the thing that smartly smooths the input
ResponsiveAnalogRead *analog[channelCount];

// mux config
CD74HC4067 mux(8, 7, 6, 5);
const int muxMapping[16] = {0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8};

// MIDI timers
IntervalTimer midiWriteTimer;
IntervalTimer midiReadTimer;
int midiInterval = 1000; // 1ms
bool shouldDoMidiRead = false;
bool shouldDoMidiWrite = false;
bool forceMidiWrite = false;

// helper values for i2c reading and future expansion
int activeInput = 0;
int activeMode = 0;

/*
 * The function that sets up the application
 */
void setup()
{

  D(Serial.println("16n Firmware Debug Mode"));

#ifdef BOOTDELAY // wait some time before continuing boot-up (default is none)
  delay(BOOTDELAY);
#endif

  pinMode (Butt, INPUT_PULLUP);
  pinMode (Led_, OUTPUT);
  // boot up pattern
  for ( i = 0; i < 3; i++)
  {
    digitalWrite(Led_, HIGH);
    delay(100);
    digitalWrite(Led_, LOW);
    delay(100);
  }

  checkDefaultSettings();

  loadSettingsFromEEPROM();
  i2cMaster = EEPROM.read(3) == 1;

  usbMIDI.setHandleSystemExclusive(processIncomingSysex);

  #ifdef V125
  // analog ports on the Teensy for the 1.25 board.
  if(flip) {
    int portsToAssign[] = {A15, A14, A13, A12, A11, A10, A9, A8, A7, A6, A5, A4, A3, A2, A1, A0};
    for(int i=0; i < channelCount; i++) {
      legacyPorts[i] = portsToAssign[i];
    }
  } else {
    int portsToAssign[] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15};
    for(int i=0; i < channelCount; i++) {
      legacyPorts[i] = portsToAssign[i];
    }
  }
  #endif

// initialize the TX Helper
#ifdef V125
  TxHelper::UseWire1(true);
#else
  TxHelper::UseWire1(false);
#endif
  TxHelper::SetPorts(16);
  TxHelper::SetModes(4);

  // set read resolution to teensy's 13 usable bits
  analogReadResolution(13);

  // initialize the value storage
  for (i = 0; i < channelCount; i++)
  {
    // analog[i] = new ResponsiveAnalogRead(0, false);

    analog[i] = new ResponsiveAnalogRead(0, true, .0001);
    analog[i]->setAnalogResolution(1 << 13);
    
    // ResponsiveAnalogRead is designed for 10-bit ADCs
    // meanining its threshold defaults to 4. Let's bump that for 
    // our 13-bit adc by setting it to 4 << (13-10)
    analog[i]->setActivityThreshold(32);

    currentValue[i] = 0;
    lastMidiValue[i] = 0;

    if(i2cMaster) {
      lastValue[i] = 0;
    }
  }

// i2c using the default I2C pins on a Teensy 3.2
if(i2cMaster) {

  D(Serial.println("Enabling i2c in MASTER mode"));

#ifdef V125
  Wire1.begin(I2C_MASTER, I2C_ADDRESS, I2C_PINS_29_30, I2C_PULLUP_EXT, 400000);
  Wire1.setDefaultTimeout(10000); // 10ms

  D(Serial.println ("Scanning I2C bus"));
  
  Wire1.begin();
  
  for (byte i = 8; i < 120; i++)
  {
    Wire1.beginTransmission (i);
    if (Wire1.endTransmission () == 0)
      {
        if(i == ansibleI2Caddress) {
          ansiblePresent = true;
          D(Serial.println ("Found ansible"));
        }

        if(i == txoI2Caddress) {
          txoPresent = true;
          D(Serial.println ("Found TXO"));
        }

        if(i == er301I2Caddress) {
          er301Present = true;
          D(Serial.println ("Found ER301"));
        }
      delay (1);  // maybe unneeded?
      } // end of good response
  } // end of for loop

  D(Serial.println ("I2C scan complete."));

#else
  Wire.begin(I2C_MASTER, I2C_ADDRESS, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  Wire.setDefaultTimeout(10000); // 10ms

  D(Serial.println ("Scanning I2C bus"));
  
  Wire.begin();
  
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
      {
        if(i == ansibleI2Caddress) {
          ansiblePresent = true;
          D(Serial.println ("Found ansible"));
        }

        if(i == txoI2Caddress) {
          txoPresent = true;
          D(Serial.println ("Found TXO"));
        }

        if(i == er301I2Caddress) {
          er301Present = true;
          D(Serial.println ("Found ER301"));
        }
      delay (1);  // maybe unneeded?
      } // end of good response
  } // end of for loop

  Serial.println ("I2C scan complete.");

#endif

} else {
  // non-master mode

  D(Serial.println("Enabling i2c enabled in SLAVE mode"));

#ifdef V125
  Wire1.begin(I2C_SLAVE, I2C_ADDRESS, I2C_PINS_29_30, I2C_PULLUP_EXT, 400000);
  Wire1.onReceive(i2cWrite);
  Wire1.onRequest(i2cReadRequest);
#else
  Wire.begin(I2C_SLAVE, I2C_ADDRESS, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  Wire.onReceive(i2cWrite);
  Wire.onRequest(i2cReadRequest);
#endif

}

  // turn on the MIDI party
  MIDI.begin();
  midiWriteTimer.begin(writeMidi, midiInterval);
  midiReadTimer.begin(readMidi, midiInterval);

  pinMode(ledPin, OUTPUT);

  if(ledOn) {
    digitalWrite(ledPin, HIGH);
  }
}

/*
 * The main read loop that goes through all of the sliders
 */
void loop()
{
  // this whole chunk makes the LED flicker on MIDI activity - 
  // and inverts that flicker if the power light is on.
  if (ledFlash) {
    if (millis() > (lastMidiActivityAt + midiFlashDuration)) {
      if(ledOn) {
        digitalWrite(ledPin, HIGH);
      } else {
        digitalWrite(ledPin, LOW);
      }
      midiDirty = 0;
    } else {
      if(ledOn) {
        digitalWrite(ledPin, LOW);
      } else {
        digitalWrite(ledPin, HIGH);
      }
    }
  } else {
    if(ledOn) {
      digitalWrite(ledPin, HIGH);
    } else {
      digitalWrite(ledPin, LOW);
    }
  }

  // read loop using the i counter
  for (i = 0; i < channelCount; i++)
  {
#ifdef V125
    temp = analogRead(legacyPorts[i]); // mux goes into A0
#else
    // set mux to appropriate channel
   // if(flip) {
   //   mux.channel(muxMapping[channelCount - i - 1]);
   // } else {
      mux.channel(muxMapping[i]);
   // }

    // read the value
    temp = analogRead(0); // mux goes into A0
#endif

    // put the value into the smoother
    analog[i]->update(temp);

    // read from the smoother, constrain (to account for tolerances), and map it
    temp = analog[i]->getValue();

    if(flip){ 
      temp = 8191 - temp; //       temp = faderMax - temp;
    }

    temp = constrain(temp, faderMin, faderMax);

    temp = map(temp, faderMin, faderMax, 0, 16383);

    // map and update the value
    currentValue[i] = temp;

  }

  if (shouldDoMidiRead)
  {
    doMidiRead();
    noInterrupts();
    shouldDoMidiRead = false;
    interrupts();
  }

  if (shouldDoMidiWrite)
  {
    doMidiWrite();
    noInterrupts();
    shouldDoMidiWrite = false;
    interrupts();
  }
}

/*
 * Tiny function called via interrupt
 * (it's important to catch inbound MIDI messages even if we do nothing with
 * them.)
 */
void readMidi()
{
  shouldDoMidiRead = true;
}

/*
 * Function called when shouldDoMidiRead flag is HIGH
 */

void doMidiRead()
{
  MIDI.read();
  usbMIDI.read();
}

/*
 * Tiny function called via interrupt
 */
void writeMidi()
{
  shouldDoMidiWrite = true;
}

/*
 * The function that writes changes in slider positions out the midi ports
 * Called when shouldDoMidiWrite flag is HIGH
 */
void doMidiWrite()
{
  // write loop using the q counter (
  // (can't use i or temp cuz this might interrupt the reads)
  for (q = 0; q < channelCount; q++)
  {
    notShiftyTemp = currentValue[q];

    // shift for MIDI precision (0-127)
    shiftyTemp = notShiftyTemp >> 7;

    // if there was a change in the midi value
    // if ((shiftyTemp != lastMidiValue[q]) || forceMidiWrite)
    if (( shiftyTemp != lastMidiValue[q]) || ((( usbCCs[q] == 127 ) || ( trsCCs[q] == 127 )) && ( notShiftyTemp != lastValue[q] )) || forceMidiWrite )
    {
      if(ledFlash && !midiDirty) {
        lastMidiActivityAt = millis();
        midiDirty = 1;
      }

#ifdef PITCHBEND
      if ( usbCCs[q] == 127 )
      {
        usbMIDI.sendPitchBend((notShiftyTemp - 8192), usbChannels[q]);          
      }
      else
#endif        
      {          
        // send the message over USB and physical MIDI
        usbMIDI.sendControlChange(usbCCs[q], shiftyTemp, usbChannels[q]);
      }
#ifdef PITCHBEND
      if ( trsCCs[q] == 127 )
      { 
        MIDI.sendPitchBend((notShiftyTemp - 8192), trsChannels[q]);
      }
      else
#endif
      {
        // send the message over physical MIDI
        MIDI.sendControlChange(trsCCs[q], shiftyTemp, trsChannels[q]);
      }     

      // store the shifted value for future comparison
      lastMidiValue[q] = shiftyTemp;

      // D(Serial.printf("MIDI[%d]: %d\n", q, shiftyTemp));
    }

    if(i2cMaster) {

      // we send out to all three supported i2c slave devices
      // keeps the firmware simple :)

      if (notShiftyTemp != lastValue[q])
      {
        D(Serial.printf("i2c Master[%d]: %d\n", q, notShiftyTemp));

        // for 4 output devices
        port = q % 4;
        device = q / 4;

        // TXo
        if(txoPresent) {
          sendi2c(txoI2Caddress, device, 0x11, port, notShiftyTemp);
        }

        // ER-301
        if(er301Present) {
          sendi2c(er301I2Caddress, 0, TO_CV_SET, q, notShiftyTemp);
        }

        // ANSIBLE
        if(ansiblePresent) {
          sendi2c(0x20, device << 1, 0x06, port, notShiftyTemp);
        }

        lastValue[q] = notShiftyTemp;
      }
    }
  }
  forceMidiWrite = false;

  // button read to load & save presets etc:
  _Butt = digitalReadFast(Butt);

  if (_Butt != old_Butt) // change in the button
  {
    if (_Butt == true )
    {
      // button depressed, send all I2C values
      sendAllI2CValues();
    }
    old_Butt = _Butt;
  }
  
} // end of doMidiWrite

/*
 * Sends an i2c command out to a slave when running in master mode
 */
void sendi2c(uint8_t model, uint8_t deviceIndex, uint8_t cmd, uint8_t devicePort, int value)
{

  valueTemp = (uint16_t)value;
  messageBuffer[2] = valueTemp >> 8;
  messageBuffer[3] = valueTemp & 0xff;

#ifdef V125
  Wire1.beginTransmission(model + deviceIndex);
  messageBuffer[0] = cmd;
  messageBuffer[1] = (uint8_t)devicePort;
  Wire1.write(messageBuffer, 4);
  Wire1.endTransmission();
#else
  Wire.beginTransmission(model + deviceIndex);
  messageBuffer[0] = cmd;
  messageBuffer[1] = (uint8_t)devicePort;
  Wire.write(messageBuffer, 4);
  Wire.endTransmission();
#endif
}

/*
 * The function that responds to a command from i2c.
 * In the first version, this simply sets the port to be read from.
 */
void i2cWrite(size_t len)
{

  D(Serial.printf("i2c Write (%d)\n", len));

  // parse the response
  TxResponse response = TxHelper::Parse(len);

  // true command our setting of the input for a read?
  if (len == 1)
  {

    // use a helper to decode the command
    TxIO io = TxHelper::DecodeIO(response.Command);

    D(Serial.printf("Port: %d; Mode: %d [%d]\n", io.Port, io.Mode, response.Command));

    // this is the single byte that sets the active input
    activeInput = io.Port;
    activeMode = io.Mode;
  }
  else
  {
    // act on the command
    actOnCommand(response.Command, response.Output, response.Value);
  }
}

/*
 * The function that responds to read requests over i2c.
 * This uses the port from the write request to determine which slider to send.
 */
void i2cReadRequest()
{

  D(Serial.print("i2c Read\n"));

  // get and cast the value
  uint16_t shiftReady = 0;
  switch (activeMode)
  {
  case 1:
    shiftReady = (uint16_t)currentValue[activeInput];
    break;
  case 2:
    shiftReady = (uint16_t)currentValue[activeInput];
    break;
  default:
    shiftReady = (uint16_t)currentValue[activeInput];
    break;
  }

  D(Serial.printf("delivering: %d; value: %d [%d]\n", activeInput, currentValue[activeInput], shiftReady));

// send the puppy as a pair of bytes
#ifdef V125
  Wire1.write(shiftReady >> 8);
  Wire1.write(shiftReady & 255);
#else
  Wire.write(shiftReady >> 8);
  Wire.write(shiftReady & 255);
#endif
}

/*
 * Future function if we add more i2c capabilities beyond reading values.
 */
void actOnCommand(byte cmd, byte out, int value) {}


void sendAllI2CValues()
{
  if(i2cMaster) {
    // for 4 output devices
    port = q % 4;
    device = q / 4;
    for (q = 0; q < channelCount; q++)
    {
      if(txoPresent) {
        sendi2c(txoI2Caddress, device, 0x11, port, lastValue[q]);
      }

      if(er301Present) {
        sendi2c(er301I2Caddress, 0, TO_CV_SET, q, lastValue[q]);
      }

      if(ansiblePresent) {
        sendi2c(0x20, device << 1, 0x06, port, lastValue[q]);
      }
    }
  }
}