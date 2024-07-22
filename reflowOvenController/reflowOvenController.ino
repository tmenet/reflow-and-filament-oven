/*******************************************************************************
 * Title: Reflow Oven Controller
 * Version: 1.20
 * Date: 11/12/2013
 * Company: Rocket Scream Electronics
 * Author: Lim Phang Moh modified by Albert R Tejera
 * Website: www.rocketscream.com
 * 
 * Brief
 * =====
 * This is an example firmware for our Arduino compatible reflow oven controller. 
 * The reflow curve used in this firmware is meant for lead-free profile or lead 
 * profile (in parenthesis below) Enable Lead profile by simply commenting out the
 * //#define LEAD_FREE
 *
 *
 * Temperature (Degree Celcius)                 Magic Happens Here!
 * 245 (225)-|                                               x  x  
 *           |                                            x        x
 *           |                                         x              x
 *           |                                      x                    x
 * 200 (180)-|                                   x                          x
 *           |                              x    |                          |   x   
 *           |                         x         |                          |       x
 *           |                    x              |                          |
 *       150-|               x                   |                          |
 *           |             x |                   |                          |
 *           |           x   |                   |                          | 
 *           |         x     |                   |                          | 
 *           |       x       |                   |                          | 
 *           |     x         |                   |                          |
 *           |   x           |                   |                          |
 *       30 -| x             |                   |                          |
 *           |<  60 - 90 s  >|<    90 - 120 s   >|<       90 - 120 s       >|
 *           | Preheat Stage |   Soaking Stage   |       Reflow Stage       | Cool
 *        0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
 *                                                                Time (Seconds)
 *
 * This firmware owed very much on the works of other talented individuals as
 * follows:
 * ==========================================
 * Brett Beauregard (www.brettbeauregard.com)
 * ==========================================
 * Author of Arduino PID library. On top of providing industry standard PID 
 * implementation, he gave a lot of help in making this reflow oven controller 
 * possible using his awesome library.
 *
 * ==========================================
 * Limor Fried of Adafruit (www.adafruit.com)
 * ==========================================
 * Author of Arduino MAX6675 library. Adafruit has been the source of tonnes of
 * tutorials, examples, and libraries for everyone to learn.
 *
 * Disclaimer
 * ==========
 * Dealing with high voltage is a very dangerous act! Please make sure you know
 * what you are dealing with and have proper knowledge before hand. Your use of 
 * any information or materials on this reflow oven controller is entirely at 
 * your own risk, for which we shall not be liable. 
 *
 * Licences
 * ========
 * This reflow oven controller hardware and firmware are released under the 
 * Creative Commons Share Alike v3.0 license
 * http://creativecommons.org/licenses/by-sa/3.0/ 
 * You are free to take this piece of code, use it and modify it. 
 * All we ask is attribution including the supporting libraries used in this 
 * firmware. 
 *
 * Required Libraries
 * ==================
 * - Arduino PID Library: 
 *   >> https://github.com/br3ttb/Arduino-PID-Library
 * - MAX31855 Library (for board v1.60 & above): 
 *   >> https://github.com/rocketscream/MAX31855
 * - MAX6675 Library (for board v1.50 & below):
 *   >> https://github.com/adafruit/MAX6675-library
 *
 * Revision  Description
 * ========  ===========
 * 1.4       Added Filament Dryer feature and changed splas to indicate Multi Oven and prompt for a choice
 * 1.3       Changed hardware defines for breadboard with 16X2 LCD and added profile for LEAD solder
 * 1.20			Adds supports for v1.60 (and above) of Reflow Oven Controller 
 *           Shield:
 *					  - Uses MAX31855KASA+ chip and pin reassign (allowing A4 & A5 (I2C)
 *             to be used for user application).
 *					  - Uses analog based switch (allowing D2 & D3 to be used for user 
 *						  application).	
 *						Adds waiting state when temperature too hot to start reflow process.
 *						Corrected thermocouple disconnect error interpretation (MAX6675).
 * 1.10      Arduino IDE 1.0 compatible.
 * 1.00      Initial public release.
 *******************************************************************************/

//  PLA (Polylactic Acid)
// Temperature: 40-45°C (104-113°F)
// Time: 4-6 hours

// ABS (Acrylonitrile Butadiene Styrene)
// Temperature: 80-85°C (176-185°F)
// Time: 2-4 hours

// PETG (Polyethylene Terephthalate Glycol)
// Temperature: 65-70°C (149-158°F)
// Time: 4-6 hours

// TPU (Thermoplastic Polyurethane)
// Temperature: 40-50°C (104-122°F)
// Time: 4-6 hours

// Nylon
// Temperature: 70-90°C (158-194°F)
// Time: 8-12 hours

// PC (Polycarbonate)
// Temperature: 80-100°C (176-212°F)
// Time: 6-8 hours

// PVA (Polyvinyl Alcohol)
// Temperature: 45-55°C (113-131°F)
// Time: 4-6 hours

// HIPS (High Impact Polystyrene)
// Temperature: 65-75°C (149-167°F)
// Time: 3-5 hours

// PEEK (Polyether Ether Ketone)
// Temperature: 120-150°C (248-302°F)
// Time: 4-6 hours

// ASA (Acrylonitrile Styrene Acrylate)
// Temperature: 80-85°C (176-185°F)
// Time: 4-6 hours

// PP (Polypropylene)
// Temperature: 80-100°C (176-212°F)
// Time: 4-6 hours





// Comment either one the following #define to select your board revision
// Newer board version starts from v1.60 using MAX31855KASA+ chip
#define USE_MAX31855


// Uncomment below for lead solder
//#define LEAD_FREE


// ***** INCLUDES *****
#include <LiquidCrystal.h>
#ifdef USE_MAX31855
#include <MAX31855.h>
#else
#include <max6675.h>
#endif
#include <PID_v1.h>

// ***** TYPE DEFINITIONS *****

typedef enum OVEN_STATE {
  FILAMENT_OVEN,
  REFLOW_OVEN
} ovenState_t;

typedef enum REFLOW_STATE {
  REFLOW_STATE_IDLE,
  REFLOW_STATE_PREHEAT,
  REFLOW_STATE_SOAK,
  REFLOW_STATE_REFLOW,
  REFLOW_STATE_COOL,
  REFLOW_STATE_COMPLETE,
  REFLOW_STATE_TOO_HOT,
  REFLOW_STATE_ERROR
} reflowState_t;

typedef enum FILAMENT_STATE {
  FILAMENT_STATE_ON,
  FILAMENT_STATE_COOL,
  FILAMENT_STATE_COMPLETE,
  FILAMENT_STATE_TOO_HOT,
   FILAMENT_STATE_IDLE,
  FILAMENT_STATE_ERROR
} filamentState_t;

typedef enum REFLOW_STATUS {
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
} reflowStatus_t;

typedef enum SWITCH {
  SWITCH_NONE,
  SWITCH_1,
  SWITCH_2
} switch_t;

typedef enum DEBOUNCE_STATE {
  DEBOUNCE_STATE_IDLE,
  DEBOUNCE_STATE_CHECK,
  DEBOUNCE_STATE_RELEASE
} debounceState_t;

// ***** CONSTANTS *****

#define TEMPERATURE_ROOM 50       // Safe Starting Temp
#define TEMPERATURE_SOAK_MIN 150  // Beginning Soaking Stage Temperature

#ifdef LEAD_FREE
#define TEMPERATURE_SOAK_MAX 200    // End Soaking Stage Temperature
#define TEMPERATURE_REFLOW_MAX 250  // Absolute Peak temperature (actually stops 5 deg C before this value)
#else
#define TEMPERATURE_SOAK_MAX 180
#define TEMPERATURE_REFLOW_MAX 225
#endif

#define TEMPERATURE_COOL_MIN 100   // Ending temperature DONE
#define SENSOR_SAMPLING_TIME 1000  // milliseconds between samples
#define SOAK_TEMPERATURE_STEP 5    // Temperature increase per Micro Period during Soaking Stage
#define SOAK_MICRO_PERIOD 9000     // Period of time (milliseconds) to hold temp step so (TEMPERATURE_SOAK_MAX-TEMPERATURE_SOAK_MIN  / SOAK_TEMPERATURE_STEP) * SOAK_MICRO_PERIOD = Soaking time


#define DEBOUNCE_PERIOD_MIN 50

// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 100
#define PID_KI_PREHEAT 0.025
#define PID_KD_PREHEAT 20
// ***** SOAKING STAGE *****
#define PID_KP_SOAK 300
#define PID_KI_SOAK 0.05
#define PID_KD_SOAK 250
// ***** REFLOW STAGE *****
#define PID_KP_REFLOW 300
#define PID_KI_REFLOW 0.05
#define PID_KD_REFLOW 350
#define PID_SAMPLE_TIME 1000

typedef enum FLMT_TYPE {

  PLA = 0,
  ABS,
  PETG,
  TPU,
  Nylon,
  PC,
  PVA,
  HIPS,
  PEEK,
  ASA,
  PP

} flmtType_t;

char flmtName[11][6] = { "PLA", "ABS", "PETG", "TPU", "Nylon", "PC", "PVA", "HIPS", "PEEK", "ASA", "PP" };
int flmtTemp[11] = { 45, 85, 70, 50, 90, 100, 55, 75, 150, 85, 100 };
int flmtHours[11] = { 6, 4, 6, 6, 12, 8, 6, 5, 6, 6, 6 };

// ***** LCD MESSAGES *****
const char* lcdMessagesReflowStatus[] = {
  "Ready",
  "Pre-heat",
  "Soak",
  "Reflow",
  "Cool",
  "Complete",
  "Wait,hot",
  "Error"
};


const char* lcdMessagesfilamentStatus[] = {
"On",
"Cool",
"Complete",
"Wait,HOT",
"Idle"
};

// ***** DEGREE SYMBOL FOR LCD *****
unsigned char degree[8] = {
  140, 146, 146, 140, 128, 128, 128, 128
};

// ***** PIN ASSIGNMENT *****

int ssrPin = 2;
int thermocoupleSOPin = 3;
int thermocoupleCSPin = 4;
int thermocoupleCLKPin = 5;
int lcdRsPin = 7;
int lcdEPin = 8;
int lcdD4Pin = 9;
int lcdD5Pin = 10;
int lcdD6Pin = 11;
int lcdD7Pin = 12;
int ledRedPin = 13;
int buzzerPin = 6;
int rightswitch = A2;
int leftswitch = A4;
int upswitch = A5;
int downswitch = A3;


// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
int windowSize;
unsigned long windowStartTime;
unsigned long nextCheck;
unsigned long nextRead;
unsigned long timerSoak;
unsigned long buzzerPeriod;

unsigned long lastmillis;


// filament oven controller function state machine state variable
filamentState_t filamentState;
//filament type selected
flmtType_t flmtType;
// oven controller function state machine state variable
ovenState_t ovenState;
// Reflow oven controller state machine state variable
reflowState_t reflowState;
// Reflow oven controller status
reflowStatus_t reflowStatus;
// Switch debounce state machine state variable
debounceState_t debounceState;
// Switch debounce timer
long lastDebounceTime;
// Switch press status
switch_t switchStatus;
// Seconds timer
int timerSeconds;
// menu state points to filamentType
int flmtTypeCount = 0;
int menuState = 0;
bool exitMenu = 0;
int secondsTimer;
// Specify PID control interface
PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
// Specify LCD interface
LiquidCrystal lcd(lcdRsPin, lcdEPin, lcdD4Pin, lcdD5Pin, lcdD6Pin, lcdD7Pin);
// Specify MAX6675 thermocouple interface

MAX31855 thermocouple(thermocoupleSOPin, thermocoupleCSPin,
                      thermocoupleCLKPin);

void setup() {
  // SSR pin initialization to ensure reflow oven is off
  digitalWrite(ssrPin, LOW);
  pinMode(ssrPin, OUTPUT);

  // Buzzer pin initialization to ensure annoying buzzer is off
  digitalWrite(buzzerPin, LOW);
  pinMode(buzzerPin, OUTPUT);

  // LED pins initialization and turn on upon start-up (active low)
  digitalWrite(ledRedPin, LOW);
  pinMode(ledRedPin, OUTPUT);


  // Start-up splash ****************************************************************
  digitalWrite(buzzerPin, HIGH);
  lcd.begin(8, 2);
  lcd.createChar(0, degree);
  lcd.clear();

#ifdef LEAD_FREE
  lcd.print("ROHS Profile");
#else
  lcd.print("LEAD Profile");
#endif
  lcd.setCursor(0, 1);
  lcd.print("Multi Oven 1.4");
  digitalWrite(buzzerPin, LOW);
  delay(5000);
  lcd.clear();


//  **************************** SELECTION MENU **************************************************************************
  lcd.setCursor(0, 1);
  lcd.print("<Filmnt Reflow> ");

  while ((analogRead(leftswitch)) && (analogRead(rightswitch))) {
    delay(100);
    if (!analogRead(leftswitch)) {
      lcd.clear();
      ovenState = FILAMENT_OVEN;
      lcd.setCursor(0, 1);
      lcd.print(" Filament OVEN");
    }
    if (!analogRead(rightswitch)) {
      lcd.clear();
      ovenState = REFLOW_OVEN;
      lcd.setCursor(0, 1);
      lcd.print("Reflow OVEN ");
    }
  }
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("<     PLA     >");
// ***********************************  FILAMENT SELECTION **********************************************************************
  if (ovenState == FILAMENT_OVEN) {  // letrs select the filament parameters

    flmtTypeCount = 0;
    exitMenu = 0;
    while (!exitMenu) {
      // tests to exit menu
      if (!analogRead(upswitch)) {
        delay(100);
        if (!analogRead(upswitch)) exitMenu = 1;
      }
      if (!analogRead(downswitch)) {
        delay(100);
        if (!analogRead(downswitch)) exitMenu = 1;
      }

      if (!analogRead(rightswitch)) {
        delay(100);
        if (!analogRead(rightswitch)) {
          flmtTypeCount++;
          if (flmtTypeCount > 10) {
            flmtTypeCount = 0;
          }
          lcd.setCursor(6, 1);
          lcd.print("     ");
          lcd.setCursor(6, 1);
          lcd.print(flmtName[flmtTypeCount]);
          delay(400);
        }
      }
      if (!analogRead(leftswitch)) {
        delay(100);
        if (!analogRead(leftswitch)) {
          flmtTypeCount--;
          if (flmtTypeCount < 0) {
            flmtTypeCount = 10;
          }
          lcd.setCursor(6, 1);
          lcd.print("     ");
          lcd.setCursor(6, 1);
          lcd.print(flmtName[flmtTypeCount]);
          delay(400);
        }
      }
    }

    secondsTimer = 300;//flmtHours[flmtTypeCount] * 3600;
    setpoint = 80; //flmtTemp[flmtTypeCount];

    lcd.setCursor(0, 0);
    lcd.print(secondsTimer / 3600);
    lcd.print("Hrs @  ");
    lcd.print(setpoint);
    lcd.print(" C");
    lastmillis = millis();
    delay(3000);
  }

  // Serial communication at 57600 bps ************************************************
  Serial.begin(57600);

  // Turn off LED (active low)
  digitalWrite(ledRedPin, HIGH);

  // Set window size
  windowSize = 2000;
  // Initialize time keeping variable
  nextCheck = millis();
  // Initialize thermocouple reading variable
  nextRead = millis();
}

void loop() {
  // Current time
  unsigned long now;

  // Time to read thermocouple? ***********************************************
  if (millis() > nextRead) {
    // Read thermocouple next sampling period
    nextRead += SENSOR_SAMPLING_TIME;
    // Read current temperature

    input = thermocouple.readThermocouple(CELSIUS);

    if ((input == FAULT_OPEN) || (input == FAULT_SHORT_GND) || (input == FAULT_SHORT_VCC))

    {
      // Illegal operation
      reflowState = REFLOW_STATE_ERROR;
      reflowStatus = REFLOW_STATUS_OFF;
    }
  }

  // count off seconds *****************************************************
  if (millis() > nextCheck) {
    // Check input in the next seconds
    nextCheck += 1000;
    // If reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON || ovenState == FILAMENT_OVEN) {
      // Toggle red LED as system heart beat
      digitalWrite(ledRedPin, !(digitalRead(ledRedPin)));
      // Increase seconds timer for reflow curve analysis
      timerSeconds++;
      // Send temperature and time stamp to serial
      Serial.print(timerSeconds);
      Serial.print(" ");
      Serial.print(setpoint);
      Serial.print(" ");
      Serial.print(input);
      Serial.print(" ");
      Serial.println(output);
    } else {
      // Turn off red LED
      digitalWrite(ledRedPin, HIGH);
    }
    //****************** REFLOW OUTPUT
    if (ovenState == REFLOW_OVEN) {
      // Clear LCD
      lcd.clear();
      // Print current system state ******************************************************
      lcd.print(lcdMessagesReflowStatus[reflowState]);
      // Move the cursor to the 2 line
      lcd.setCursor(0, 1);
      if (ovenState == FILAMENT_OVEN) {
        // If currently in error state
        if (reflowState == REFLOW_STATE_ERROR) {
          // No thermocouple wire connected
          lcd.print("TC Error!");
        } else {
          // Print current temperature ******************************************************
          lcd.print(input);  // temperature was loaded into input

#if ARDUINO >= 100
          // Print degree Celsius symbol
          lcd.write((uint8_t)0);
#else
          // Print degree Celsius symbol
          lcd.print(0, BYTE);
#endif
          lcd.print("C ");
        }
      }
    }

    // *****************FILAMENT OUTPUT *********************************************************
    if (millis() > lastmillis + 1000) {
      lastmillis = millis();

      lcd.print("                ");
      lcd.setCursor(0, 0);
      lcd.print((double)(secondsTimer - timerSeconds) / 3600);
      lcd.print(" Hrs ");
      lcd.print(input);
      // Print degree Celsius symbol
      lcd.write((uint8_t)0);
      lcd.print("C");

      if (filamentState = FILAMENT_STATE_ON) {
              lcd.setCursor(1, 1);
        lcd.print(lcdMessagesfilamentStatus[reflowState]);
      }




      // *****************************************FILAMENT STATE Machine ***************************************************************************************************************
      switch (filamentState) {

        case FILAMENT_STATE_ON:  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< xxx
          reflowStatus = REFLOW_STATUS_ON;
          reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
          // Ramp down to minimum cooling temperature
          if (timerSeconds == secondsTimer) {
            // Proceed to cooling state
            filamentState = FILAMENT_STATE_COOL;
            setpoint = 0;
          }

          break;

        case FILAMENT_STATE_COOL:  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< xxx
          setpoint = TEMPERATURE_COOL_MIN;
          // If minimum cool temperature is achieve
          if (input <= TEMPERATURE_COOL_MIN) {
            // Retrieve current time for buzzer usage
            buzzerPeriod = millis() + 1000;
            // Turn on buzzer and green LED to indicate completion

            digitalWrite(buzzerPin, HIGH);
            // Turn off reflow process
            reflowStatus = REFLOW_STATUS_OFF;
            // Proceed to reflow Completion state
            filamentState = FILAMENT_STATE_COMPLETE;
          }
          break;

        case FILAMENT_STATE_COMPLETE:  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< xxx
          if (millis() > buzzerPeriod) {
            // Turn off buzzer and green LED
            digitalWrite(buzzerPin, LOW);

            // Reflow process ended
            filamentState = FILAMENT_STATE_IDLE;
          }
          break;

        case FILAMENT_STATE_TOO_HOT:  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< xxx
          // If oven temperature drops below room temperature
          if (input < TEMPERATURE_ROOM) {
            // Ready to reflow
            filamentState = FILAMENT_STATE_IDLE;
          }
          break;
        case FILAMENT_STATE_IDLE:  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< xxx
          // If oven temperature is still above room temperature don't allow start reflow
          if (input >= TEMPERATURE_ROOM) {
            filamentState = FILAMENT_STATE_TOO_HOT;
          } else {
            // If switch is pressed to start filament  process
            while (!exitMenu) {
              // tests to exit menu
              if (!analogRead(upswitch)) {
                delay(100);
                if (!analogRead(upswitch)) exitMenu = 1;
              }
              if (!analogRead(downswitch)) {
                delay(100);
                if (!analogRead(downswitch)) exitMenu = 1;
              }
            }
            // If switch is pressed to start reflow process

            Serial.println("Time Setpoint Input Output");
            // Intialize seconds timer for serial debug information
            timerSeconds = 0;
            // Initialize PID control window starting time
            windowStartTime = millis();
            // Ramp up to minimum soaking temperature
            setpoint = TEMPERATURE_SOAK_MIN;
            // Tell the PID to range between 0 and the full window size
            reflowOvenPID.SetOutputLimits(0, windowSize);
            reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
            // Turn the PID on
            reflowOvenPID.SetMode(AUTOMATIC);
            // Proceed to preheat stage
            filamentState = FILAMENT_STATE_ON;
          }
          break;
      }
    }
  } else  //we are a reflow oven

  {
    // Reflow oven controller state machine  ********************************************************************************************************************** REFLOW OVEN STATE MACHINE
    switch (reflowState) {
      case REFLOW_STATE_IDLE:  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< xxx
        // If oven temperature is still above room temperature don't allow start reflow
        if (input >= TEMPERATURE_ROOM) {
          reflowState = REFLOW_STATE_TOO_HOT;
        } else {
          // If switch is pressed to start reflow process
          if (switchStatus == SWITCH_1) {
            // Send header for CSV file
            Serial.println("Time Setpoint Input Output");
            // Intialize seconds timer for serial debug information
            timerSeconds = 0;
            // Initialize PID control window starting time
            windowStartTime = millis();
            // Ramp up to minimum soaking temperature
            setpoint = TEMPERATURE_SOAK_MIN;
            // Tell the PID to range between 0 and the full window size
            reflowOvenPID.SetOutputLimits(0, windowSize);
            reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
            // Turn the PID on
            reflowOvenPID.SetMode(AUTOMATIC);
            // Proceed to preheat stage
            reflowState = REFLOW_STATE_PREHEAT;
          }
        }
        break;

      case REFLOW_STATE_PREHEAT:  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< xxx
        reflowStatus = REFLOW_STATUS_ON;
        // If minimum soak temperature is achieve
        if (input >= TEMPERATURE_SOAK_MIN) {
          // Chop soaking period into smaller sub-period
          timerSoak = millis() + SOAK_MICRO_PERIOD;
          // Set less agressive PID parameters for soaking ramp
          reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
          // Ramp up to first section of soaking temperature
          setpoint = TEMPERATURE_SOAK_MIN + SOAK_TEMPERATURE_STEP;
          // Proceed to soaking state
          reflowState = REFLOW_STATE_SOAK;
        }
        break;

      case REFLOW_STATE_SOAK:  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<  xxx
        // If micro soak temperature is achieved
        if (millis() > timerSoak) {
          timerSoak = millis() + SOAK_MICRO_PERIOD;
          // Increment micro setpoint
          setpoint += SOAK_TEMPERATURE_STEP;
          if (setpoint > TEMPERATURE_SOAK_MAX) {
            // Set agressive PID parameters for reflow ramp
            reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
            // Ramp up to first section of soaking temperature
            setpoint = TEMPERATURE_REFLOW_MAX;
            // Proceed to reflowing state
            reflowState = REFLOW_STATE_REFLOW;
          }
        }
        break;

      case REFLOW_STATE_REFLOW:  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< xxx
        // We need to avoid hovering at peak temperature for too long
        // Crude method that works like a charm and safe for the components
        if (input >= (TEMPERATURE_REFLOW_MAX - 5)) {
          // Set PID parameters for cooling ramp
          reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
          // Ramp down to minimum cooling temperature
          setpoint = TEMPERATURE_COOL_MIN;
          // Proceed to cooling state
          reflowState = REFLOW_STATE_COOL;
        }
        break;

      case REFLOW_STATE_COOL:  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< xxx
        // If minimum cool temperature is achieve
        if (input <= TEMPERATURE_COOL_MIN) {
          // Retrieve current time for buzzer usage
          buzzerPeriod = millis() + 1000;
          // Turn on buzzer and green LED to indicate completion

          digitalWrite(buzzerPin, HIGH);
          // Turn off reflow process
          reflowStatus = REFLOW_STATUS_OFF;
          // Proceed to reflow Completion state
          reflowState = REFLOW_STATE_COMPLETE;
        }
        break;

      case REFLOW_STATE_COMPLETE:  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< xxx
        if (millis() > buzzerPeriod) {
          // Turn off buzzer and green LED
          digitalWrite(buzzerPin, LOW);

          // Reflow process ended
          reflowState = REFLOW_STATE_IDLE;
        }
        break;

      case REFLOW_STATE_TOO_HOT:  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< xxx
        // If oven temperature drops below room temperature
        if (input < TEMPERATURE_ROOM) {
          // Ready to reflow
          reflowState = REFLOW_STATE_IDLE;
        }
        break;

      case REFLOW_STATE_ERROR:  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< xxx
                                // If thermocouple problem is still present

        if ((input == FAULT_OPEN) || (input == FAULT_SHORT_GND) || (input == FAULT_SHORT_VCC))

        {
          // Wait until thermocouple wire is connected
          reflowState = REFLOW_STATE_ERROR;
        } else {
          // Clear to perform reflow process
          reflowState = REFLOW_STATE_IDLE;
        }
        break;
    }

    // If switch 1 is pressed // **************************************************
    if (switchStatus == SWITCH_1) {
      // If currently reflow process is on going
      if (reflowStatus == REFLOW_STATUS_ON) {
        // Button press is for cancelling
        // Turn off reflow process
        reflowStatus = REFLOW_STATUS_OFF;
        // Reinitialize state machine
        reflowState = REFLOW_STATE_IDLE;
      }
    }

    // Simple switch debounce state machine (for switch #1 (both analog & digital
    // switch supported))
    switch (debounceState) {
      case DEBOUNCE_STATE_IDLE:  // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< xxx
        // No valid switch press
        switchStatus = SWITCH_NONE;
        // If switch #1 is pressed

        if (analogRead(upswitch) == 0)

        {
          // Intialize debounce counter
          lastDebounceTime = millis();
          // Proceed to check validity of button press
          debounceState = DEBOUNCE_STATE_CHECK;
        }
        break;

      case DEBOUNCE_STATE_CHECK:  // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< xxx

        if (analogRead(upswitch) == 0)

        {
          // If minimum debounce period is completed
          if ((millis() - lastDebounceTime) > DEBOUNCE_PERIOD_MIN) {
            // Proceed to wait for button release
            debounceState = DEBOUNCE_STATE_RELEASE;
          }
        }
        // False trigger
        else {
          // Reinitialize button debounce state machine
          debounceState = DEBOUNCE_STATE_IDLE;
        }
        break;

      case DEBOUNCE_STATE_RELEASE:  // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< xxx

        if (analogRead(upswitch) > 0)

        {
          // Valid switch 1 press
          switchStatus = SWITCH_1;
          // Reinitialize button debounce state machine
          debounceState = DEBOUNCE_STATE_IDLE;
        }
        break;
    }
    // ********************************************************************************************************************************************************************************** END
  }

  // PID computation and SSR control // ***********************************************
  if (reflowStatus == REFLOW_STATUS_ON) {
    now = millis();

    reflowOvenPID.Compute();

    if ((now - windowStartTime) > windowSize) {
      // Time to shift the Relay Window
      windowStartTime += windowSize;
    }
    if (output > (now - windowStartTime)) digitalWrite(ssrPin, HIGH);
    else digitalWrite(ssrPin, LOW);
  }
  // Reflow oven process is off, ensure oven is off
  else {
    digitalWrite(ssrPin, LOW);
  }
}
