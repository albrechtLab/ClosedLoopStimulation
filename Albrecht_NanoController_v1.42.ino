#define version_name "Albrecht_NanoController_v1.42" // 2021.07.05 DRA

/** ------------------------------
Control valves at specific frames during a streaming acquisition
Input trigger from camera comes from pin 2
Output trigger mirrors input for controlling the illumination on pin 7
Valves are triggered to open and close at specific frames according to the serial command:
LED1 and LED8 are on pins 5, 6
Brightfield LED on Pin 8

//NEW SYNTAX
   ***new format: A/a = valve1 on/off, L/l = LED1 on/off (full power)
   A50,a100,A150,a200,L250,l300
**/

/***************************************************
No touchscreen or display on this version
See "AlbrechtController_v1.42" or higher for touchscreen version
****************************************************/

#include <Arduino.h>

#define TRIGGERIN  2      // for interrupt 0 (CAM IN)
#define TRIGGEROUT_FL  7  // Digital out for fluorescence excitation (FL OUT - Lumencor)
#define TRIGGEROUT_BF  8  // Digital out for brightfield (BF OUT)
#define INT_LED 13        // internal LED pin for display (CAM IN and stimulus)

#define LED1 5    //  PWM for adjustable LED intensity (or stimulus voltage)
#define LED2 6    //  PWM for adjustable LED intensity

#define VALVE1 14    // TTL for valves, high = open  
#define VALVE2 15    // A0 = 14, A1 = 15, A2 = 16, etc 
#define VALVE3 16

// Variable sizes are limited to serial commands of 128 char, and up to 32 switches
// Expand as needed if patterns appear truncated!
#define INPUT_SIZE 128 // max input serial string size
#define MAX_SWITCHES 32 // max number of switch frames

// Following applies only to BuckPuck LEDs. Other sources may use TTL high = on, low = off
//const int BF_LEDON = 210; // Intensity control for BF; <80 = max; >220 = off; ~linear in between, 150 = 50% current
//const int BF_OFF = 255;

byte FL_ON = LOW;         // should be HIGH for Mightex; LOW for Lumencor
byte BF_ON = HIGH;        // should be HIGH for Zeiss, LOW for buckpuck
byte INPUT_ON = LOW;      // depends on and should match Micromanager settings (low for negative polarity)

byte FL_OFF = !FL_ON;
byte BF_OFF = !BF_ON;
byte INPUT_OFF = !INPUT_ON;

const int brightfieldInterval = 2;    // how often to pulse brightfield illumination, e.g. 2 = every other frame is BF; 3 = every 3rd frame...
byte testHz = 10;                     // test signal frequency
long int BFmicros = 0;                // brightfield illumination pulse duration (us). If 0, then follows input pulse
long int FLmicros = 0;                // fluorescent illumination pulse duration (us). If 0, then follows input pulse

// Initialize variables
int valve1state = LOW;         // default valve 1 OFF
int valve2state = HIGH;        // default valve 2 ON
int valve3state = LOW;         // default valve 3 OFF
int LED1out = 0;               // default value for LED1
int LEDintensity = 255;        // default LED intensity value
// add LED2out?

volatile long pulseCount = 0;
int valveSwitchCount = 0;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
boolean switchToNextValve = false;   // is there a state change to be made
boolean setLEDintensity = false;     // is there an intensity level change
boolean debug = true;

boolean FL = true;            // default power-on settings: FL only
boolean BF = false;
boolean testMode = false;

// Initialize variables
int Frames[MAX_SWITCHES];       // vector with frame numbers
int LED1levels[MAX_SWITCHES];   // LED1 state at each frame number
int V1states[MAX_SWITCHES];     // Valve 1 state at each frame number
int V2states[MAX_SWITCHES];     // ...   2
int V3states[MAX_SWITCHES];     // ...   3

long int ms = millis();   // elapsed time in ms
long int testms = ms;
float fps;                // frame rate 

char input[INPUT_SIZE + 1];

void setup()
{
    for (int i=0; i<MAX_SWITCHES; i++) {    // initialize all vectors as -1
        Frames[i] = -1;
        LED1levels[i] = -1;
    }
    
    // Set Pin settings
    pinMode(TRIGGERIN, INPUT_PULLUP);
    pinMode(TRIGGEROUT_FL, OUTPUT);
    pinMode(TRIGGEROUT_BF, OUTPUT);
    pinMode(VALVE1, OUTPUT);
    pinMode(VALVE2, OUTPUT);
    pinMode(VALVE3, OUTPUT);
    pinMode(LED1, OUTPUT);
    // Add LED2?

    // Initialize outputs
    digitalWrite(TRIGGEROUT_FL, FL_OFF);
    //if (!BF) analogWrite(LED2, BF_OFF);
    digitalWrite(TRIGGEROUT_BF, BF_OFF);
    digitalWrite(VALVE1, valve1state);
    digitalWrite(VALVE2, valve2state);
    digitalWrite(VALVE3, valve3state);
    digitalWrite(INT_LED, LOW);

    digitalWrite(LED1, LOW);
    // Add LED2?

    // Define Interrupts. 0 (pin 2 detects change at CAM IN)
    attachInterrupt(0, triggerChange, CHANGE);
    
    // start serial port
    Serial.begin(115200);
    
    Serial.println(F("------------------------------"));
    Serial.println(F("Valve Commands:"));
    Serial.println(F("  A###, a###    valve 1 on, off at frame ###"));
    Serial.println(F("  B###, b###    valve 2 on, off ..."));
    Serial.println(F("  C###, c###    valve 3 ..."));
    Serial.println(F("Stimulus (LED) Commands:"));
    Serial.println(F("  L###, l###    LED1 on, off at frame ###"));
    Serial.println(F("  i###          sets intensity of next LED [0=off, 255=max]"));
    Serial.println();
    Serial.println(F("example:  A50,a100,C100,c125,i100,L200,i200,L225,l250"));
    Serial.println();
    Serial.println(F("------------------------------"));
    Serial.println(F("Manual Commands:"));
    Serial.println(F("  v1on, v1off   immediate valve 1 on, off"));
    Serial.println(F("  v2on, v2off   immediate valve 2 on, off"));
    Serial.println(F("  v3on, v3off   immediate valve 3 ..."));
    Serial.println();
    Serial.println(F("  =N, =P      set Fluor. polarity to Negative or Positive"));
    Serial.println(F("  =n, =p      set CAM IN polarity to Negative or Positive"));
    Serial.println(F("              can combine: =Nn  set both to negative [default]"));
    Serial.println();
    Serial.println(F("  _F, _B      set Fluor. or brightfield illumination"));
    Serial.println(F("              can combine: _BF set both to alternate "));
    Serial.println();
    Serial.println(F("  ~f###       set Fluor. pulse to ### microseconds (us)"));
    Serial.println(F("  ~b###       set brightfield pulse to ### microseconds (us)"));
    Serial.println(F("              if set to 0, follows the CAM IN trigger [default]"));
    Serial.println();
    Serial.println(F("  T           start test signal"));
    Serial.println(F("  X           end test signal"));
    Serial.println();
    Serial.println(F("  reset       reset Arduino"));
    Serial.println();
    Serial.println(F("------------------------------"));
    Serial.println(F(version_name)); 
      
}

// Define the software reset function
void(* resetFunc) (void) = 0; //declare reset function @ address 0

void loop()
{
    // digitalWrite(VALVE3, LOW);

    // Check for serial command to define and start the pulse counting and valve control.
    int i = 0;
    //int j = 0;

    while (Serial.available() > 0)
    {
        if (Serial.available()) {
            // Get next command from Serial (add 1 for termination character)
            byte size = Serial.readBytes(input, INPUT_SIZE);
            // Add the final 0 to end the string
            input[size] = 0;
            inputString = String(input);
        }
                
        Serial.print(F("Input: [")); Serial.print(inputString); Serial.print("] ");
        
        boolean timingEntry = true; // assume timing pattern until evidence otherwise

///////////////////// PARSE MANUAL COMMANDS HERE //////////////////////
        
        // Check for manual valve command: v1on/v1off/v2on/v2off, etc
        char* ch = strchr(input, 'v');
        if (ch != 0)
        {
            ++ch;
            int valvenum = atoi(ch); // get valve number 
            ch += 2;                 // move char pointer

            if (valvenum == 1)
            {
                valve1state = (strchr(input, 'on') != 0);
                digitalWrite(VALVE1, valve1state);
                Serial.print(F("Valve 1 switch to: ")); Serial.println(valve1state);
            }
            if (valvenum == 2)
            {
                valve2state = (strchr(input, 'on') != 0);
                digitalWrite(VALVE2, valve2state);
                Serial.print(F("Valve 2 switch to: ")); Serial.println(valve2state);
            }
            if (valvenum == 3)
            {
                valve3state = (strchr(input, 'on') != 0);
                digitalWrite(VALVE3, valve3state);
                Serial.print(F("Valve 3 switch to: ")); Serial.println(valve3state);
            }
            timingEntry = false;
        }

        // Check for selection of brightfield vs fluorescence illumination
        ch = strchr(input, '_');  // _F = fluor, _B = brightfield, _BF or _FB = both
        if (ch != 0)
        {
            ++ch;

            BF = (strchr(input, 'B') != 0);
            FL = (strchr(input, 'F') != 0);
                        
            Serial.print(F("Changing FL, BF setting to: ")); Serial.print(FL); Serial.print(", "); Serial.println(BF);
            timingEntry = false;
        }

        // Check for selection of OUTPUT and CAM_IN polarity (OUTPUT = capital, CAM_IN = lower)
        ch = strchr(input, '=');  // =Nn = negative FL, negative CAM IN,
                                  // =Np = negative FL, positive CAM IN, etc
        if (ch != 0)
        {
            ++ch;
            INPUT_ON = LOW;
            INPUT_OFF = LOW;
            if (strchr(input, 'P') != 0) {
                FL_ON = HIGH;
            }
            if (strchr(input, 'p') != 0) {
                INPUT_ON = HIGH; 
            }
            FL_OFF = !FL_ON;
            INPUT_OFF = !INPUT_ON;
            Serial.print(F("FL_out, CAM_in Polarities: ")); Serial.print(FL_ON); Serial.print(", "); Serial.println(INPUT_ON);
            timingEntry = false;
        }

        // Check for manual definition of pulse timing use ~f## for FL and ~b## for BF
        ch = strchr(input, '~');  // e.g. ~f10 = 10 us, ~f10000 = 10 ms
        if (ch != 0)
        {
            ++ch;
            
            if (strchr(input, 'f') != 0) {
                ++ch;
                FLmicros = atoi(ch);
                Serial.print(F("Changing FL pulse duration to: "));
                Serial.print(FLmicros);
                Serial.println(" us");
                String label = "FL| |";
            } else if (strchr(input, 'b') != 0) {
                ++ch;
                BFmicros = atoi(ch);
                Serial.print(F("Changing BF pulse duration to: "));
                Serial.print(BFmicros);
                Serial.println(" us");
            }         
            timingEntry = false;
        }
                
        // Check to start test mode or stop test mode 
        // (internally-generated camera signals)
        ch = strchr(input, 'T');  // test mode start
        if (ch != 0)
        {
            ++ch;
            testMode = true;
            setTestMode(testMode);
            timingEntry = false;
            Serial.flush();
        }
        ch = strchr(input, 'X');  // test mode end
        if (ch != 0)
        {
            ++ch;
            testMode = false;
            setTestMode(testMode);
            timingEntry = false;
            Serial.flush();
        }
        
        ch = strchr(input, 'reset');  // hard reset  
        if (ch != 0) resetFunc();     //call reset

///////////////////////////  PARSE TIMING COMMANDS HERE ///////////////////
                        
        if (timingEntry)
        {
            V1states[0] = 0;
            V2states[0] = -1;    // important: -1 means don't change it!
            V3states[0] = -1;    // ...
            LED1levels[0] = 0;
            Frames[0] = 0;
            setLEDintensity = false;
            i = 1;

            Serial.print(F("Parse: "));
            // Parse timing and LED intensity commands
            char* command = strtok(input, " ,");
            while (command != 0)
            {
                //Serial.print(" "); Serial.println(command);

                // Initialize next command as same as prior step
                //  (so we don't need to do it repeatedly, and
                //  they will be reset if no valid command is found)
                LED1levels[i] = LED1levels[i-1];
                V1states[i] = V1states[i-1];
                V2states[i] = V2states[i-1];
                V3states[i] = V3states[i-1];
                
                char firstChar = *command;
                if (debug) { Serial.print("\n ["); Serial.print(firstChar); Serial.print(']');}
                switch (firstChar) {
                    case 'i':         // to set current intensity
                        setLEDintensity = true;
                        break;
                    case 'A':         // valve 1 open
                        V1states[i] = 1; 
                        break;
                    case 'a':         // valve 1 closed
                        V1states[i] = 0; 
                        break;
                    case 'B':         // valve 2 open
                        V2states[i] = 1; 
                        break;
                    case 'b':         // valve 2 closed
                        V2states[i] = 0; 
                        break;
                    case 'C':         // valve 3 open
                        V3states[i] = 1; 
                        break;
                    case 'c':         // valve 3 closed
                        V3states[i] = 0; 
                        break;
                    case 'L':
                        LED1levels[i] = LEDintensity;
                        break;
                    case 'l':
                        LED1levels[i] = 0;
                        break;
                    default:
                        LED1levels[i] = -1;
                        V1states[i] = -1;
                        V2states[i] = -1;
                        V3states[i] = -1;
                    break;
                }
                *command = 0;
                ++command;
                if (setLEDintensity)
                {
                    LEDintensity = atoi(command);
                    if(debug) { Serial.print(F(" LED intensity set to: ")); Serial.print(LEDintensity); }
                    setLEDintensity = false;
                }
                else 
                {
                    Frames[i] = atoi(command);
                    if(debug) { Serial.print(F(" at frame ")); Serial.print(Frames[i]); }
                    i++;
                }

                // Find the next command in input string
                command = strtok(0, " ,");
            }

            Serial.print(F("\n# Frame switches: ")); Serial.print(i);
            //Serial.print(F(", LED1levels ")); Serial.println(j);
            Serial.println();

            /*
            // Sort output by time in ascending order
            int idx[i]; 
            for (int k = 0; k < i; k++) idx[k] = k; // assume initially in ascending order
            for (int k = 0; k < i; k++) {           // then test and swap as necessary
                for (int k2 = k; k2 < i; k2++) {    // step through each additional value
                    if (Frames[k2] < Frames[k]) { 
                      int temp = Frames[k]; Frames[k]=Frames[k2]; Frames[k2]=temp; 
                          temp = idx[k];    idx[k]=idx[k2];       idx[k2]=temp; 
                          temp = LED1levels[k]; LED1levels[k]=LED1levels[k2]; LED1levels[k2]=temp;
                          temp = V1states[k]; V1states[k]=V1states[k2]; V1states[k2]=temp;
                          temp = V2states[k]; V2states[k]=V2states[k2]; V2states[k2]=temp;
                          temp = V3states[k]; V3states[k]=V3states[k2]; V3states[k2]=temp;
                    }
                }
                Serial.print(idx[k]); Serial.print(", ");
            }
            */

            for (int z = i; z < MAX_SWITCHES; z++) {
              Frames[z] = -1;      // blank out any previous frame switch settings
              LED1levels[z] = -1;  // blank out any previous intensity settings
              V1states[z] = -1;    // blank out any previous valve settings
              V2states[z] = -1;    // ...
              V3states[z] = -1;    // ...
            }

            // display vectors info for debugging
            Serial.println("Fr\t v1\t v2\t v3\t LED1");

            // check for non-controlled valves and out-of-order values
            int v1used, v2used, v3used;          
            for (int k = 0; k < MAX_SWITCHES; k++) {
              if (k>0 && k<i && Frames[k]<Frames[k-1]) {
                  Serial.println(F("\n*** WARNING: Switch times out of order. May not switch properly! ***"));
              }
              v1used += (V1states[k]>= 0);
              v2used += (V2states[k]>= 0);
              v3used += (V3states[k]>= 0);
                            
              if (debug) { 
                Serial.print(Frames[k]); Serial.print('\t'); 
                Serial.print(V1states[k]); Serial.print('\t'); 
                Serial.print(V2states[k]); Serial.print('\t'); 
                Serial.print(V3states[k]); Serial.print('\t'); 
                Serial.print(LED1levels[k]); Serial.println('\t'); 
                //if (V2states < 0) Serial.print("No V2states");
              }
            }
            Serial.print("\t"); Serial.print(v1used);
            Serial.print("\t"); Serial.print(v2used);
            Serial.print("\t"); Serial.println(v3used);
                   
            if (i > 0)                               // if there is timing data, then:
            {
                pulseCount = -2;                     // Initialize pulse count
                valveSwitchCount = 0;                // Initialize valve switch count
                digitalWrite(VALVE1, LOW);           // Initialize valve 1 off
                digitalWrite(VALVE2, HIGH);          // Initialize valve 2 ON
                //digitalWrite(VALVE3, LOW);         // Initialize valve 3 off
                //digitalWrite(LED1, LOW);           // Initialize LED1 off         
            }
            Serial.read();   //
        }
    }

    if (testMode) {    // establish a 1ms pulse every 100ms test input
        int delayms = testms + (1000/testHz) - millis();
        if (delayms > 0) delay(delayms);
        
        digitalWrite(TRIGGERIN, INPUT_ON);
        delayMicroseconds(1000);            // 1ms pulse
        digitalWrite(TRIGGERIN, INPUT_OFF);
        
        //Serial.print('.');
        Serial.println(testms);
    }

    // Update valves if pulseCount equals the next switch frame
    if (switchToNextValve)
    {
        while (Frames[valveSwitchCount] > 0 && pulseCount >= Frames[valveSwitchCount]) 
        {
            LED1out = LED1levels[valveSwitchCount];
            if (LED1out >= 0) analogWrite(LED1, LED1out);

            // Get desired valve states and set. Ignore if value is -1
            valve1state = V1states[valveSwitchCount];
            if (valve1state >= 0) digitalWrite(VALVE1, valve1state);
            valve2state = V2states[valveSwitchCount];
            if (valve2state >= 0) digitalWrite(VALVE2, valve2state);
            valve3state = V3states[valveSwitchCount];
            if (valve3state >= 0) digitalWrite(VALVE3, valve3state);
        
            valveSwitchCount++;
        }
        switchToNextValve = false;
    }
}

void setTestMode( boolean testMode)
{
    if (testMode) {
        pinMode(TRIGGERIN, OUTPUT);
    } else {
        pinMode(TRIGGERIN, INPUT);
    }
}

void triggerChange()
{
    if (digitalRead(TRIGGERIN) == INPUT_ON)  // input signal from camera to pulse the light
    {
        pulseCount++;
        Serial.print(pulseCount); Serial.print(' ');
        if (pulseCount == Frames[valveSwitchCount]) switchToNextValve = true;
        fps = 1000.0 / (millis()-testms);
        testms = millis();

        digitalWrite(INT_LED, HIGH); // indicate pulse on status LED
        
        // next pulse is fluorescent if only FL, OR both FL & BF and it's not BF's turn
        boolean fluor_pulse = (FL && (!BF || (BF && !(pulseCount % brightfieldInterval))));

        if (fluor_pulse) {
            digitalWrite(TRIGGEROUT_FL, FL_ON);
            Serial.print("/f");
            if (FLmicros > 0) {
                delayMicroseconds(FLmicros);          // if specified, define FL pulse
                digitalWrite(TRIGGEROUT_FL, FL_OFF);  // otherwise, follow TRIGGERIN 
                Serial.print("| ");
            }       
        } else {
            digitalWrite(TRIGGEROUT_BF, BF_ON);
            Serial.print("/b");
            if (BFmicros > 0) {
                delayMicroseconds(BFmicros);          // if specified, define BF pulse
                digitalWrite(TRIGGEROUT_BF, BF_OFF);  // otherwise, follow TRIGGERIN
                Serial.print("| ");
            }             
        }
    }
    else
    {
        digitalWrite(TRIGGEROUT_BF, BF_OFF);
        digitalWrite(TRIGGEROUT_FL, FL_OFF);
        digitalWrite(INT_LED, LOW); // indicate pulse on status LED
        Serial.print("\\ ");
    }
}
