#include <SoftwareSerial.h>
#include <ModbusMaster.h> //Library for using ModbusMaster
#include <SerialMenu.hpp>

#define OUTPUT_1 10 //1. výstup D10
#define OUTPUT_2 11 //2. výstup D11
#define OUTPUT_3 12 //3. výstup D12
#define loopDelay 1000

ModbusMaster node; //object node for class ModbusMaster
SoftwareSerial Serial1(2, 3); // RX, TX
const SerialMenu& menu = SerialMenu::get();

int battery_current;
uint8_t battery_soc;
float PvVoltage1;
float PvCurrent1;
float PvPower1;
bool manual_control = false;
int status = 0; // Stav - nabíjení 0 až 3

uint8_t firmware_major;
uint8_t firmware_minor;

uint8_t getfirmware_Major() {
  uint8_t reading_status = node.readHoldingRegisters(0x0082, 1); // FirmwareVersion_DSP_Major
   if (reading_status == node.ku8MBSuccess) {return node.getResponseBuffer(0);}
   else {
    Serial.print(F("Error: "));
    Serial.println(reading_status);
    return 0;
   }
}

uint8_t getfirmware_Minor() {
   uint8_t reading_status = node.readHoldingRegisters(0x007D, 1); // FirmwareVersion_DSP_Minor
   if (reading_status == node.ku8MBSuccess) {return node.getResponseBuffer(0);}
   else {
    Serial.print(F("Error: "));
    Serial.println(reading_status);
    return 0;
   }
}

void getBatterySOC(uint8_t *battery_soc) { //Get battery SOC
   uint8_t reading_status = node.readInputRegisters(0x001C, 1); //0x001C
   if (reading_status == node.ku8MBSuccess)
   {
    *battery_soc = node.getResponseBuffer(0);
    }
   else {
    Serial.print(F("Error: "));
    Serial.println(reading_status);
   }
}

void getBatteryCurrent(int *battery_current) {
    uint8_t reading_status = node.readInputRegisters(0x0016, 1);
    if (reading_status == node.ku8MBSuccess) {
      *battery_current = node.getResponseBuffer(0);
      }
    else {
      Serial.print(F("Error: "));
      Serial.println(reading_status);
    }
}

void getPvCurrent1(float *current) {
  int16_t a = node.readInputRegisters(0x0005, 1);
  if (a == node.ku8MBSuccess)
  {     
    *current = (node.getResponseBuffer(0) / 10.0);
  } else {
    Serial.print(F("PV1 Error: "));
    Serial.println(a);
  }
}

void getPvVoltage1(float *voltage) {
  int16_t a = node.readInputRegisters(0x0003, 1);
  if (a == node.ku8MBSuccess)
  {     
    *voltage = (node.getResponseBuffer(0) / 10.0);
  } else {
    Serial.print(F("PVCurrent1 Error: "));
    Serial.println(a);
  }
}

//Menu
void info_firmware() {
  Serial.print(F("Firmware: "));
  Serial.print(firmware_major);
  Serial.print(F("."));
  Serial.println(firmware_minor);
}

void show_capacity() {
  Serial.print(F("Battery capacity: "));
  Serial.print(battery_soc);
  Serial.println(F(" %"));
}
void show_status() {
  Serial.print(F("Status: "));
  Serial.println(status);
}
void show_battery_current() {
  Serial.print(F("Battery current: "));
  Serial.println(battery_current);
}
void show_solar_power() {
  Serial.print(F("Solar power: "));
  Serial.println(PvPower1);
}
void turn_on_heater() {
  digitalWrite(OUTPUT_1, HIGH);
  digitalWrite(OUTPUT_2, HIGH);
  digitalWrite(OUTPUT_3, HIGH);
  manual_control = true;
  Serial.println(F("Manual mode ON"));
}
void turn_off_heater() {
  digitalWrite(OUTPUT_1, LOW);
  digitalWrite(OUTPUT_2, LOW);
  digitalWrite(OUTPUT_3, LOW);
  manual_control = false;
  Serial.println(F("Manual mode OFF"));
}

// Declare the menu and its callback functions
const SerialMenuEntry mainMenu[] = {
  {
    "[i]nfo firmware",
    false,
    'i',
    [](){
          info_firmware();
       }
  },
  {
    "[c]apacity",
    false,
    'c',
    [](){
          show_capacity();
       }
  },
  {
    "[s]tatus",
    false,
    's',
    [](){
          show_status();
       }
  },
  {
   "[b]attery current",
   false,
   'b',
    []() {
      show_battery_current();
    }
  },
  {
   "solar [p]ower",
   false,
   'p',
   [](){
    show_solar_power();
   }
  },
  {
    "heate[r] manually ON",
    false,
    'r',
    [](){   
           turn_on_heater();
      }
  },
    {
    "heater manuall[y] OFF",
    false,
    'y',
    [](){   
           turn_off_heater();
      }
  },
  {
    "[m]enu",
    false,
    'm',
    [](){menu.show();}
  }
};

void control() {
   if(!manual_control && (battery_current >= 0)) {
            if (battery_soc >= 95 && (PvPower1 > 2000)) {
                digitalWrite(OUTPUT_1, HIGH);
                digitalWrite(OUTPUT_2, HIGH);
                digitalWrite(OUTPUT_3, HIGH);
                status = 3;
            } else if (battery_soc >= 85 && (PvPower1 > 1500)) {
                digitalWrite(OUTPUT_1, HIGH);
                digitalWrite(OUTPUT_2, HIGH);
                digitalWrite(OUTPUT_3, LOW);
                status = 2;
            } else if (battery_soc >= 75 && (PvPower1 > 700)) {
                digitalWrite(OUTPUT_1, HIGH);
                digitalWrite(OUTPUT_2, LOW);
                digitalWrite(OUTPUT_3, LOW);
                status = 1;
            }
            else {
                digitalWrite(OUTPUT_1, LOW);
                digitalWrite(OUTPUT_2, LOW);
                digitalWrite(OUTPUT_3, LOW);
                status = 0;
            }
        } else if (manual_control) {
                digitalWrite(OUTPUT_1, HIGH);
                digitalWrite(OUTPUT_2, HIGH);
                digitalWrite(OUTPUT_3, HIGH);
                status = 3;
        } else {
            digitalWrite(OUTPUT_1, LOW);
            digitalWrite(OUTPUT_2, LOW);
            digitalWrite(OUTPUT_3, LOW);
            status = 0;
        }
}


void setup() {
   Serial.begin(9600);
     while (!Serial) {
     ; // wait for serial port to connect. Needed for Native USB only
   }

   Serial1.begin(19200);             //Baud Rate as 19200
   node.begin(1, Serial1);            //Slave ID as 1

   pinMode(OUTPUT_1, OUTPUT);
   pinMode(OUTPUT_2, OUTPUT);
   pinMode(OUTPUT_3, OUTPUT);

   firmware_major = getfirmware_Major();
   firmware_minor = getfirmware_Minor();
     
   menu.load(mainMenu, GET_MENU_SIZE(mainMenu));
   menu.show();
}



void loop() {
    menu.run(loopDelay);

    getBatterySOC(&battery_soc);
    getBatteryCurrent(&battery_current);
    getPvVoltage1(&PvVoltage1);
    getPvCurrent1(&PvCurrent1);
    PvPower1 = PvVoltage1 * PvCurrent1;

    
    /*Serial.print(F("Proud baterie: "));
    Serial.println(battery_current);
    Serial.print(F("voltage: "));
    Serial.print(PvVoltage1);
    Serial.print(F("\tcurrent: "));
    Serial.print(PvCurrent1);
    Serial.print(F("\tPower: "));
    Serial.println(PvPower1);*/

    control();
   
    delay(loopDelay);
    delay(loopDelay);
    delay(loopDelay);
    delay(loopDelay);
    delay(loopDelay);
}
