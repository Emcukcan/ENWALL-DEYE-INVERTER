












//LIBRARIES
#include <Preferences.h>

Preferences preferences;
#include <Arduino.h>



#include<can_bms.h>
#include<comm_can_enhub.h>










#include "soc/rtc_wdt.h"
#include <esp_task_wdt.h>
#include "bms_temp.h"

#include "slaver.h"
#include <adcread.h>
#include "BluetoothSerial.h"
#include "nvs_flash.h"



bool FORCE = false;
bool disconnection = false;
int CANBUS_PULSE = 0;
int MODBUS_PULSE = 0;

int AlarmCode = 0;






//PINS
#define GEN_RE  0


uint16_t MODBUSARRAY[60];

//OBJECTS
uint8_t *BMSAR;


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//TASK HANDLES
TaskHandle_t BMS_CANBUSTASK;
TaskHandle_t COMM_CANBUSTASK;
TaskHandle_t MODBUSTASK;
TaskHandle_t BT;



//VARIABLES
double TerminalVoltageArray[16];
double TerminalCurrentArray[16];
double TerminalTempArray[16];
double TerminalSOCArray[16];
double RemainingCapacity[16];
int ChargeStatusArray[16];
int DischargeStatusArray[16];
int AlarmStatusArray[16];
double MaxCellArray[16];
double MinCellArray[16];
float String1Current = 0;
float String1Voltage = 0;
float String1SOC = 0;
float String1CurrentCal = 0;
float String1VoltageCal = 0;
float String1SOCCal = 0;
float String1Temp = 0;
float String1TempCal = 0;

float BMSCurrentArray[40];
float MaxBMSCurrent = -500;
float MinBMSCurrent = 500;




uint8_t datart[8];



String FirmwareVer = "1.0.0";
int SubID = 1;
int ModuleSize = 1;
String SerialNumber = "ENHUB"; // ENC-DATE-NUMBER
String Connection = "Series";


uint8_t alarmsr[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 13 alarms
void setup() {
  RXS = 16;
  TXS = 17;




  Serial.begin(115200);


  Serial.println("nvs is init");
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);



  preferences.begin("my-app", false);

  SerialNumber = preferences.getString("SN", "00001");
  SubID = preferences.getInt("SubID", 1);
  ModuleSize = 1;
  Connection = preferences.getString("Connection", "Series");
  preferences.end();

  Serial.println("");
  Serial.println("Connection type:" + Connection);
  Serial.println("");



  //CREATING TASK FOR CANBUS_____________________________________________________________________________________________________
  xTaskCreatePinnedToCore(
    BMS_CANBUSTASK_CODE,   /* Task function. */
    "BMS_CANBUSTASK",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    8,           /* priority of the task */
    &BMS_CANBUSTASK,      /* Task handle to keep track of created task */
    1);          /* pin task to core 0 */
  delay(50);


  //CREATING TASK FOR CANBUS_____________________________________________________________________________________________________
  xTaskCreatePinnedToCore(
    COMM_CANBUSTASK_CODE,   /* Task function. */
    "COMM_CANBUSTASK",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    8,           /* priority of the task */
    &COMM_CANBUSTASK,      /* Task handle to keep track of created task */
    1);          /* pin task to core 0 */
  delay(50);



  //CREATING TASK FOR MODBUS
  xTaskCreatePinnedToCore(
    MODBUSTASK_CODE,   /* Task function. */
    "MODBUSTASK",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    5,           /* priority of the task */
    &MODBUSTASK,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */
  delay(50);


  //CREATING TASK FOR BLUETOOTH_____________________________________________________________________________________________________
  xTaskCreatePinnedToCore(
    BT_CODE,   /* Task function. */
    "BT",     /* name of task. */
    9000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    2,           /* priority of the task */
    &BT,      /* Task handle to keep track of created task */
    1);          /* pin task to core 0 */
  delay(50);

}






void BMS_CANBUSTASK_CODE( void * pvParameters ) {


  int BMSCurrentIteration = 0;

  Serial.println("BMS CANBUS TASK STARTED");
  delay(10);
  can_start(250);
  delay(50);

  for (;;) {

    CANBUS_PULSE++;


    for (int i = 0; i < 1; i++) {

      BMS_recieve(0x90, i + 1);
      BMS_recieve(0x92, i + 1);
      BMS_recieve(0x91, i + 1);
      BMS_recieve(0x93, i + 1);
      BMS_recieve(0x95, i + 1);
      BMS_recieve(0x98, i + 1);



      if ( abs(BMS_can.current_can - 30000) * 0.1 < 1000) {
        TerminalVoltageArray[i] = BMS_can.sum_voltage_can * 0.1;
        TerminalCurrentArray[i] = (BMS_can.current_can - 30000) * 0.1;
        TerminalTempArray[i] = BMS_can.max_cell_temp_can - 40;
        TerminalSOCArray[i] = BMS_can.SOC_can * 0.1;
        ChargeStatusArray[i] = BMS_can.charge_can;
        DischargeStatusArray[i] = BMS_can.discharge_can;
        AlarmStatusArray[i] = BMS_can.AlarmStatus_can;
        RemainingCapacity[i] = BMS_can.rem_cap_can * 0.001 * 48 * 0.001;
        MaxCellArray[i] = BMS_can.max_cell_volt_can * 0.001;
        MinCellArray[i] = BMS_can.min_cell_volt_can * 0.001;


        BMSCurrentArray[BMSCurrentIteration] = TerminalCurrentArray[0];
        if (BMSCurrentIteration < 40) {
          BMSCurrentIteration++;
        }
        else {
          BMSCurrentIteration = 0;
        }


        //        BMSCurrentArray[40];
        //        MaxBMSCurrent = 0;
        //        MinBMSCurrent = 500;



        MaxBMSCurrent = -500;
        for (int i = 0; i < 40; i++) {
          if (BMSCurrentArray[i] > MaxBMSCurrent) {
            MaxBMSCurrent = BMSCurrentArray[i];
          }
        }

        MinBMSCurrent = 500;
        for (int i = 0; i < 40; i++) {
          if (BMSCurrentArray[i] < MinBMSCurrent) {
            MinBMSCurrent = BMSCurrentArray[i];
          }
        }
      }
      else {
        AlarmStatusArray[i] = 63;
        AlarmCode = 63;

        Serial.println("---- ID:" + String(i + 1) + "failed ------");

      }
      delay(50);
    }

    //    Serial.println("------Rack Data------");
    //    Serial.println("StringVoltage:" + String(String1Voltage));
    //    Serial.println("StringCurrent:" + String(String1Current));
    //    Serial.println("StringSOC:" + String(String1SOC));
    //    Serial.println("------------");

  }
}

void COMM_CANBUSTASK_CODE( void * pvParameters ) {
  Serial.println("COMM CANBUS TASK STARTED");

  spi_class_begin();

  //setup canbus
  CAN.setSPI(hspi);
  for (int i = 0; i < 5; i++) {

    if (CAN_OK == CAN.begin(CAN_500KBPS, MCP_8MHz)) {
      Serial.println("CAN BUS Shield init ok!");
      break;
    }
    else {
      Serial.println("CAN BUS Shield init fail");
      Serial.println(" Init CAN BUS Shield again");

      delay(100);
    }
  }


  for (;;) {



    if ( Connection == "Series") {

      String1VoltageCal = 0;
      for (int i = 0; i < ModuleSize; i++) {
        String1VoltageCal = TerminalVoltageArray[i] + String1VoltageCal;
      }
      String1Voltage = String1VoltageCal;

      String1CurrentCal = 0;
      for (int i = 0; i < ModuleSize; i++) {
        String1CurrentCal = TerminalCurrentArray[i] + String1CurrentCal;
      }
      String1CurrentCal = String1CurrentCal / ModuleSize;
      String1Current = String1CurrentCal;










      String1SOCCal = 0;
      for (int i = 0; i < ModuleSize; i++) {
        String1SOCCal = TerminalSOCArray[i] + String1SOCCal;
      }
      String1SOCCal = String1SOCCal / ModuleSize;
      String1SOC = String1SOCCal;


      String1TempCal = 0;
      for (int i = 0; i < ModuleSize; i++) {
        String1TempCal = TerminalTempArray[i] + String1TempCal;
      }
      String1Temp = String1TempCal / ModuleSize;
    }

    else {

      String1TempCal = 0;
      for (int i = 0; i < ModuleSize; i++) {
        String1TempCal = TerminalTempArray[i] + String1TempCal;
      }
      String1Temp = String1TempCal / ModuleSize;


      String1VoltageCal = 0;
      for (int i = 0; i < ModuleSize; i++) {
        String1VoltageCal = TerminalVoltageArray[i] + String1VoltageCal;
      }
      String1Voltage = String1VoltageCal / ModuleSize;

      String1CurrentCal = 0;
      for (int i = 0; i < ModuleSize; i++) {
        String1CurrentCal = TerminalCurrentArray[i] + String1CurrentCal;
      }
      String1Current = String1CurrentCal;

      String1SOCCal = 0;
      for (int i = 0; i < ModuleSize; i++) {
        String1SOCCal = TerminalSOCArray[i] + String1SOCCal;
      }
      String1SOCCal = String1SOCCal / ModuleSize;
      String1SOC = String1SOCCal;
    }











    if (String1SOC < 80 && String1SOC > 30) {
      can_send_charge(55 , 200, 200);
    }
    else if (String1SOC >= 80 && String1SOC <= 90)
    {
      can_send_charge(55 , 100, 200);
    }
    else if (String1SOC > 90)
    {
      can_send_charge(55 , 20, 200);
    }
    else if (String1SOC < 30 && String1SOC > 20)
    {
      can_send_charge(55 , 200, 100);
    }
    else if (String1SOC <= 20)
    {
      can_send_charge(55 , 200, 20);
    }





    can_send_soc(String1SOC , String1SOC);
    can_send_total_volt(String1Voltage, (MaxBMSCurrent + MinBMSCurrent) * 0.5, String1Temp );
    can_send_request(1, 1 , 0, 0, 0);
    can_send_alarm(alarmsr) ;
    Serial.println("CAN WORKS");
    delay(500);

  }
}


void MODBUSTASK_CODE( void * pvParameters ) {
  Serial.println("MODBUS TASK STARTED");
  baudbaud = 9600;
  Modbus slave(1, Serial2, GEN_RE);


  slave.start();

  for (;;) {


    MODBUS_PULSE++;

    MODBUSARRAY[0] = TerminalVoltageArray[0] * 10;
    MODBUSARRAY[1] = (MaxBMSCurrent + MinBMSCurrent) * 0.5 * 10;
    MODBUSARRAY[2] = TerminalTempArray[0] * 10;
    MODBUSARRAY[3] = String1SOC * 10;
    MODBUSARRAY[4] = AlarmStatusArray[0];
    MODBUSARRAY[5] = ChargeStatusArray[0];
    MODBUSARRAY[6] = DischargeStatusArray[0];
    MODBUSARRAY[7] = RemainingCapacity[0] * 100;
    MODBUSARRAY[8] = MaxCellArray[0] * 100;
    MODBUSARRAY[9] = MinCellArray[0] * 100;



    Serial.println("---- ID:" + String(1) + "Successfull ------");
    Serial.println("ID:" + String(1) + "Voltage:" + String(TerminalVoltageArray[0]));
    Serial.println("ID:" + String(1) + "Current:" + String(TerminalCurrentArray[0]));
    Serial.println("ID:" + String(1) + "SOC:" + String(TerminalSOCArray[0]));
    Serial.println("ID:" + String( 1) + "Charge:" + String(ChargeStatusArray[0]));
    Serial.println("ID:" + String( 1) + "Discharge:" + String(DischargeStatusArray[0]));
    Serial.println("ID:" + String( 1) + "AlarmStatus:" + String(AlarmStatusArray[0]));
    Serial.println("ID:" + String( 1) + "RemainingEnergy:" + String(RemainingCapacity[0]));
    Serial.println("ID:" + String( 1) + "MaxCell:" + String(MaxCellArray[0]));
    Serial.println("ID:" + String( 1) + "MinCell:" + String(MinCellArray[0]));
    Serial.println("----------");
    Serial.println("----------");
    Serial.println("MAX Current:" + String(MaxBMSCurrent));
    Serial.println("MIN Current:" + String(MinBMSCurrent));
    Serial.println("String1SOC:" + String(String1SOC));
    Serial.println("String1Voltage:" + String(String1Voltage));
    Serial.println("----------");


    slave.poll( MODBUSARRAY, 60);

    delay(100);
  }
}


void BT_CODE( void * pvParameters ) {

  String message = "";
  int param_start = 0;
  int param_end = 0;
  String BTProcessor;
  int param_start2 = 0;
  int param_end2 = 0;
  String BTProcessor2;

  String BT_STRING;
  String BT_REMAINING;



  String BT_STATION = "EN-HUB-" + SerialNumber + "-#" + String(SubID);

  BluetoothSerial SerialBT;
  SerialBT.begin(BT_STATION.c_str()); //Bluetooth device name
  Serial.println(F("The device started, now you can pair it with bluetooth!"));

  for (;;) {

    if (SerialBT.available()) {
      char incomingChar = SerialBT.read();
      if (incomingChar != '\n') {
        message += String(incomingChar);
      }
      else {
        message = "";
      }


      if (message != "") {
        Serial.println(message);

      }


      //SET ID NUMBER ////////////////////////////////
      param_start2 = message.indexOf("SETID");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 5, param_end2);
        SubID = BTProcessor2.toInt();
        SerialBT.println("ID:" + String(SubID));
        message = "";

        preferences.begin("my-app", false);
        preferences.putInt("SubID", SubID);
        preferences.end();
        SerialBT.println("System will restart");
        delay(1000);

        ESP.restart();
      }

      //GET ID NUMBER ////////////////////////////////
      param_start2 = message.indexOf("GETID");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        SerialBT.println("ID:" + String(SubID));
        message = "";
      }

      //GET ModuleSize NUMBER ////////////////////////////////
      param_start2 = message.indexOf("GETMS");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        SerialBT.println("ModuleSize:" + String(ModuleSize));
        message = "";
      }

      //SET ModuleSize NUMBER ////////////////////////////////
      param_start2 = message.indexOf("SETMS");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 5, param_end2);
        ModuleSize = BTProcessor2.toInt();
        SerialBT.println("ModuleSize:" + String(ModuleSize));
        message = "";

        preferences.begin("my-app", false);
        preferences.putInt("MS", ModuleSize);
        preferences.end();
        SerialBT.println("System will restart");
        delay(1000);

        ESP.restart();
      }


      //GET fIRMWARE NUMBER ////////////////////////////////
      param_start2 = message.indexOf("GETFW");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        SerialBT.println("FW:" + FirmwareVer);
        message = "";
      }

      //SET SERIAL NUMBER ////////////////////////////////
      param_start2 = message.indexOf("SETSN");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 5, param_end2);
        SerialNumber = BTProcessor2;
        SerialBT.println("SerialNumber:" + String(SerialNumber));
        message = "";

        preferences.begin("my-app", false);
        preferences.putString("SN", SerialNumber);
        preferences.end();
      }


      //get SERIAL ////////////////////////////////
      param_start2 = message.indexOf("GETSN");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {

        SerialBT.println("SerialNumber:" + SerialNumber);
        message = "";
      }




      //get PULSES ////////////////////////////////
      param_start2 = message.indexOf("GETPS");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {

        SerialBT.println(  "CNBS:" + String(CANBUS_PULSE) + "/MDBS:" + String(MODBUS_PULSE));
        message = "";
      }


    }
  }
}





void loop() {



}
