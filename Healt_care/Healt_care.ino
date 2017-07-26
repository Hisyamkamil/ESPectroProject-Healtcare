#include <Ticker.h>
#include "PulseSensor.h"
#include "Wire.h"
#include "MPU6050.h"

//Set USE_EXTERNAL_ADC to 1 if you have ESPectro Base (which has external ADC), otherwise set 0 to use INTERNAL ESP8266 ADC
#define USE_EXTERNAL_ADC    0

//Set this to 1 to wait for callback when BPM available, or set to 0 to poll the last read BPM and display using serial monitor or desktop app
#define USE_CALLBACK        1

//Set to 1 if you're using ESPectro board
#define USE_ESPECTRO_BOARD  1

//Set to 1 to start measurement automatically upon program start
#define AUTO_START_MEASUREMENT  0

#define OLED_SSD1306_DISPLAY    1

#define USE_CLOUD               1


#if USE_ESPECTRO_BOARD
#include <ESPectro.h>
ESPectro board;
ESPectro_Button onBoardButton;
#endif

#if USE_EXTERNAL_ADC
#include <ESPectroBase.h>
ESPectroBase base;
#endif

#if OLED_SSD1306_DISPLAY
#include "UIService.h"
UIService uiSvc;
#endif

#if USE_CLOUD
#include <MakestroCloudClient.h>
#include <DCX_AppSetting.h>
#include <DCX_WifiManager.h>
#include <ESP8266WiFi.h>

DCX_WifiManager wifiManager(AppSetting);
MakestroCloudClient makestroCloudClient("Hisyam_Kamil", "CjIzfO689VdBxb0O5krsgLk7zdQWEDlwhz49eA0AaZ7b6rW2ZMAryQiguDIqeyE0", "HealtCare", "Hisyam_Kamil-HealtCare-default");

#endif

#if !USE_CALLBACK
// Set to 'false' by default to send the data via serial and display using desktop app.  Set to 'true' to see Arduino Serial Monitor ASCII Visual Pulse
static boolean serialVisual = false;
#endif

MPU6050 accelgyro;

//Ticker ticker;
PulseSensor sensor;
int lastBPM;

int16_t ax, ay, az;
boolean langkahact = false;
int koordinatX ;
int koordinatY ;
int koordinatZ ;
int Langkah = 0;

void readADC() {
#if USE_EXTERNAL_ADC
    int raw = base.analogRead(3);
#else
    int raw = analogRead(0);
#endif
    Serial.printf("Raw: %d\n", raw);
}

#if USE_CLOUD

  void  onSubscribedPropertyCallback(const String prop, const String value) {
        Serial.print("incoming: ");
        Serial.print(prop);
        Serial.print(" = ");
        Serial.print(value);
        Serial.println();
    }


void onMqttConnect(bool sessionPresent) {
    Serial.println("** Connected to the broker **");
    //board.turnOnNeopixel(HtmlColor(0x00ff00),1);
    makestroCloudClient.subscribeProperty("switch", onSubscribedPropertyCallback);

}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
    Serial.println("** Disconnected from the broker **");
    //board.turnOnNeopixel(HtmlColor(0xff0000),1);
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
    Serial.println("** Subscribe acknowledged **");
    Serial.print("  packetId: ");
    Serial.println(packetId);
    Serial.print("  qos: ");
    Serial.println(qos);
}

#endif

// the setup function runs once when you press reset or power the board
void setup() {
    Serial.begin(115200);

    while(!accelgyro.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)){
        Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
        delay(500);
    }

    //checkSettings();

    #if OLED_SSD1306_DISPLAY
    uiSvc.begin();
    #endif

    #if USE_CLOUD    
        AppSetting.load();
        AppSetting.debugPrintTo(Serial);

        wifiManager.onWifiConnectStarted([]() {
            DEBUG_SERIAL("WIFI CONNECTING STARTED\r\n");
        });

        wifiManager.onWifiConnected([](boolean newConn) {
            DEBUG_SERIAL("WIFI CONNECTED\r\n");
           // board.turnOnNeopixel(HtmlColor(0x0000ff),0);
            //board.turnOffLED();

            makestroCloudClient.onConnect(onMqttConnect);
            makestroCloudClient.onDisconnect(onMqttDisconnect);
            makestroCloudClient.onSubscribe(onMqttSubscribe);
            makestroCloudClient.connect();
            sensor.start();

        });

        wifiManager.onWifiConnecting([](unsigned long elapsed) {
            DEBUG_SERIAL("%d\r\n", elapsed);
            //board.toggleLED();
        });

        wifiManager.onWifiDisconnected([](WiFiDisconnectReason reason) {
            DEBUG_SERIAL("WIFI GIVE UP\r\n");
            //board.turnOnNeopixel(HtmlColor(0xff0000),0);
            //board.turnOffLED();
        });

        //wifiManager.begin();
        wifiManager.begin("dycodex", "11223344");
    #endif

    #if USE_EXTERNAL_ADC
        base.beginADC();
    #endif

//    ticker.attach_ms(200, readADC); //-> just for testig

    #if USE_CALLBACK
        
        sensor.onBPMAvailable([](int BPM) {
            //Serial.printf("BPM: %d\n", BPM);
            //leitura();
            //pedometer();
            //qualifies BPM for a grown-up
            if (BPM > lastBPM+20 || BPM < lastBPM-20|| BPM > 160 || BPM < 54) {
                Serial.printf("[Not Valid] BPM: %d\n", BPM);
            }
            else {
                Serial.printf("[Valid] BPM: %d\n", BPM);
                uiSvc.setBPMValue(BPM);

                #if USE_CLOUD
                    if (makestroCloudClient.connected()) {
                        makestroCloudClient.publishKeyValue("BPM", BPM);
                        Serial.print("Publish BPM");
                    }
                #endif

            }

            lastBPM = BPM;
        });

        sensor.onPulseDetectionCallback([](bool detected) {
            #if USE_ESPECTRO_BOARD
                if (detected) {
                    board.turnOnLED();
                }
                else {
                    board.turnOffLED();
                }
            #endif
        });

    #endif

    //set callback to actually read the raw value from ADC
    sensor.onReadingRawSignal([](int &raw) {

        #if USE_EXTERNAL_ADC
            raw = base.analogRead(3);
            #else
            raw = analogRead(0);
            #endif

    });

    #if AUTO_START_MEASUREMENT
        sensor.start();
    #else
    #if USE_ESPECTRO_BOARD
        onBoardButton.begin();
        onBoardButton.onButtonUp([](){
            if (sensor.isStarted()) {
                Serial.println("Pulse sensor stopped");
                sensor.stop();
            }
            else {
                Serial.println("Pulse sensor started");
                sensor.start();
                
            }
        });

    #endif
    #endif

}

#if !USE_CALLBACK
//  Code to Make the Serial Monitor Visualizer Work
    void arduinoSerialMonitorVisual(char symbol, int data ){
        const int sensorMin = 0;      // sensor minimum, discovered through experiment
        const int sensorMax = 1024;    // sensor maximum, discovered through experiment

        int sensorReading = data;
        // map the sensor range to a range of 12 options:
        int range = map(sensorReading, sensorMin, sensorMax, 0, 11);

        // do something different depending on the
        // range value:
        switch (range) {
            case 0:
                Serial.println("");     /////ASCII Art Madness
                break;
            case 1:
                Serial.println("---");
                break;
            case 2:
                Serial.println("------");
                break;
            case 3:
                Serial.println("---------");
                break;
            case 4:
                Serial.println("------------");
                break;
            case 5:
                Serial.println("--------------|-");
                break;
            case 6:
                Serial.println("--------------|---");
                break;
            case 7:
                Serial.println("--------------|-------");
                break;
            case 8:
                Serial.println("--------------|----------");
                break;
            case 9:
                Serial.println("--------------|----------------");
                break;
            case 10:
                Serial.println("--------------|-------------------");
                break;
            case 11:
                Serial.println("--------------|-----------------------");
                break;

    }
}

void sendDataToSerial(char symbol, int data ){
    Serial.print(symbol);
    Serial.println(data);
}

void serialOutput(){   // Decide How To Output Serial.
    if (serialVisual == true){
        arduinoSerialMonitorVisual('-', sensor.doReadRawSignal());   // goes to function that makes Serial Monitor Visualizer
    } else{
        sendDataToSerial('S', sensor.doReadRawSignal());     // goes to sendDataToSerial function
    }
}

void serialOutputWhenBeatHappens(){
    if (serialVisual == true){            //  Code to Make the Serial Monitor Visualizer Work
        Serial.print("*** Heart-Beat Happened *** ");  //ASCII Art Madness
        Serial.print("BPM: ");
        lastBPM = sensor.getLastBPM();
        Serial.print(lastBPM);
        Serial.print("  ");
    } else{
        sendDataToSerial('B', sensor.getLastBPM());   // send heart rate with a 'B' prefix
        sendDataToSerial('Q', sensor.getLastIBI());   // send time between beats with a 'Q' prefix
    }
}
#endif

// the loop function runs over and over again forever
void loop() {
    
    accRead();
    pedometer();

    #if USE_CLOUD
        wifiManager.run();
    #endif

    #if OLED_SSD1306_DISPLAY
        uiSvc.loop();
    #endif

    #if !AUTO_START_MEASUREMENT
    #if USE_ESPECTRO_BOARD
    //    onBoardButton.loop();
    #endif
    #endif

    #if !USE_CALLBACK
        serialOutput();

    if (sensor.isBeatDetected()) {
        serialOutputWhenBeatHappens();
    }
    
    delay(20);                             //  take a break
    #endif

}

void checkSettings()
{
  Serial.println();
  
  Serial.print(" * Sleep Mode:            ");
  Serial.println(accelgyro.getSleepEnabled() ? "Enabled" : "Disabled");
  
  Serial.print(" * Clock Source:          ");
  switch(accelgyro.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }
  
  Serial.print(" * Accelerometer:         ");
  switch(accelgyro.getRange())
  {
    case MPU6050_RANGE_16G:            Serial.println("+/- 16 g"); break;
    case MPU6050_RANGE_8G:             Serial.println("+/- 8 g"); break;
    case MPU6050_RANGE_4G:             Serial.println("+/- 4 g"); break;
    case MPU6050_RANGE_2G:             Serial.println("+/- 2 g"); break;
  }  

  Serial.print(" * Accelerometer offsets: ");
  Serial.print(accelgyro.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(accelgyro.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(accelgyro.getAccelOffsetZ());
  
  Serial.println();
}
void pedometer() {
 int m = 0;
  //PdXYZ[k] = abs((signed long int) (ax)) + abs((signed long int) (ay)) + abs((signed long int) (az));
  koordinatX = ax;
  koordinatY = ay;
  koordinatZ = az;
  if ((koordinatX>9) && (koordinatY>34) && (koordinatZ>0) ) 
  {
    langkahact = true;
    Serial.println("langkah aktif");
  }
  if ((koordinatX > 9) && (koordinatY > 34) && (koordinatZ > 0) && (langkahact) ) {
    Langkah = Langkah;
    Langkah++;
    langkahact = false;
    Serial.print("langkah nambah");
    Serial.println(Langkah);
  
    #if USE_CLOUD
    makestroCloudClient.publishKeyValue("pedometer",Langkah);
    uiSvc.setPedoValue(Langkah);
    Serial.println("publish to cloud");
    #endif

  }
}
void accRead()
{
  Vector normAccel = accelgyro.readNormalizeAccel();
 // accelgyro.readNormalizeAccel();
  ax = normAccel.XAxis;
  Serial.println(ax);
  ay = normAccel.YAxis;
  Serial.println(ay);
  az = normAccel.ZAxis;
  Serial.println(az);
}