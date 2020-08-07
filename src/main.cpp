/////////////////////
// lib Definitions //
/////////////////////
#include "queue_list.h" 
#include "lidar_buffer.h"  
#include "config.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <YDLidar.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
/////////////////////
// Pin Definitions //
/////////////////////
#define X_STEP_PIN                            12  
#define X_DIR_PIN                             13 
#define X_ENABLE_PIN                          17 

#define M_EN  27 //14 step-dir e0 e4d
#define DEV_EN 0 //not used 
#define M_SCTP 0 //not used 

#define CW 0
#define CCW 1
#define Microstep 16
#define SIZE_OF_SCAN_BUFFER 1024
#define BUF_SIZE 1800
#define Pi 3.14
//////////////////////
// WiFi Definitions //
//////////////////////
const char WiFiSSID[] = "Nik-Lidar";
const char WiFiAPPSK[] = "scanner12123";


//////////////////////
// Global           //
//////////////////////
//WiFiServer server(80);
WebServer server(80);
YDLidar lidar;
QueueList<scanPoint> scans;
buffer<uint8_t> buf(BUF_SIZE);
bool isScanning = false;   
unsigned int pos = 0;
bool dir = CW;

//////////////////////
// functions        //
//////////////////////
void listDir(){
server.sendContent("<table style=\"width: 100%; border-color: #000000; margin-left: auto; margin-right: auto;\" border=\1\"><tbody><tr><th style=\"width: 271px;\"><em>Listing file:</em></th><th>Size</th></tr>");
File root = SD.open("/","r");
File file = root.openNextFile();
while(file){
String fname=file.name();
String fsize=String(file.size());
server.sendContent("<tr><td><a href=/download?download=" +fname + ">" +fname + "</a></td><td>" + fsize+ "</td></tr>");
file = root.openNextFile();
   }
server.sendContent("</tbody></table>");
root.close();
file.close();
}

void HomePage()
{ // Prepare the response. Start with the common header:
  String s = "HTTP/1.1 200 OK\r\n";
  s += "Content-Type: text/html\r\n\r\n";
  s += "<!DOCTYPE HTML>\r\n<html>\r\n";
  s += "<h2><span style=\"color: #99cc00;\">Nik-Lidar <span style=\"color: #00ff00;\">v0.5</span></span></h2>";
  s += "<table style=\"height: 34px;\" width=\"100%\"><tbody><tr><td style=\"width: 132px; text-align: center;\"><a title=\"Start\" href=\"./start\">Start</a></td><td style=\"width: 132px; text-align: center;\"><a title=\"Reset\" href=\"./zero\">Reset</a></td></tr></tbody>";
  //s += "<h1 style=\"color: #5e9ca0;\">Nik-Lidar</h1><p><strong><a title=\"Start\" href=\"./start?3\">Start</a>&nbsp; &nbsp;<a title=\"Home\" href=\"./zero\">Home</a>&nbsp; &nbsp;<a title=\"SD Card\" href=\"./SD\">SD Card</a></strong></p>" ; 
  s += "</html>\n";
  server.sendContent(s);
  listDir();
  server.client().stop();
}

void lidar_parser(Stream& S) {
    if (lidar.waitScanDot() == RESULT_OK) {
        scanPoint _point = lidar.getCurrentScanPoint();
        if(scans.count() <= SIZE_OF_SCAN_BUFFER){
            scans.push(_point);
        }else{
            scans.pop();
            scans.push(_point);
        }
    }else{
        //S.println(" YDLIDAR get Scandata failed!!");
        //restartScan(S);
    }
}

void Stepperzero()
{ 
digitalWrite(X_ENABLE_PIN,LOW);
digitalWrite(X_DIR_PIN, CCW);
  for(int i=0; i<3000; i++){
        digitalWrite(X_STEP_PIN, HIGH);
        delayMicroseconds(300);
        digitalWrite(X_STEP_PIN, LOW);
        delayMicroseconds(300);
    }
digitalWrite(X_ENABLE_PIN,HIGH);
HomePage();
}

void Steppermove(int direction,int nstep, int speed)
{ digitalWrite(X_ENABLE_PIN,LOW);
digitalWrite(X_DIR_PIN, direction);
  for(int i=0; i<nstep; i++){
        digitalWrite(X_STEP_PIN, HIGH);
        delayMicroseconds(speed);
        digitalWrite(X_STEP_PIN, LOW);
        delayMicroseconds(speed);
    }

}    

void restartScan(Stream& S){
    device_info deviceinfo;
    if (lidar.getDeviceInfo(deviceinfo, 100) == RESULT_OK) {
        int _samp_rate=4;
        String model;
        float freq = 7.0f;
        switch(deviceinfo.model){
            case 1:
                model="F4";
                _samp_rate=4;
                freq = 7.0;
                break;
            case 4:
                model="S4";
                _samp_rate=4;
                freq = 7.0;
                break;
            case 5:
                model="G4";
                _samp_rate=9;
                freq = 7.0;
                break;
            case 6:
                model="X4";
                _samp_rate=5;
                freq = 7.0;
                break;
            default:
                model = "Unknown";
        }

        uint16_t maxv = (uint16_t)(deviceinfo.firmware_version>>8);
        uint16_t midv = (uint16_t)(deviceinfo.firmware_version&0xff)/10;
        uint16_t minv = (uint16_t)(deviceinfo.firmware_version&0xff)%10;
        if(midv==0){
            midv = minv;
            minv = 0;
        }

        S.print("Firmware version:");
        S.print(maxv,DEC);
        S.print(".");
        S.print(midv,DEC);
        S.print(".");
        S.println(minv,DEC);

        S.print("Hardware version:");
        S.println((uint16_t)deviceinfo.hardware_version,DEC);

        S.print("Model:");
        S.println(model);

        S.print("Serial:");
        for (int i=0;i<16;i++){
            S.print(deviceinfo.serialnum[i]&0xff, DEC);
        }
        S.println("");

        S.print("[YDLIDAR INFO] Current Sampling Rate:");
        S.print(_samp_rate,DEC);
        S.println("K");

        S.print("[YDLIDAR INFO] Current Scan Frequency:");
        S.print(freq,DEC);
        S.println("Hz");
        delay(100);
        device_health healthinfo;
        if (lidar.getHealth(healthinfo, 100) == RESULT_OK){
            // detected...
            S.print("[YDLIDAR INFO] YDLIDAR running correctly! The health status:");
            S.println( healthinfo.status==0?"well":"bad");
            digitalWrite(M_EN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(500));
            if(lidar.startScan() == RESULT_OK){
                isScanning = true;
                //start motor in 1.8v
                //digitalWrite(YDLIDAR_MOTRO_EN, HIGH);
                S.println("Now YDLIDAR is scanning ......");
                server.sendContent("start scanning");
                //vTaskDelay(pdMS_TO_TICKS(1000));
            }else{
                S.println("start YDLIDAR is failed!  Continue........");
            }
        }else{
            S.println("cannot retrieve YDLIDAR health");
        }
    }else{
        S.println("YDLIDAR get DeviceInfo Error!!!");
    }
}

void scanlidar()
{ int MAX_POS = int((Microstep*200*0.5*2.5));
  MAX_POS = 300;
  int prev = 0;
  unsigned long time = 0;
  int samples = 0;
  pos=0;
  File file = SD.open("/text.xyz", FILE_WRITE);//need change in timestamp.xyz 
  restartScan(Serial);
  Serial.println(MAX_POS);
  Serial.println(pos);
  server.sendContent("<br>go!<br>");
    while (pos<=MAX_POS) {           
                 //Serial.println(pos);
                 lidar_parser(Serial2);
                 if(isScanning){
                   
                 if(scans.count() > 0){
                    scanPoint _point;
                    _point = scans.pop();
                    float distance = _point.distance; //ra distance value in mm unit
                    float angle    =  _point.angle; //anglue value in degree
                    if(distance > 0){
                        //buf.append((uint8_t)(((uint16_t)(angle*100)) >> 8));
                        //buf.append((uint8_t)(((uint16_t)(angle*100)) & 0xFF));
                        //buf.append((uint8_t)(((uint16_t)distance) >> 8));
                        //buf.append((uint8_t)(((uint16_t)distance) & 0xFF));
                        //buf.append((uint8_t)(((uint16_t)(pos)) >> 8));
                        //buf.append((uint8_t)(((uint16_t)(pos)) & 0xFF));
                        float x = distance * sin((angle*Pi)/180) * cos(((pos/22.2222222222)*Pi)/180);
                        float y = distance * sin((angle*Pi)/180) * sin(((pos/22.2222222222)*Pi)/180) ;
                        float z = distance * cos((angle*Pi)/180);
                        file.println(String(x) + " " + String(y)+ " " +String(z));
                        samples++;
                    }

                    if(prev-angle > 300){
                        int t = millis() - time;
                        time = millis();
                        Serial.println( "rev! time: " + String(t) + 
                                        " freq:" + String(1000.0/t) + 
                                        " samples:" + String(samples) + 
                                        " sps:" + String((float)samples/(t/1000.0)));
                        samples = 0;

                        pos += 1 - dir*2;
                        //Serial.println("step");
                        Steppermove(dir,1,400);
                        }
                    prev = angle;
                    
                    }
                }
                              
    }
digitalWrite(M_EN, LOW);
digitalWrite(X_ENABLE_PIN, HIGH);
file.close();
server.sendContent("end<br>");
}




void File_Download()
{
    File download = SD.open("/"+server.arg("download"));
    if (download) {
      server.sendHeader("Content-Type", "text/text");
      server.sendHeader("Content-Disposition", "attachment; filename="+server.arg("download"));
      server.sendHeader("Connection", "close");
      server.streamFile(download, "application/octet-stream");
      download.close();
    } else Serial.println("file not present download"); 
  
}


/////////////////////
// setup and main loop //
/////////////////////


void setup() 
{
  disableCore0WDT();
  disableCore1WDT();
  Serial.begin(250000);
  Serial2.begin(128000,SERIAL_8N1,25,26);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WiFiSSID, WiFiAPPSK);
  
  pinMode(M_EN, OUTPUT);
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);
  lidar.begin(Serial2, 128000);
  SD.begin();
    ///////////////////////////// Server Commands
  server.on("/",         HomePage);
  server.on("/download", File_Download);
  server.on("/start",    scanlidar);
  server.on("/zero",     Stepperzero);
  ///////////////////////////// End of Request commands
  server.begin();
  Serial.println("HTTP server started");

  digitalWrite(M_EN,HIGH);
  delay(1000);
  digitalWrite(M_EN,LOW);
  digitalWrite(X_ENABLE_PIN,HIGH);
  
}

void loop() 
{ 
  server.handleClient();
  delayMicroseconds(500);
}






