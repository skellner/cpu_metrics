//
// Use one MAX7219 to control 8-digit seven-segment display
// the display module: http://www.icshop.com.tw/product_info.php/products_id/20686
//
// MAX7219 datasheet: https://datasheets.maximintegrated.com/en/ds/MAX7219-MAX7221.pdf
//

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h> 
#include <WiFiUdp.h>
#include <ArduinoJson.h>

#include <ESP8266HTTPClient.h>

const char* WSSD = "crackhouse";
const char* WPW = "danmotherf4cker";

ESP8266WiFiMulti WiFiMulti;

unsigned int counter = 0;

// the MAX7219 address map (datasheet table 2)
#define MAX7219_DECODE_REG      (0x09)
#define MAX7219_INTENSITY_REG   (0x0A)
#define MAX7219_SCANLIMIT_REG   (0x0B)
#define MAX7219_SHUTDOWN_REG    (0X0C)
#define MAX7219_DISPLAYTEST_REG (0x0F)
#define MAX7219_DIGIT_REG(pos)  ((pos) + 1)

// shutdown mode (datasheet table 3)
#define MAX7219_OFF             (0x0)
#define MAX7219_ON              (0x1)

// pin 13 of MAX7219 (CLK)
const int clock_pin = D2;
// pin 12 of MAX7219 (LOAD)
const int data_latch_pin = D1;
// pin 1 of MAX7219 (DIN)
const int data_input_pin = D5;

// digit pattern for a 7-segment display. datasheet table 5
const byte digit_pattern[25] =
{
  B01111110,  // 0
  B00110000,  // 1
  B01101101,  // 2
  B01111001,  // 3
  B00110011,  // 4
  B01011011,  // 5
  B01011111,  // 6
  B01110000,  // 7
  B01111111,  // 8
  B01111011,  // 9
  B01110111,  // A 10
  B00011111,  // b 11
  B01001110,  // C 12
  B00111101,  // d 13
  B01001111,  // E 14
  B01000111,  // F 15
  B00000001,  // - 16
  B00000000,  // BLANK 17
  B01001110,  // C 18
  B01100111,  // P 19
  B00111110,  // U 20
  B01011110,   // G 21
  B01110110,   // N 22
  B01111110,   // O 23
  B00001111   // t 24
};

#define DP_FLAG       (B10000000)
#define NUM_OF_DIGITS (8)

//unsigned int digit_base = 10;
//unsigned int number = 1234; 
unsigned int display_selector = 0;


// update the register value of MAX7219
void set_register(byte address, byte value)  
{
  digitalWrite(data_latch_pin, LOW);
  shiftOut(data_input_pin, clock_pin, MSBFIRST, address);
  shiftOut(data_input_pin, clock_pin, MSBFIRST, value);
  digitalWrite(data_latch_pin, HIGH);
}

void init_max7219()
{
  // disable test mode. datasheet table 10
  set_register(MAX7219_DISPLAYTEST_REG, MAX7219_OFF);
  // set medium intensity. datasheet table 7
  set_register(MAX7219_INTENSITY_REG, 0x8);
  // turn off display. datasheet table 3
  set_register(MAX7219_SHUTDOWN_REG, MAX7219_OFF);
  // drive 8 digits. datasheet table 8
  set_register(MAX7219_SCANLIMIT_REG, 7);
  // no decode mode for all positions. datasheet table 4
  set_register(MAX7219_DECODE_REG, B00000000);
}


void setup()  
{

  Serial.begin(115200);
  Serial.println();
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WSSD, WPW);

  Serial.println("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    writeMsg(1);
  }
 
  
  Serial.println("");
  
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

    // init pin states
  pinMode(clock_pin, OUTPUT);
  pinMode(data_latch_pin, OUTPUT);    
  pinMode(data_input_pin, OUTPUT);

  // init MAX2719 states
  init_max7219();

}

void loop(){
  
  delay(2000);

  if (display_selector == 0){
    display_selector = 1;
  } else {
    display_selector = 0;
  }


  WiFiClient client;  // or WiFiClientSecure for HTTPS
  HTTPClient http;

  // Send request
  http.useHTTP10(true);
  http.begin(client, "http://192.168.86.37:8085/data.json");
  http.GET();

  //DeserializationOption::NestingLimit(12).

  // Parse response
  DynamicJsonDocument doc(24576);
  DeserializationError error = deserializeJson(doc, http.getStream(),DeserializationOption::NestingLimit(12));

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    writeMsg(0);
    return;
  }

  int id = doc["id"]; // 0
  const char* Text = doc["Text"]; // "Sensor"
  
  JsonObject Children_0 = doc["Children"][0];
  int Children_0_id = Children_0["id"]; // 1
  const char* Children_0_Text = Children_0["Text"]; // "DESKTOP-29MVFVC"
  JsonArray Children_0_Children = Children_0["Children"];
  JsonObject Children_0_Children_1 = Children_0_Children[1];
  int Children_0_Children_1_id = Children_0_Children_1["id"]; // 31
  const char* Children_0_Children_1_Text = Children_0_Children_1["Text"]; // "AMD Ryzen 5 5600X"
  JsonArray Children_0_Children_1_Children = Children_0_Children_1["Children"];
  JsonObject Children_0_Children_1_Children_1 = Children_0_Children_1_Children[1];
  int Children_0_Children_1_Children_1_id = Children_0_Children_1_Children_1["id"]; // 40
  const char* Children_0_Children_1_Children_1_Text = Children_0_Children_1_Children_1["Text"];

if (display_selector == 0){
    for (JsonObject Children_0_Children_1_Children_1_Child : Children_0_Children_1_Children_1["Children"].as<JsonArray>()) {
    
      int Children_0_Children_1_Children_1_Child_id = Children_0_Children_1_Children_1_Child["id"]; // 41, 42
      const char* Children_0_Children_1_Children_1_Child_Text = Children_0_Children_1_Children_1_Child["Text"];
    
      const char* Children_0_Children_1_Children_1_Child_Min = Children_0_Children_1_Children_1_Child["Min"];
      const char* Children_0_Children_1_Children_1_Child_Value = Children_0_Children_1_Children_1_Child["Value"];
      const char* Children_0_Children_1_Children_1_Child_Max = Children_0_Children_1_Children_1_Child["Max"];
      const char* Children_0_Children_1_Children_1_Child_ImageURL = Children_0_Children_1_Children_1_Child["ImageURL"];
    
      int x=0;
      char foo[12] = "CPU Package";
      int res = 0;
      for(int i=0;i<11;i++){
        if(Children_0_Children_1_Children_1_Child_Text[i]==foo[i]){
          res = 1;
        }else{
          res = 0;
          break;
        }
      }
      if (res == 1) {
       writeNumber(Children_0_Children_1_Children_1_Child_Value);
      }
    }
}
  
  JsonObject Children_0_Children_3 = Children_0_Children[3];
  int Children_0_Children_3_id = Children_0_Children_3["id"]; // 66
  const char* Children_0_Children_3_Text = Children_0_Children_3["Text"]; // "NVIDIA NVIDIA GeForce RTX ...
  JsonArray Children_0_Children_3_Children = Children_0_Children_3["Children"];
  JsonObject Children_0_Children_3_Children_1 = Children_0_Children_3_Children[1];
  int Children_0_Children_3_Children_1_id = Children_0_Children_3_Children_1["id"]; // 71
  const char* Children_0_Children_3_Children_1_Text = Children_0_Children_3_Children_1["Text"];
  JsonObject Children_0_Children_3_Children_1_Children_0 = Children_0_Children_3_Children_1["Children"][0];
  int Children_0_Children_3_Children_1_Children_0_id = Children_0_Children_3_Children_1_Children_0["id"];
  const char* Children_0_Children_3_Children_1_Children_0_Text = Children_0_Children_3_Children_1_Children_0["Text"];
  
  const char* Children_0_Children_3_Children_1_Children_0_Min = Children_0_Children_3_Children_1_Children_0["Min"];
  const char* Children_0_Children_3_Children_1_Children_0_Value = Children_0_Children_3_Children_1_Children_0["Value"];

  if (display_selector == 1){
     writeNumber(Children_0_Children_3_Children_1_Children_0_Value);
  }
  
  
  // Disconnect
  http.end();
}

void writeMsg(const int msg) {

  unsigned int no_conn[8] = {17,22,22,23,18,17,23,22}; // NO CONN, msg 1
  unsigned int no_data[8] = {17,10,24,10,13,17,23,22}; // NO DATA, msg 0

  int i;
  
  for (i = 0; i < NUM_OF_DIGITS; i++) {
    if (msg == 1) {
      set_register(MAX7219_DIGIT_REG(i),digit_pattern[no_conn[i]]);
    } else {
      set_register(MAX7219_DIGIT_REG(i),digit_pattern[no_data[i]]);
    }
  }

  // turn on display
  set_register(MAX7219_SHUTDOWN_REG, MAX7219_ON);
}
  
void writeNumber(const char *foo) {
  
  int i;
  byte bytearr[5];

  // turn off display first
  set_register(MAX7219_SHUTDOWN_REG, MAX7219_OFF);
  
  if (foo[2] == ',') { // two digit temp
    bytearr[3] = digit_pattern[17];
    bytearr[2]= digit_pattern[char2uint(foo[0])];
    bytearr[1]= digit_pattern[char2uint(foo[1])];
    bytearr[1] = bytearr[1] + DP_FLAG;
    bytearr[0]= digit_pattern[char2uint(foo[3])];
    bytearr[0]= digit_pattern[char2uint(foo[3])];
 }

 if (foo[3] == ',') { // three digit temp
    Serial.println("3-digit");
    bytearr[3] = digit_pattern[char2uint(foo[0])];
    bytearr[2]= digit_pattern[char2uint(foo[1])];
    bytearr[1]= digit_pattern[char2uint(foo[2])];
    bytearr[1] = bytearr[1] + DP_FLAG;
    bytearr[0]= digit_pattern[char2uint(foo[4])];
 }

  for (i = 0; i < NUM_OF_DIGITS-4; i++) {
    set_register(MAX7219_DIGIT_REG(i), bytearr[i]);
  }

  
// CPU or GPU
  set_register(MAX7219_DIGIT_REG(NUM_OF_DIGITS-4),digit_pattern[17]);
  set_register(MAX7219_DIGIT_REG(NUM_OF_DIGITS-3),digit_pattern[20]);
  set_register(MAX7219_DIGIT_REG(NUM_OF_DIGITS-2),digit_pattern[19]);
  if (display_selector == 0){
    set_register(MAX7219_DIGIT_REG(NUM_OF_DIGITS-1),digit_pattern[18]);
  } else {
    set_register(MAX7219_DIGIT_REG(NUM_OF_DIGITS-1),digit_pattern[21]);
  }

  // turn on display
  set_register(MAX7219_SHUTDOWN_REG, MAX7219_ON);

}

unsigned int char2uint (char chr) {
  return chr - '0';
}
