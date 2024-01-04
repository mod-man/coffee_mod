// Switch should be toward the edge of the board during typical operation

// in the USB debug mode the temperature sensor uses pin 16 which would typically be button B
// it does this to get off of pin 1 which needs to be used by the TX serial communication.

// Library for temperature sensor
#include <Adafruit_MAX31865.h>
// Library for I2C
#include <Wire.h>

// Library for PID control
#include <AutoPID.h>

// Libraries for LCD
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Libraries for wifi
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>

// Degug mode permits messages sent over usb connection
//#define DEBUG_MODE 1

// Define SSR pin location
#define SSR_PIN 15

// Define the LCD screen
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Wifi setup
const char *ssid = "wifi_name";
const char *password = "wifi_password";
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// variables for html page
String temp_string = String(0.0);
String PID_string = String(0.0);

// Initialize the temperature sensor
#ifdef DEBUG_MODE
Adafruit_MAX31865 thermo = Adafruit_MAX31865(16);
#else
Adafruit_MAX31865 thermo = Adafruit_MAX31865(1);
#endif

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF 430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL 100.0

// pid settings and gains
#define OUTPUT_MIN 0
#define OUTPUT_MAX 1000
#define KP 225
#define KI 0
#define KD 5000000000 // assumed 0.1 degree change at 1 degree off setpoint over 1 second iteration time gives 20 as output.
//(20 + 225*0.8) / 0.1
// library for PID control was wrong.
// variables for PID
double temperature, setPoint, outputVal;
double counter;

// constant for file system
const char *setPointFileName = "/setPoint.txt";

// menu variables
int menu_screen = 0;
bool steamState = false;

const int buttonA = 0;
#ifdef DEBUG_MODE
const int buttonB = 0; // button cannot be used (used for temp sensor while serial connected)
#else
const int buttonB = 16;
#endif
const int buttonC = 2;
int buttonAstate = 0;
int buttonBstate = 0;
int buttonCstate = 0;
int buttonAstate2 = 0;
int buttonBstate2 = 0;
int buttonCstate2 = 0;
int buttonAtime = 0;
int buttonBtime = 0;
int buttonCtime = 0;
bool buttonAshort = false;
bool buttonAlong = false;
bool buttonBshort = false;
bool buttonBlong = false;
bool buttonCshort = false;
bool buttonClong = false;
const int LongPressLimit = 1000;

bool timer_enable = false;
double timer_val;
double reference_time = millis();

static const unsigned char PROGMEM image_data_Saraarray[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xfe, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x3f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xe0, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x01, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xe0, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0xff, 0xff, 0xe0, 0x07, 0xff, 0xff,
    0xff, 0xfe, 0x00, 0xf3, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0xff, 0xff, 0x9f, 0x00, 0xff, 0xff,
    0xff, 0xf0, 0x1f, 0xf3, 0xff, 0x03, 0x81, 0xf0, 0xf3, 0x33, 0xe0, 0xff, 0x0f, 0xf0, 0x0f, 0xff,
    0xff, 0xc0, 0xff, 0xf0, 0x0f, 0x03, 0x01, 0xe0, 0x73, 0x33, 0xc0, 0x7e, 0x6f, 0xfc, 0x03, 0xff,
    0xff, 0x81, 0xff, 0xf0, 0x06, 0x7f, 0xfc, 0xcf, 0x33, 0x33, 0xff, 0x3e, 0x60, 0xff, 0x01, 0xff,
    0xff, 0x81, 0xff, 0xf3, 0xe2, 0x7e, 0x00, 0xc3, 0xf3, 0x33, 0x80, 0x3c, 0xfe, 0x7f, 0x80, 0xff,
    0xff, 0x81, 0xff, 0xf3, 0xf2, 0x7c, 0x00, 0xf0, 0x73, 0x33, 0x00, 0x3f, 0xfc, 0xff, 0x80, 0xff,
    0xff, 0xc0, 0xff, 0xf3, 0xf2, 0x7c, 0xfc, 0xff, 0x33, 0x33, 0x3f, 0x3f, 0xf1, 0xff, 0x81, 0xff,
    0xff, 0xe0, 0x7f, 0xf0, 0x02, 0x7c, 0x00, 0xc0, 0x33, 0x33, 0x8f, 0xff, 0xe7, 0xff, 0x01, 0xff,
    0xff, 0xfc, 0x1f, 0xf8, 0x06, 0x7e, 0x01, 0xc0, 0x73, 0x33, 0xc0, 0x00, 0x0f, 0xf8, 0x07, 0xff,
    0xff, 0xff, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x7f, 0xff,
    0xff, 0xff, 0xf0, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x07, 0xff, 0xff,
    0xff, 0xff, 0xff, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x80, 0x01, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x7f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

// set up the PID control
AutoPID myPID(&temperature, &setPoint, &outputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

void setup()
{
  Serial.begin(115200);

  thermo.begin(MAX31865_3WIRE); // set to 2WIRE or 4WIRE as necessary

  pinMode(SSR_PIN, OUTPUT); // set SSR pin to output

  // set up the PID
  myPID.setBangBang(20.0, 20.0);
  myPID.setTimeStep(1000);

  // set up buttons
  pinMode(buttonA, INPUT);
  pinMode(buttonB, INPUT);
  pinMode(buttonC, INPUT);

  // set up screen
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
#ifdef DEGUG_MODE
    Serial.println(F("SSD1306 allocation failed"));
#endif
    for (;;)
      ;
  }
  delay(1000); // Pause for 1 second

  // Clear the buffer.
  display.clearDisplay();

  // Draw bitmap on the screen
  display.drawBitmap(0, 0, image_data_Saraarray, 128, 32, 1);
  display.display();

  // Initialize SPIFFS
  if (!SPIFFS.begin())
  {
#ifdef DEGUG_MODE
    Serial.println("An Error has occurred while mounting SPIFFS");
#endif
    return;
  }

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  int wifi_timeout = 0;
  while (WiFi.status() != WL_CONNECTED and wifi_timeout < 5)
  {
#ifdef DEGUG_MODE
    Serial.println("Connecting to WiFi..");
#endif
    delay(1000);
    wifi_timeout += 1;
  }

// Print ESP32 Local IP Address
#ifdef DEBUG_MODE
  Serial.println(WiFi.localIP());
#endif

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index.html"); });
  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/plain", temp_string.c_str()); });
  server.on("/PID_output", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/plain", PID_string.c_str()); });

  // Start server
  server.begin();

  if (SPIFFS.exists(setPointFileName)) // File exists, open file and pull the setpoint
  {
    String setPointString = load_from_file(setPointFileName);
    setPoint = setPointString.toDouble();
  }
  else // File does not exist, create the file
  {
    SPIFFS.mkdir(setPointFileName);                     // create file
    setPoint = 95.0;                                    // original setpoint value
  }

  delay(1000); // Pause for 1 second
}

void loop()
{

  // run the menu
  if (menu_screen == 0)
  {
    disp_menu();
  }
  else if (menu_screen == 1)
  {
    disp_PIDsettings();
  }
  else if (menu_screen == 2)
  {
    disp_LANsettings();
  }

  // run the timer
  if (timer_enable == true)
  {
    timer_val = (millis() - reference_time) / 1000;
  }
  else
  {
    reference_time = millis();
  }

  CheckButtons(); // check for button presses

  // cycle the menu screen based on button A
  if (buttonAshort == true)
  {
    if (menu_screen == 0)
    {
      menu_screen = 1;
      reference_time = millis(); // reset the timer
      timer_val = 0.0;
      timer_enable = false;
    }
    else if (menu_screen == 1)
      menu_screen = 2;
    else
      menu_screen = 0;
  }

  // button B and C operation based on menu screen.
  if (buttonBshort == true)
  {
    if (menu_screen == 0)
    {
      if (timer_enable == true)
      {
        timer_enable = false;
      }
      else
      {
#ifdef DEBUG_MODE
        timer_enable = false;
#else
        timer_enable = true;
#endif
        reference_time = millis() - timer_val * 1000;
      }
    }
    else if (menu_screen == 1)
    {
#ifdef DEBUG_MODE
      setPoint = setPoint;
#else
      setPoint += 0.5;
      write_to_file(setPointFileName, String(setPoint, 1));
#endif
    }
  }
  if (buttonCshort == true)
  {
    if (menu_screen == 0)
    {
      steamState = not(steamState);
    }
    else if (menu_screen == 1)
    {
      setPoint -= 0.5;
      write_to_file(setPointFileName, String(setPoint, 1));
    }
  }

  // pull the temeprature
  uint16_t rtd = thermo.readRTD();
  temperature = thermo.temperature(RNOMINAL, RREF);

  myPID.run(); // for AutoPID

  // set the SSR duty cycle depending on PID output.
  // also turn on the SSR if timer is enabled.
  counter = millis() % 1000;
  if ((outputVal > counter) || (steamState == true))
  {
    digitalWrite(SSR_PIN, HIGH);
  }
  else
  {
    digitalWrite(SSR_PIN, LOW);
  }
// Print internal values in debug mode
#ifdef DEBUG_MODE
  Serial.print("Set temp = ");
  Serial.print(setPoint);
  Serial.print(", Temp = ");
  Serial.print(temperature);
  Serial.print(", PID output = ");
  Serial.println(outputVal);
#endif

  temp_string = String(temperature);
  PID_string = String(outputVal / 10);

  if (myPID.atSetPoint(2))
    myPID.setOutputRange(0, 500); // set output range to half if we are within 2 degrees of our temp
  else
    myPID.setOutputRange(0, 1000);
}

void disp_menu()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("Time ");
  display.print(timer_val, 1);
  display.print(" Temp ");
  display.print(temperature, 1);
  display.print('\n');
  display.println("A - next menu");
  display.println("B - start/stop timer");
  if (steamState == false)
  {
    display.println("C - steam OFF");
  }
  else
  {
    display.println("C - steam ON");
  }
  display.display();
}

void disp_PIDsettings()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("Set ");
  display.print(setPoint, 1);
  display.print(" Temp ");
  display.print(temperature, 1);
  display.print('\n');
  display.println("A - next menu");
  display.println("B - increase temp");
  display.println("C - decrease temp");
  display.display();
}

void disp_LANsettings()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("SSID: ");
  display.println(ssid);
  display.print("Password: ");
  display.println(password);
  display.print("IP: ");
  display.println(WiFi.localIP());
  // display.println("A - next menu");
  display.display();
}

void CheckButtons()
{
  buttonAstate = digitalRead(buttonA);
  buttonBstate = digitalRead(buttonB);
  buttonCstate = digitalRead(buttonC);

  if ((buttonAstate == LOW) && (buttonAstate2 == HIGH))
  {
    buttonAtime = millis();
    buttonAshort = false;
    buttonAlong = false;
  }
  else if ((buttonAstate == HIGH && buttonAstate2 == LOW))
  {
    buttonAtime = millis() - buttonAtime;
    if (buttonAtime < LongPressLimit)
    {
      buttonAshort = true;
      buttonAlong = false;
    }
    else
    {
      buttonAshort = false;
      buttonAlong = true;
    }
  }
  else
  {
    buttonAshort = false;
    buttonAlong = false;
  }

  if (buttonBstate == LOW && buttonBstate2 == HIGH)
  {
    buttonBtime = millis();
    buttonBshort = false;
    buttonBlong = false;
  }
  else if (buttonBstate == HIGH && buttonBstate2 == LOW)
  {
    buttonBtime = millis() - buttonBtime;
    if (buttonBtime < LongPressLimit)
    {
      buttonBshort = true;
      buttonBlong = false;
    }
    else
    {
      buttonBshort = false;
      buttonBlong = true;
    }
  }
  else
  {
    buttonBshort = false;
    buttonBlong = false;
  }

  if (buttonCstate == LOW && buttonCstate2 == HIGH)
  {
    buttonCtime = millis();
    buttonCshort = false;
    buttonClong = false;
  }
  else if (buttonCstate == HIGH && buttonCstate2 == LOW)
  {
    buttonCtime = millis() - buttonCtime;
    if (buttonCtime < LongPressLimit)
    {
      buttonCshort = true;
      buttonClong = false;
    }
    else
    {
      buttonCshort = false;
      buttonClong = true;
    }
  }
  else
  {
    buttonCshort = false;
    buttonClong = false;
  }

  buttonAstate2 = buttonAstate;
  buttonBstate2 = buttonBstate;
  buttonCstate2 = buttonCstate;
}

bool write_to_file(String file_name, String contents)
{
  File this_file = SPIFFS.open(file_name, "w");
  if (!this_file)
  { // failed to open the file, return false
    return false;
  }
  int bytesWritten = this_file.print(contents);

  if (bytesWritten == 0)
  { // write failed
    return false;
  }

  this_file.close();
  return true;
}

String load_from_file(String file_name)
{
  String result = "";

  File this_file = SPIFFS.open(file_name, "r");
  if (!this_file)
  { // failed to open the file, retrn empty result
    return result;
  }
  while (this_file.available())
  {
    result += (char)this_file.read();
  }

  this_file.close();
  return result;
}
