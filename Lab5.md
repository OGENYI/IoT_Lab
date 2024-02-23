<!-- Updates to this file will be overwritten -->
<!-- markdown-link-check-disable -->

|  School of Computing |  |
| --------------- | --------------- |
| Title | Internet of Things |
| Module Coordinator| Uchenna Ogenyi |
| Email | [uchenna.ogenyi@port.ac.uk](mailto:uchenna.ogenyi@port.ac.uk) |
| Code | M30226|


<!-- markdown-link-check-enable -->
## Internet of Things

Learning objectives:
<!-- TODO -->
* ESP-01 and WiFi

### Notes and Advice

<!-- markdown-link-check-disable -->
* The [Extenuating Circumstances procedure](https://www.upsu.net/advice) is
  there to support you if you have had any circumstances (problems) that have
  been serious or significant enough to prevent you from attending, completing
  or submitting an assessment on time.
* [ASDAC](http://www2.port.ac.uk/additional-support-and-disability-advice-centre/)
  are available to any students who disclose a disability or require additional
  support for their academic studies with a good set of resources on the [ASDAC
  Moodle site](https://moodle.port.ac.uk/course/view.php?id=3012)
* The University takes plagiarism seriously. Please ensure you adhere to the
  plagiarism guidelines [https://www.upsu.net/advice/plagiarism](https://www.upsu.net/advice/plagiarism).
* Any material included in your coursework should be
  fully cited and referenced in APA format (**seventh** edition). Detailed advice on
  referencing is available from [http://referencing.port.ac.uk/](http://referencing.port.ac.uk/)
* Any material submitted that does not meet format or submission guidelines, or
  falls outside of the submission deadline could be subject to a cap on your
  overall result or disqualification entirely.
* If you need additional assistance, you can ask your personal tutor, learning
  support ana.baker@port.ac.uk and xia.han@port.ac.uk or your lecturers.
<!-- markdown-link-check-enable-->

### Lab Notes

* Working with ESP-01 module 


### Lab 5 Part 1

Using WiFi on your Arduino (Ensure the parts below work fine)

Wire the ESP-01 onto software serial and send some data. We will use pins 2 and 3 for the serial connection to the arduino.

Connect as follows:

  ```text
  ESP-01 TX  --> Arduino pin 2
  ESP-01 RX  --> Arduino pin 3
  ESP-01 GND --> Arduino GND
  ESP-01 CH_PD (Chip enable) --> Arduino 3.3v
  ESP-01 VCC/3.3v --> Arduino 3.3v
  ```

Extend your (currently empty) arduino script to import the serial library.

```c
#include <Arduino.h>
#include <SoftwareSerial.h>
```

Configure the serial port:

```c
#define DEBUG true
#define serialCommunicationSpeed 9600
SoftwareSerial esp8266(2,3);
```

You can just send AT commands to the serial port, but remember to wait for an answer, and check everything went well. Here is some code that will connect to a WiFi and assist with  output. Be sure you understand this code and ensure it works.

You will need an access point (Phone/Laptop, not eduroam) to connect your ESP-01 to, use the code below to test you can connect.

```c
#include <Arduino.h>
#include <SoftwareSerial.h>


#define DEBUG true
#define serialCommunicationSpeed 9600
SoftwareSerial esp8266(2,3);
//int wifiEnablePin = 8;


String sendData(String command, const int timeout, boolean debug)
{
    String response = "";                                             //initialize a String variable named "response". we will use it later.
if(debug)                                                         //if the "debug" variable value is TRUE, print the response on the Serial monitor.
    {
      Serial.print(command);
    }
    esp8266.print(command);                                           //send the AT command to the esp8266 (from ARDUINO to ESP8266).
    long int time = millis();                                         //get the operating time at this specific moment and save it inside the "time" variable.
    while( (time+timeout) > millis())                                 //excute only whitin 1 second.
    {
      while(esp8266.available())                                      //is there any response came from the ESP8266 and saved in the Arduino input buffer?
      {
        char c = esp8266.read();                                      //if yes, read the next character from the input buffer and save it in the "response" String variable.
        response+=c;                                                  //append the next character to the response variabl. at the end we will get a string(array of characters) contains the response.
      }
    }
    if(debug)                                                         //if the "debug" variable value is TRUE, print the response on the Serial monitor.
    {
      Serial.print(response);
    }
    return response;                                                  //return the String response.
}

void InitWifiModule()
{
  sendData("AT+RST\r\n", 2000, DEBUG);                                                  //reset the ESP8266 module.
  //delay(1000);
  sendData("AT+CWJAP=\"Device Error.\",\"TestTest\"\r\n", 2000, DEBUG);        //connect to the WiFi network.
  delay (3000);
  sendData("AT+CWMODE=3\r\n", 1500, DEBUG);                                             //set the ESP8266 WiFi mode to station mode.
  delay (1000);
  sendData("AT+CIFSR\r\n", 1500, DEBUG);                                                //Show IP Address, and the MAC Address.
  delay (1000);
  sendData("AT+CIPMUX=1\r\n", 1500, DEBUG);                                             //Multiple conections.
  delay (1000);
  sendData("AT+CIPSERVER=1,80\r\n", 1500, DEBUG);                                       //start the communication at port 80, port 80 used to communicate with the web servers through the http requests.
}


void setup() {

 Serial.begin(serialCommunicationSpeed);
 esp8266.begin(serialCommunicationSpeed);
 InitWifiModule();

}

void loop() {
  // put your main code here, to run repeatedly:
}
```

# Lab 5 Part 2

Your arduino does not understand or know about the current date/time. So we need to get it from an external source.

* Read and understand what `NTP` is [https://en.wikipedia.org/wiki/Network_Time_Protocol](https://en.wikipedia.org/wiki/Network_Time_Protocol)

Communbicating to your ESP-01 via AT commands can get very tiring as there are lots of strings to parse. There are libraries that will help you with this. We will use an old but reliable library called `WiFiESP`

* Install the `WiFiESP` library, we will work through the examples. [https://github.com/bportaluri/WiFiEsp](https://github.com/bportaluri/WiFiEsp)

# Lab 5 Part 3

Get the correct time from an NTP server on the internet. You will need:

* WiFi connection
* ESP-01 wired correctly (Check connection pin numbers)

You must get the correct time returned.

```c
/*
 WiFiEsp example: UdpNTPClient

 Get the time from a Network Time Protocol (NTP) time server.
 Demonstrates use of UDP to send and receive data packets
 For more on NTP time servers and the messages needed to communicate with them,
 see http://en.wikipedia.org/wiki/Network_Time_Protocol

 NOTE: The serial buffer size must be larger than 36 + packet size
 In this example we use an UDP packet of 48 bytes so the buffer must be
 at least 36+48=84 bytes that exceeds the default buffer size (64).
 You must modify the serial buffer size to 128
 For HardwareSerial modify _SS_MAX_RX_BUFF in
   Arduino\hardware\arduino\avr\cores\arduino\SoftwareSerial.h
 For SoftwareSerial modify _SS_MAX_RX_BUFF in
   Arduino\hardware\arduino\avr\libraries\SoftwareSerial\SoftwareSerial.h
*/

#include "WiFiEsp.h"
#include "WiFiEspUdp.h"

// Emulate Serial1 on pins 6/7 if not present
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(2, 3); // RX, TX
#endif

char ssid[] = "SSID";            // your network SSID (name)
char pass[] = "PASSWORD";        // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status

char timeServer[] = "time.nist.gov";  // NTP server
unsigned int localPort = 2390;        // local port to listen for UDP packets

const int NTP_PACKET_SIZE = 48;  // NTP timestamp is in the first 48 bytes of the message
const int UDP_TIMEOUT = 2000;    // timeout in miliseconds to wait for an UDP packet to arrive

byte packetBuffer[NTP_PACKET_SIZE]; // buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiEspUDP Udp;



// send an NTP request to the time server at the given address
void sendNTPpacket(char *ntpSrv)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)

  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(ntpSrv, 123); //NTP requests are to port 123

  Udp.write(packetBuffer, NTP_PACKET_SIZE);

  Udp.endPacket();
}


void setup()
{
  // initialize serial for debugging
  Serial.begin(9600);
  // initialize serial for ESP module
  Serial1.begin(9600);
  // initialize ESP module
  WiFi.init(&Serial1);

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }

  // you're connected now, so print out the data
  Serial.println("You're connected to the network");

  Udp.begin(localPort);
}

void loop()
{
  sendNTPpacket(timeServer); // send an NTP packet to a time server

  // wait for a reply for UDP_TIMEOUT miliseconds
  unsigned long startMs = millis();
  while (!Udp.available() && (millis() - startMs) < UDP_TIMEOUT) {}

  Serial.println(Udp.parsePacket());
  if (Udp.parsePacket()) {
    Serial.println("packet received");
    // We've received a packet, read the data from it into the buffer
    Udp.read(packetBuffer, NTP_PACKET_SIZE);

    // the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print("Seconds since Jan 1 1900 = ");
    Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    Serial.println(epoch);


    // print the hour, minute and second:
    Serial.print("The UTC time is ");       // UTC is the time at Greenwich Meridian (GMT)
    Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
    Serial.print(':');
    if (((epoch % 3600) / 60) < 10) {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
    Serial.print(':');
    if ((epoch % 60) < 10) {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.println(epoch % 60); // print the second
  }
  // wait ten seconds before asking for the time again
  delay(10000);
}
```

## Lab 5 Part 4

Arduino have a webpage that will return some text (their logo) ; you can see this here [https://www.arduino.cc/asciilogo.txt](https://www.arduino.cc/asciilogo.txt)

* Get this logo using you hardware setup. You will need WiFi and the correct hardware configuration.
* The example below does not work, can you fix it?

```c
/*
 WiFiEsp example: WebClient

 This sketch connects to google website using an ESP8266 module to
 perform a simple web search.

 For more details see: http://yaab-arduino.blogspot.com/p/wifiesp-example-client.html
*/

#include "WiFiEsp.h"

// Emulate Serial1 on pins 6/7 if not present
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(2,3); // RX, TX
#endif

char ssid[] = "SSID";            // your network SSID (name)
char pass[] = "Password";        // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status

char server[] = "arduino.cc";

// Initialize the Ethernet client object
WiFiEspClient client;


void printWifiStatus()
{
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void setup()
{
  // initialize serial for debugging
  Serial.begin(9600);
  // initialize serial for ESP module
  Serial1.begin(9600);
  // initialize ESP module
  WiFi.init(&Serial1);

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }

  // you're connected now, so print out the data
  Serial.println("You're connected to the network");

  printWifiStatus();

  Serial.println();
  Serial.println("Starting connection to server...");
  // if you get a connection, report back via serial
  if (client.connect(server, 80)) {
    Serial.println("Connected to server");
    // Make a HTTP request
    client.println("GET /asciilogo.txt HTTP/1.1");
    client.println("Host: arduino.cc");
    client.println("Connection: close");
    client.println();
  }
}

void loop()
{
  // if there are incoming bytes available
  // from the server, read them and print them
  while (client.available()) {
    char c = client.read();
    Serial.write(c);
  }

  // if the server's disconnected, stop the client
  if (!client.connected()) {
    Serial.println();
    Serial.println("Disconnecting from server...");
    client.stop();

    // do nothing forevermore
    while (true);
  }
}


```

# Lab 5 Part 5

Have a look at the other examples [https://github.com/bportaluri/WiFiEsp/tree/master/examples](https://github.com/bportaluri/WiFiEsp/tree/master/examples)

* You man want to consider doing another example
* You may want to consider looking for an alternative library.
