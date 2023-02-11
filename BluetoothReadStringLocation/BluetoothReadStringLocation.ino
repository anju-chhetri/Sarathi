#include <SoftwareSerial.h>
#define LED 13
#define buttonPin 5
int buttonState = 1;
  String num="0";

SoftwareSerial Blc(10, 11); //TX, RX
SoftwareSerial SIM900A(2,3); //TX, RX
bool numsend = false;
String number;
String name = "AT+CMGS=\"+9779840194288\"\r";
void setup()
{
    SIM900A.begin(115200);   // GSM Module Baud rate - communication speed 

  Blc.begin(9600);   // GSM Module Baud rate - communication speed
  Serial.begin(115200);    // Baud rate of Serial Monitor in the IDE app

}

void loop()
{
  String num;
  String part1 = "AT+CMGS=\"";
  String part2 = "\"\r"; 
  //Blc.println("AT+CMGS=\"+9779840194288\"\r"); // Receiver's Mobile Number
  //Serial.println((String)name);
  while (Blc.available()){
    char Data = Blc.read();
   Serial.println(Data);
//        if(Data=='\n'){break;}

    num+=Data;
    number = num;
    numsend = true;
//    Serial.println((int)Data);
    }
    if(numsend){
      if(number[0]=='+'){
    number.trim();
     name  = part1 + number + part2;
     Serial.println(name);
     numsend = false;
     Serial.println((String)name);
     }}
  if (Serial.available()>0)
   switch(Serial.read())
  {
    case 's':
      SendMessage();

      break;
  }
}


void SendMessage()
{     Serial.println(name);

  Serial.println("INside GSM");
  Serial.println("Sending Message please wait....");
  SIM900A.println("AT+CMGF=1");    //Text Mode initialisation
  delay(1000);
  Serial.println("Set SMS Number");
  SIM900A.println((String)name); // Receiver's Mobile Number
  delay(1000);
  Serial.println("Set SMS Content");
  SIM900A.println("Fall detected");// Messsage content
  delay(100);
  Serial.println("Done");
  SIM900A.println((char)26);//   delay(1000);
  Serial.println("Message sent succesfully");
  digitalWrite(13, HIGH);

}
