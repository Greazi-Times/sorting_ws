#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

// ------------ ROS Setup ------------
ros::NodeHandle nh;

// ------------ Transport Variabelen ------------
const int motorIn1 = 2;
const int motorIn2 = 3;
const int motorPWM = 4;
const int beginSensorPin = A2;
const int endSensorPin   = A3;

bool transport_actief = false;
int temp_begin = 0;
int temp_end = 0;

// ------------ Signaleringsvariabelen ------------
const int BUZZER_CONS  = 6;
const int RED_CONS     = 7;
const int ORANGE_CONS  = 8;
const int GREEN_CONS   = 9;
const int BUZZER_BEEP  = 10;
const int RED_BLINK    = 11;
const int ORANGE_BLINK = 12;
const int GREEN_BLINK  = 13;

// ------------ ROS Publishers ------------
std_msgs::String status_msg;
ros::Publisher status_pub("transportband/motor", &status_msg);

std_msgs::Bool object_msg;
ros::Publisher object_pub("transportsystem/sensor/end", &object_msg);

std_msgs::Bool begin_object_msg;
ros::Publisher begin_object_pub("transportsystem/sensor/start", &begin_object_msg);

// ------------ Functies Transport Detectie ------------
bool isProductAanwezigBegin(int value) {
  if (value <= 500) {
    temp_begin++;
    return (temp_begin >= 2);
  } else {
    temp_begin = 0;
    return false;
  }
}

bool isProductAanwezigEnd(int value) {
  if (value <= 500) {
    temp_end++;
    return (temp_end >= 2);
  } else {
    temp_end = 0;
    return false;
  }
}

// ------------ Callback Transport Commando ------------
void commandCallback(const std_msgs::Bool &msg) {
  transport_actief = msg.data;

  if (transport_actief) {
    digitalWrite(motorIn1, HIGH);
    digitalWrite(motorIn2, LOW);
    analogWrite(motorPWM, 255);
    status_msg.data = "motor gestart";
  } else {
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, LOW);
    analogWrite(motorPWM, 0);
    status_msg.data = "motor gestopt";
  }

  status_pub.publish(&status_msg);
}

// ------------ Signaleringsfuncties ------------
int getLEDPin(String color, bool blink) {
  color.toUpperCase();

  if (color == "RED")    return blink ? RED_BLINK : RED_CONS;
  if (color == "ORANGE") return blink ? ORANGE_BLINK : ORANGE_CONS;
  if (color == "GREEN")  return blink ? GREEN_BLINK : GREEN_CONS;
  if (color == "BUZZER") return blink ? BUZZER_BEEP : BUZZER_CONS;

  return -1;
}

void lightIndicationConstantCallback(const std_msgs::String &msg) {
  int pin = getLEDPin(msg.data, false);
  if (pin >= 0) digitalWrite(pin, HIGH);
}

void lightIndicationBlinkCallback(const std_msgs::String &msg) {
  int pin = getLEDPin(msg.data, true);
  if (pin >= 0) digitalWrite(pin, HIGH);
}

void lightIndicationOffCallback(const std_msgs::String &msg) {
  int pinConst = getLEDPin(msg.data, false);
  int pinBlink = getLEDPin(msg.data, true);

  if (pinConst >= 0) digitalWrite(pinConst, LOW);
  if (pinBlink >= 0) digitalWrite(pinBlink, LOW);
}

// ------------ ROS Subscribers ------------
ros::Subscriber<std_msgs::Bool> command_sub("transportsystem/command", &commandCallback);
ros::Subscriber<std_msgs::String> light_const_on_sub("light_indication/constant/on", &lightIndicationConstantCallback);
ros::Subscriber<std_msgs::String> light_blink_on_sub("light_indication/blink/on", &lightIndicationBlinkCallback);
ros::Subscriber<std_msgs::String> light_off_sub("light_indication/off", &lightIndicationOffCallback);

// ------------ Setup ------------
void setup() {
  // Transport
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorPWM, OUTPUT);

  // Signalisatie
  pinMode(BUZZER_CONS, OUTPUT);
  pinMode(RED_CONS, OUTPUT);
  pinMode(ORANGE_CONS, OUTPUT);
  pinMode(GREEN_CONS, OUTPUT);
  pinMode(BUZZER_BEEP, OUTPUT);
  pinMode(RED_BLINK, OUTPUT);
  pinMode(ORANGE_BLINK, OUTPUT);
  pinMode(GREEN_BLINK, OUTPUT);

  nh.initNode();

  nh.subscribe(command_sub);
  nh.subscribe(light_const_on_sub);
  nh.subscribe(light_blink_on_sub);
  nh.subscribe(light_off_sub);

  nh.advertise(status_pub);
  nh.advertise(object_pub);
  nh.advertise(begin_object_pub);
}

// ------------ Loop ------------
void loop() {
  nh.spinOnce();

  int beginRaw = analogRead(beginSensorPin);
  int endRaw   = analogRead(endSensorPin);

  begin_object_msg.data = isProductAanwezigBegin(beginRaw);
  object_msg.data       = isProductAanwezigEnd(endRaw);

  begin_object_pub.publish(&begin_object_msg);
  object_pub.publish(&object_msg);

  if (!transport_actief) {
    status_msg.data = "transport niet actief";
    status_pub.publish(&status_msg);
  }

  delay(200);
}
