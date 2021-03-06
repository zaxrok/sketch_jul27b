
/*  
 *  CodeStar Entry firmware
 *  (주)이산솔루션, www.isans.co.kr
 *  
 */
 
char remainData;
const int pinEcho = A1;
const int pinBuzzer = A3;
const int pinLeftIR = 47;
const int pinDirL = 2;
const int pinDirR = 3;
const int pinSpeedL = 5;
const int pinSpeedR = 6;
const int pinTrig = 13;
const int pinButton = 12;
const int tones[13] = {196,220,247,262,294,330,349,392,440,494,523,587,659};
int preDistance = 0;      // 초음파 -1 때문에

enum deviceNumber{
  FORWARD = 0,
  REVERSE = 1,
  MOTOR_L = 0,
  MOTOR_R = 1,
  BUZZERSTOP = 24,
};


void setup(){
  Serial.begin(9600);
  Serial.flush();
  //initPorts();
  delay(200);
  pinMode(pinBuzzer, OUTPUT);
  pinMode(pinSpeedL, OUTPUT);
  pinMode(pinDirL, OUTPUT);
  pinMode(pinSpeedR, OUTPUT);
  pinMode(pinDirR, OUTPUT);  
  pinMode(pinTrig, OUTPUT); // 출력용 핀으로 설정
  pinMode(pinEcho, INPUT);  // 입력용 핀으로 설정
  pinMode(pinButton, INPUT);
}

void initPorts () {
  for (int pinNumber = 0; pinNumber < 14; pinNumber++) {
    pinMode(pinNumber, OUTPUT);
    digitalWrite(pinNumber, LOW);
  }
}

void loop() {
  while (Serial.available()) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      updateDigitalPort(c);
    }
  } 
  delay(25);
  sendPinValues();
  delay(10);
}

void sendPinValues() {
  sendDigitalValue(4);  // 리모콘
  sendDigitalValue(6);  // 진동센서
  sendDigitalValue(12); // 버튼

  sendAnalogValue(0); // 왼쪽 벽감지
  sendAnalogValue(2); // 마이크
  sendAnalogValue(4); // 왼쪽 라인감지
  sendAnalogValue(5); // 오른쪽 라인감지
  sendAnalogValue(6); // 조도센서
  sendAnalogValue(7); // 오른쪽 벽감지

  // sonar
  int value = GetDistance();
  Serial.write(B11000000
               | ((1 & B111)<<3)
               | ((value>>7) & B111));
  Serial.write(value & B1111111);
}

void updateDigitalPort (char c) {
  // first data
  if (c>>7) {
    // is output
    if ((c>>6) & 1) {
      // is data end at this chunk
      if ((c>>5) & 1) {
        int port = (c >> 1) & B1111;
        if(port == 12 ) return ;
        
        setPortWritable(port);
        if (c & 1)
          digitalWrite(port, HIGH);
        else
          digitalWrite(port, LOW);
      }
      else {
        remainData = c;
      }
    } else {
      int port = (c >> 1) & B1111;
      setPortReadable(port);
    }
  } else {
    int port = (remainData >> 1) & B1111;
    // 10000000(128) + 01111111(127)
    int value = ((remainData & 1) << 7) + (c & B1111111); 
    if(port > 13){
      // 14 A0, 15 A1, 16 A2, 17 A3, 18 A4, 19 A5
      // A3: Buzzer(output)
      // A1: Sonar(input)
      // A5: Geomagnetic(input)
      if(port == 15){
        if(value == BUZZERSTOP){
          noTone(pinBuzzer);
        }else{
          tone(pinBuzzer, tones[value-1]); // 1 ~ 254
        }
      }else if(port == 14){
        //
        int id = value >> 7;
        int dir = (value >> 5) & B11;
        int tmp = value & B11111;
        if(value == 1){ // stop
          move(MOTOR_L, FORWARD, 0);
          move(MOTOR_R, FORWARD, 0);
        }else{
          if(id == 0 && dir == 0){ // forward
            move(MOTOR_L, FORWARD, tmp*30);
            move(MOTOR_R, FORWARD, tmp*30);
          }
          else if(id == 0 && dir == 1){ // backward
            move(MOTOR_L, REVERSE, tmp*30);
            move(MOTOR_R, REVERSE, tmp*30);
          }
          else if(id == 0 && dir == 2){ // left
            move(MOTOR_L, REVERSE, tmp*30);
            move(MOTOR_R, FORWARD, tmp*30);
          }
          else if(id == 0 && dir == 3){ // right
            move(MOTOR_L, FORWARD, tmp*30);
            move(MOTOR_R, REVERSE, tmp*30);
          }
          else if(id == 1 && dir == 0){ // left wheel
            move(MOTOR_L, FORWARD, tmp*30);
          }
          else if(id == 1 && dir == 1){ // right wheel
            move(MOTOR_R, FORWARD, tmp*30);
          }
          else if(id == 1 && dir == 2){ // left wheel+REVERSE
            move(MOTOR_L, REVERSE, tmp*30);
          }
          else if(id == 1 && dir == 3){ // right wheel+REVERSE
            move(MOTOR_R, REVERSE, tmp*30);
          }
        }        
      }
    }else{
      setPortWritable(port);
      analogWrite(port, value);
    }
    remainData = 0;
  }
}

void sendAnalogValue(int pinNumber) {
  int value = analogRead(pinNumber);
  Serial.write(B11000000
               | ((pinNumber & B111)<<3)
               | ((value>>7) & B111));
  Serial.write(value & B1111111);
}

void sendDigitalValue(int pinNumber) {
  if (digitalRead(pinNumber) == HIGH) {
    Serial.write(B10000000
                 | ((pinNumber & B1111)<<2)
                 | (B1));
  } else {
    Serial.write(B10000000
               | ((pinNumber & B1111)<<2));               
  }
}

void setPortReadable (int port) {
  if (isPortWritable(port)) {
    pinMode(port, INPUT);
  }
}

void setPortWritable (int port) {
  if (!isPortWritable(port)) {
    pinMode(port, OUTPUT);
  }
}

boolean isPortWritable (int port) {
  if (port > 7)
    return bitRead(DDRB, port - 8);
  else
    return bitRead(DDRD, port);
}

void move(int motor, int direction, int speed){
  boolean inPin1, inPin2;

  if(direction == FORWARD)
    inPin1 = HIGH;
  else // REVERSE
    inPin1 = LOW;

  if(motor == MOTOR_L){
    digitalWrite(pinDirL, inPin1);
    analogWrite(pinSpeedL, speed);
  } else { // MOTOR_R
    digitalWrite(pinDirR, inPin1);
    analogWrite(pinSpeedR, speed);
  }
}

int GetDistance(){  
  digitalWrite( pinTrig, LOW ); 
  delayMicroseconds( 2 );   
  digitalWrite( pinTrig, HIGH );
  delayMicroseconds( 10 );  
  digitalWrite( pinTrig, LOW ); 
  int  duration = pulseIn( pinEcho, HIGH );
  int  distance = (duration / 2) / 29.1;   
  if(distance < 0 ) distance = preDistance;
  preDistance = distance;
  return  distance;
}

