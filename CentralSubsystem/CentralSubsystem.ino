#include <NewPing.h>
#include <AFMotor.h>
//#include <EveryTimer.h>

enum Command {
  NONE_CMD = 0,
  STOP_CMD = 1,
  BACKWARD_CMD = 2,
  FORWARD_CMD = 3,
  LEFT_TURN_CMD = 4,
  RIGHT_TURN_CMD = 5,
  AUTOPILOT_CMD = 6
};

Command currentCommand; //команда, которая выполняется в данный момент
bool isAutopilot = false; //текущий режим: автопилот/ручное управление

//EveryTimer timer;

NewPing sonarL(37,39, 150);
NewPing sonarC(41,43, 150);
NewPing sonarR(40,42, 150);
NewPing sonarB(30,32, 150);
NewPing sonarSL(31,33, 150);
NewPing sonarSR(50,52, 150);

AF_DCMotor m_FL(4, MOTOR34_64KHZ);
AF_DCMotor m_FR(3, MOTOR34_64KHZ);
AF_DCMotor m_BL(1, MOTOR12_64KHZ);
AF_DCMotor m_BR(2, MOTOR12_64KHZ);

float distL, distC, distR, distB, distSL, distSR;
unsigned int sFL, sFR, sBL, sBR;
volatile int inc = 0;
volatile int path = 0;

void incFunc(){
  inc++;
}

void dvig (int sFL, int sFR,int sBL,int sBR) { 
  if (sFL > 0){ 
    m_FL.run(FORWARD); 
  } 
  else { 
    m_FL.run(BACKWARD);
    sFL=-sFL;
  }
  if (sFR > 0){ 
    m_FR.run(BACKWARD); 
  } 
  else { 
    m_FR.run(FORWARD); 
    sFR=-sFR;
  } 
  if (sBL > 0){ 
    m_BL.run(BACKWARD); 
  } 
  else { 
    m_BL.run(FORWARD);
    sBL=-sBL;
  } 
  if (sBR > 0){ 
    m_BR.run(FORWARD); 
  } 
  else { 
    m_BR.run(BACKWARD);
    sBR=-sBR;
  }
  m_FL.setSpeed(sFL); 
  m_FR.setSpeed(sFR); 
  m_BL.setSpeed(sBL); 
  m_BR.setSpeed(sBR);
}
void ultrazvuk(){
  distL = sonarL.ping_cm();
  if(distL==0) distL=150;
  distC = sonarC.ping_cm();
  if(distC==0) distC=150;
  distR = sonarR.ping_cm();
  if(distR==0) distR=150;
  distB = sonarB.ping_cm();
  if(distB==0) distB=150;
  distSL = sonarB.ping_cm();
  if(distSL==0) distSL=150;
  distSR = sonarSR.ping_cm();
  if(distSR==0) distSR=150;
}

void m_speed(){
  dvig (150,150,150,150);
  if  (distC <= 20){                                  // объезд препятствия и движение вдоль преграды
          dvig(0,0,0,0);
          delay(1000);
            if (distL >= 8 && distSL >= 15 && distSR >= 10) {                                 // поворот на 90гр влево
                  turnLEFT();
                  dvig(200,200,200,200);
                  delay(700);
                  turnRIGHT();
                  delay(700);}}

if (distC < 10 || distL < 10 || distR < 10){ // экстремальная остановка 
 sFL = 0; 
 sFR = 0; 
 sBL = 0; 
 sBR = 0;} 
 
dvig(sFL, sFR, sBL, sBR);
}
void turnLEFT(){ // поворот на 90 гр влево 
 sFR = 220; 
 sFL = -220; 
 sBR = 220; 
 sBL = -220; 
 dvig(sFR,sFL,sBL,sBR);
 delay(500); 
 
} 
void turnRIGHT(){ // поворот на 90 гр вправо 
 
 sFR = -220; 
 sFL = 220; 
 sBR = -220; 
 sBL = 220; 
 dvig(sFR,sFL,sBL,sBR);
 delay(500); 
}


//Попробовать применить команду. Возвращает успешность операции
bool tryApplyManualCommand(Command command) {
  bool isSuccess = false;
  switch(command) {
    case FORWARD_CMD:
    if(distC>7&&distL>10&&distR>10) {
      int speed = 120 - 1000/distC;
      dvig(speed,speed,speed,speed);
      isSuccess = true;
    }
    break;
    
    case BACKWARD_CMD:
    if(distB>10) {
      int speed = -120 + 1000/distB;
      dvig(speed,speed,speed,speed);
      isSuccess = true;
    }
    break;
    
    case LEFT_TURN_CMD:
    if(distC>5&&distL>8) {
      dvig(-220,220,-220,220);
      isSuccess = true;
    }
    break;

    case RIGHT_TURN_CMD:
    if(distC>5&&distR>8) {
      dvig(220,-220,220,-220);
      isSuccess = true;
    }
    break;

    default:
    //Команда "стоп" выполняется единожды, после остановки робот находится в ожидании
    dvig(0,0,0,0);
    currentCommand=NONE_CMD;
    isSuccess = true;
    break;
  }
  return isSuccess;
}

//Отпрвка состояния движения
void sendMotionState(){
  int avgSpeed = inc/40;
  path+=inc/20;
  inc=0;
  Serial2.print(String(avgSpeed) + " " + String(path));
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); //EasyVR
  Serial2.begin(115200); //ESP8266
  currentCommand = STOP_CMD;
  //timer.Every(2000,sendMotionState);
  //attachInterrupt(0,incFunc,FALLING);
}

void loop() {
  //timer.Update(); //обновление таймера
  
  //Проверка на новые команды
  if(Serial1.available()>0) {
    uint8_t data = Serial1.read();
    data-=48;
    if(data < 7) {
      currentCommand = (Command)data;
    }
  }
  else if(Serial2.available()>0) {
    uint8_t data = Serial2.read();
    data-=48;
    if(data < 7) {
      currentCommand = (Command)data;
    }
  }
  
  //Команда "автопилот" выполняется единожды, её задача - переключение режима
  if(currentCommand == AUTOPILOT_CMD){
    if(isAutopilot) {
      isAutopilot = false;
      currentCommand = STOP_CMD;
    }
    else{
      isAutopilot = true;
      currentCommand = NONE_CMD;
    }
  }

  if(currentCommand!=NONE_CMD) {
    ultrazvuk();
    if(!tryApplyManualCommand(currentCommand)){
      //Если путь по направлению загорожен, то остановить робота
      tryApplyManualCommand(STOP_CMD);
    }
    //При поступлении команды движения, режим автопилот сбрасывается
    isAutopilot = false;
  }
  else if(isAutopilot) {
    ultrazvuk();
    m_speed();
  }
  delay(50);
}
