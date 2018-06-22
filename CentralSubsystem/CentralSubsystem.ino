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
  AUTOPILOT_CMD = 6,
  SPEED_CMD = 7
};
enum Maneuver {
  LEFT_MVR,
  RIGHT_MVR
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

float distL, lastDistC, distC, distR, distSideR, distSideL, distB;
unsigned int balancer = 11; //количество циклов прямой езды
Maneuver lastManeuver; //последний манёвр
int direction = 0; //текущий курс относительно начального. Стремится к нулю
volatile int inc = 0;
volatile int path = 0;
int currentSpeed = 3;

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
  lastDistC=distC;
  
  distL = sonarL.ping_cm();
  if(distL==0) distL=150;
  distC = sonarC.ping_cm();
  if(distC==0) distC=150;
  distR = sonarR.ping_cm();
  if(distR==0) distR=150;
  distB = sonarB.ping_cm();
  if(distB==0) distB=150;
  distSideL = sonarSL.ping_cm();
  if(distSideL==0) distSideL=150;
  distSideR = sonarSR.ping_cm();
  if(distSideR==0) distSideR=150;
}

//Можно ли совершить определённый манёвр. Во избежание неконтролируемых поворотов, нельзя менять направление поворота до прямолинейной траектории
bool canPerfomManeuver(Maneuver maneuver) {
  if(maneuver==LEFT_MVR){
    return lastManeuver==LEFT_MVR || balancer>10;
  }
  else{
    return lastManeuver==RIGHT_MVR || balancer>10;
  }
}

//Движение вперёд с поправкой скорости при приближении к препятствию
void straight() {
  //Средняя скорость
  int avgSpeed = 95 - 700/distC;
  if(distC<20 && lastDistC>distC) {
    avgSpeed-=min((lastDistC-distC)*5,30);
  }

  /*int distSide = min(distSideL,distSideR);
  if(distSide<6){ //Разные скорости по бокам, если боковое препятствие близко
    int diff = 50/distSide;
    if(distSideL<distSideR) {
      dvig(avgSpeed+diff,avgSpeed-diff,avgSpeed+diff,avgSpeed-diff);
    }
    else {
      dvig(avgSpeed-diff,avgSpeed+diff,avgSpeed-diff,avgSpeed+diff);
    }
  }
  else{*/
    dvig(avgSpeed,avgSpeed,avgSpeed,avgSpeed);
  //}
  balancer++;
}

//Поворот на определённое количество градусов. Положительное направление - по часовой стрелке
void turn(int degree){
  if(degree<0) {
    dvig(-150,150,-150,150);
    lastManeuver=LEFT_MVR;
    direction--;
  }
  else {
    dvig(150,-150,150,-150);
    lastManeuver=RIGHT_MVR;
    direction++;
  }
  balancer=0;
  delay(4.2*abs(degree));
} 

void m_speed (){
  if(distL > 8 && distC > 20 && distR > 8) { //Путь свободен
    if(distSideL > 25 && direction>0 && canPerfomManeuver(LEFT_MVR)){ //Возвращение направления при перекосе вправо
      turn(-10);
    }
    else if(distSideR > 25 && direction<0 && canPerfomManeuver(RIGHT_MVR)) { //Возвращение направления при перекосе влево
      turn(10);
    }
    else { //Движение вперёд, медленно приближаясь к препятствию
      straight();
    }
  }
  else if(distSideL > 10 || distSideR > 10) { //Поворот
    uint8_t left = ((uint8_t)(canPerfomManeuver(LEFT_MVR) && distSideL > 10) << 2) + ((uint8_t)(direction > 3) << 1)  + (uint8_t)(distSideL>=distSideR);
    uint8_t right = ((uint8_t)(canPerfomManeuver(RIGHT_MVR) && distSideR > 10) << 2) + ((uint8_t)(direction < -3) << 1)  + (uint8_t)(distSideL<distSideR);
    Serial.print(left);
    Serial.println(right);
    if(left>3 && left >= right){
      turn(-10);
    }
    else if (right>3){
      turn(10);
    }
    else {
      dvig(0,0,0,0);
    }
  }
  else { //Остановка
    dvig(0,0,0,0);
  }
}


//Попробовать применить команду. Возвращает успешность операции
bool tryApplyManualCommand(Command command) {
  bool isSuccess = false;
  switch(command) {
    case FORWARD_CMD:
    if(distC>7&&distL>10&&distR>10) {
      int speed = 70 + currentSpeed*30 - 1000/distC;
      if(distC<20 && lastDistC>distC) {
        speed-=min((lastDistC-distC)*5,30);
      }
      dvig(speed,speed,speed,speed);
      isSuccess = true;
    }
    break;
    
    case BACKWARD_CMD:
    if(distB>10) {
      int speed = -70 - currentSpeed*30 + 1000/distB;
      dvig(speed,speed,speed,speed);
      isSuccess = true;
    }
    break;
    
    case LEFT_TURN_CMD:
    if(distC>5&&distL>8) {
      int speed = 100 + currentSpeed*24;
      dvig(-speed,speed,-speed,speed);
      isSuccess = true;
    }
    break;

    case RIGHT_TURN_CMD:
    if(distC>5&&distR>8) {
      int speed = 100 + currentSpeed*24;
      dvig(speed,-speed,speed,-speed);
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
  ultrazvuk(); //для вычисление последнего расстояния по центральной оси
  //timer.Every(2000,sendMotionState);
  //attachInterrupt(0,incFunc,FALLING);
}

void loop() {
  //timer.Update(); //обновление таймера
  
  //Проверка на новые команды
  if(Serial1.available()>0) {
    uint8_t data = Serial1.read();
    data-=48;
    if(data < 8) {
      currentCommand = (Command)data;
    }
  }
  else if(Serial2.available()>0) {
    uint8_t data = Serial2.read();
    data-=48;
    if(data < 8) {
      currentCommand = (Command)data;
    }
    if(currentCommand==SPEED_CMD) {
      data = Serial2.read();
      data-=48;
      if(data < 6) {
        currentSpeed = data;
      }
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
      balancer = 11;
      direction = 0;
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
