#include <OneWire.h>
#include <iarduino_RTC.h>  //Универсальная библиотека для RTC DS1302, DS1307, DS3231
#include <DallasTemperature.h>
#include "DHT.h"

#define ONE_WIRE_BUS 54
#define DEBUG 1 //режим отладки: 1- включен, 0 - выключен
#define vneshZasl 4 // Контакт внешней заслонки
#define vnutrZasl 5 // Контакт внутренней заслонки (рекуператор)
#define vikluch1 56 //Контакты выключателя вентиляции
#define vikluch2 57
#define vent1 22 //Контакты реле, управляющих вентилятором
#define vent2 23
#define DHTPIN 55 //Контакт датчика температуры и влажности
#define DHTTYPE DHT22 //Модель датчика температуры и влажности
#define normT 28 // Задание нормальной температуры воздуха поступающего в комнату для регуляции
#define kP 2     //Коэффициент пропорциональности
#define pinTeplPol 25 //контакт для теплого пола

DHT dht(DHTPIN, DHTTYPE); //Создание структуры датчика для температуры и влажности

iarduino_RTC time(RTC_DS1307); // Создание объекта для датчика температуры и влажности
boolean stateV=0, oldStateV=0, zima = 0; //статусные переменные
int rezVent=0; // режим вентиляции 0-вентиляция отключена, 1-обычный режим, 2-усиленный.
int stateVnesh = 0;  //Переменные для хранения положения заслонок
int stateVnutr  = 0;
int stateZapV = 0;
int tStartPol = 18; //Температура пола

float h = 0; //Переменная влажность
float t = 0; //Переменная температура датчика температуры и влажеости

long timeT = 0; //Текущее время в секундах

struct worktime //структура для описания события
{
  long lastWork; //последний запуск
  int timeWork; //время работы
  int timeNoWork; //интервал
  boolean stateWork; //статус
};

worktime Vent = {0, 600, 30000, 0}; //Событие вентиляция

//Инициализация переменных и структур для работы с тевпературными датчиками
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

DeviceAddress rabZone = {
  0x28, 0xFF, 0x6C, 0x37, 0xA1, 0x16, 0x04, 0xCC
};  // адрес датчика DS18B20 280054B604000092
DeviceAddress spalZone = {
  0x28, 0xFF, 0x9A, 0xCE, 0xA1, 0x16, 0x03, 0x97
};
DeviceAddress teplPol = {
  0x28, 0xFF, 0x66, 0x7E, 0xA1, 0x16, 0x05, 0x5C
};
DeviceAddress podVozd = {
  0x28, 0xFF, 0x11, 0x37, 0xA1, 0x16, 0x04, 0x4D
};
DeviceAddress ulica = {
  0x28, 0xFF, 0x49, 0xD4, 0xA1, 0x16, 0x03, 0xA4
};
DeviceAddress poslRecup = {
  0x28, 0xFF, 0xAD, 0xCB, 0xA1, 0x16, 0x03, 0xC3
};
//Конец инициализации темературных датчиков

int checkTime(){  //Функция проверски реального времени и запуск необходимых алгоритмов 
  time.gettime();
  //timeT = time.seconds + time.minutes * 60 + time.hours * 60 * 60;
  timeT = millis()/1000;
  
  Serial.print(timeT);
  if ( (Vent.lastWork + Vent.lastWork) > 86400 || Vent.lastWork + Vent.timeNoWork > 86400 ) // выключение или выключение выпадает на след день
  {
    timeT += 86400;
  }
  else
  {
    //Если событие не активно и текущее время больше, чем время последнего запуска + интервал запуска
    if ( !Vent.stateWork && timeT > Vent.lastWork + Vent.timeNoWork)
    {
    //включение
      Vent.stateWork = 1;
      Vent.lastWork = timeT;
    }
    //Если событие активно и время больше, чем время запуска + время работы.
    else if ( Vent.stateWork && timeT > Vent.lastWork + Vent.timeWork )
    {//выключение
      Vent.stateWork = 0;
      Vent.lastWork = timeT;
    }
  }
  return Vent.stateWork;
}

void oprosVikl() {//опрос выключателей

  if (!digitalRead(vikluch1))
  {
    if (!digitalRead(vikluch2))
    {
      rezVent = 2; //ручной режим усиленный
    }
    else rezVent = 1;               // автоматический режим
  }
  else rezVent = 0;                //отключение вентиляции

#ifdef DEBUG
  Serial.print("Rezim ventilacii: ");
  Serial.println(rezVent);
#endif
}

int oprosDat(DeviceAddress deviceAddress) { //опрос указанного датчика

  sensors.requestTemperatures();
  int tempC = sensors.getTempC(deviceAddress);
  return tempC;
}

long getTime() {  //Получаем текущее время от внешнего модуля
  time.gettime();
  return long(time.seconds) + long(time.minutes) * 60 + long(time.hours) * 60 * 60;
}

void podgProvetr(){ 	//Подготовка проветривания
#ifdef DEBUG
  Serial.println("Nacata podgotovka provetrivaniya");
#endif
  switch (rezVent)  { //Включение вентилятора согласно выбранному режиму
    case 1:
      digitalWrite(vent1, 0);
      digitalWrite(vent2, 1);
      break;
    case 2:
      digitalWrite(vent2, 0); // проверить, возиможнео оба
      digitalWrite(vent1, 1);
      break;
  }
  if (oprosDat(ulica) < -5) {   //Проверка на зиму
    zima = 1;
  }
  else if (oprosDat(ulica) > 16) {  //Проверка на лето
    zima = 0;
  }
    digitalWrite(vnutrZasl, 1);
    if (oprosDat(poslRecup) < oprosDat(spalZone) - 1) //Ждем прогрева рекуператора
    {
#ifdef DEBUG
      Serial.println("Недостаточный прогрев рекуператора, продолжение вентиляции в режиме 100%");
#endif
	stateZapV = 1;
    }
	else
	{
#ifdef DEBUG
      Serial.println("Прогрев завершен");
#endif	
	stateZapV = 0;	
	}
  if (zima)  {       //начальное положение заслонок в зависимости от времени года
	stateVnesh = 255;
	stateVnutr = 135;
	}
  else {
	stateVnesh = 255;
	stateVnutr = 0;
	}
} 

void pdProvetr(){    //регулирование подаваемого воздуха в П режиме

    int error = oprosDat(podVozd) - normT; // считаем отклонение
    if (abs(error) > 2) //Проверка на гистерезис
    {
		stateVnesh = stateVnesh + error * kP;
		stateVnutr = stateVnutr - error * kP;
		constrain(stateVnesh, 0,255);
		constrain(stateVnesh, 0,255);
		analogWrite(vneshZasl, stateVnesh);
		analogWrite(vnutrZasl, stateVnutr);
    }
#ifdef DEBUG
      Serial.print ("PodVozd: ");
      Serial.println (oprosDat(podVozd));
      Serial.print ("error: ");
      Serial.println (error);
      Serial.print ("stateVnesh: ");
      Serial.println (stateVnesh);
      Serial.print ("stateVnutr: ");
      Serial.println (stateVnutr);
#endif
	delay(5000);
}

void stopVent  (){

	analogWrite(vneshZasl, 0);
	analogWrite(vnutrZasl, 0);
	digitalWrite(vent1, 1);
    digitalWrite(vent2, 1);
}

void vitaznoiVozduh(){ //Опрашивает датчик вытяжного воздуха, сохраняет данные о температуре и влажности

  h = dht.readHumidity();
  t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
#ifdef DEBUG
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.println(" *C ");
#endif
}

void uprPol() { // управление полом ДОДЕЛАТЬ
  if(zima)     
  { 
    if(oprosDat(teplPol) < tStartPol) //если пол остыл ниже 18 градусов
    {     
		digitalWrite(pinTeplPol, 0);
		#ifdef DEBUG
		Serial.println("Теплый пол включен");  
		#endif         
    }  
    else if (oprosDat(teplPol) < tStartPol+2) //Если пол нагрелся до 20
	{
        digitalWrite(pinTeplPol, 1);
		#ifdef DEBUG
		Serial.println("Теплый пол выключен");  
		#endif      
    }
  }
}


void setup() {
#ifdef DEBUG
  Serial.begin(9600); //активируем связь с ПК для отладки
#endif
  pinMode (vikluch1, INPUT_PULLUP);
  pinMode (vikluch2, INPUT_PULLUP);
  pinMode (vneshZasl, OUTPUT);
  pinMode (vnutrZasl, OUTPUT);
  pinMode (vent1, OUTPUT);
  pinMode (vent2, OUTPUT);
  pinMode(pinTeplPol , OUTPUT);
  digitalWrite(vent1, 1);
  digitalWrite(vent2, 1);
  analogWrite(vneshZasl, 0);
  analogWrite(vnutrZasl, 0);
  time.begin(); //начало работы с часами
  dht.begin(); // начало работы с датчиком температуры и влажности
  
}

void loop() {
#ifdef DEBUG
	Serial.print("RabZone:  ");
    Serial.println ( oprosDat(rabZone));
    Serial.print("spalZone:  ");
    Serial.println ( oprosDat(spalZone));
    Serial.print("teplPol:  ");
    Serial.println ( oprosDat(teplPol));
    Serial.print("podVozd:  ");
    Serial.println ( oprosDat(podVozd));
    Serial.print("ulica:  ");
    Serial.println ( oprosDat(ulica));
    Serial.print("poslRecup:  ");
    Serial.println ( oprosDat(poslRecup));
    Serial.print("  ");
#endif
stateV = checkTime();
  if ( stateV && oldStateV && !stateZapV)
  {
    pdProvetr();//Вентиляция работает, регулируем
    Serial.print("\tWork");
  }
  else if ( stateV && !oldStateV  || stateZapV )
  {
    podgProvetr();//Вентиляция запускается, процедура обновляющая stateZapV
    Serial.print("\tZapusk");
  }
  else if ( !stateV && oldStateV )
  {
    stopVent();//Вентиляция выключается
    Serial.print("\tStop");
  }
  else if ( !stateV && !oldStateV )
  {
    Serial.print("\t----");
    //Вентиляция выключена
  }
  oldStateV=stateV;
}
