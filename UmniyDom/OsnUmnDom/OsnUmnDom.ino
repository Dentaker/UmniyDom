/* Списо событий для синхронизаци с часами
 *  1 - Опрос переключателей вентиляции и датчика CO
 *  2 - Подготовка цикла вентиляции
 *  3 - Цикл вентиляции
 *  4 - Завершение вентиляции
 *  5 - Алгоритм подогрев пола (ключает проверку датчика температуры пола)
 *  6 - Алгоритм отопление 
 */
 /*rabZone = A0; spalZone = A1; vitazVozd = A2; teplPol = A3;
 vhodVozd = A4; ulica = A5;poslRecup = A6;*/
//test 
 
 
#include <OneWire.h>
#include <iarduino_RTC.h>  //Универсальная библиотека для RTC DS1302, DS1307, DS3231
iarduino_RTC time(RTC_DS1307);

#define DEBUG 1 //режим отладки: 1- включен, 0 - выключен

#define vneshZasl 4 // Контакт внешней заслонки
#define vnutrZasl 5 // Контакт внутренней заслонки (рекуператор)

#define vikluch1 61 //Контакты выключателя вентиляции
#define vikluch2 62  

#define vent1 22 //Контакты реле, управляющих вентилятором
#define vent2 23  
#define numTT = 4; // количество событий
/* 
 *  перечень событий:
 *  1 - кнопки включения
 */
long timeT []=        {0, 0, 0, 0, 0, 0, 0}; // 0 - текущие секунды, 1 - last событие 1.
long timeInt []=      {0, 10, 0, 3600, 0, 0, 0}; // 0 - не используется, 1 - интервал событие 1.
long  timeWork[]=     {0, 1, 0, 600, 0, 0, 0}; // 0 - не используется, 1 - время работы событие 1.
boolean timeState []= {0, 0, 0, 0, 0, 0, 0}; // 0 - не используется, 1 - статус событие 1.

int rezVent = 0; // режим вентиляции 0-вентиляция отключена, 1-обычный режим, 2-усиленный.
float massDat[7] ={0}; //массив для хранения значений датчиков
int rezVent = 0; // режим вентиляции 0-вентиляция отключена, 1-обычный режим, 2-усиленный.
boolean Zima = 0; // 1 - зима, 0 - лето

void ledOn() //опрос выключателей. Событие 1
{
  digitalRead(2);          //отключение вентиляции
}

void ledOff() //опрос выключателей. Событие 1
{
  digitalRead(2);          //отключение вентиляции
}

void checkTime()  //Функция проверски реального времени и запуск необходимых алгоритмов
{
  time.gettime();
  timeT[0] = time.seconds + time.minutes*60 + time.hours*60; 
  for (int i=1; i< numTT;i++) // циклично обрабатываем все процессы
  { 
    if ( timeT[0]+timeWork[i] > 86400 ) // выключение выпадает на след день
    {
      
    }
    else if ( timeT[0]+timeT[i] > 86400 )  // след включение выпадет на след день
    {
      
    }
      
    if ( !timeState[i] && timeT[0] > timeT[i]+timeInt[i]-timeWork[i]) //Если событие не автивно и текущее время больше, чем время последнего запуска + интервал запуска - время работы
    {
      switch(i)
      {
        case 1:   // включение события 1
          oprosVikl();
          if (timeWork[]!=0)  timeState[i]=!timeState[i];
          timeT[i]=timeT[0];
        break;
        case 2:   // включение события 2
          // zapusk
          if (timeWork[]!=0)  timeState[i]=1;
          timeT[i]=timeT[0];
        break;
      }
      
    }
    else if ( timeState[i] && timeT[0] > timeT[i]+timeWork[i] || analog2==1) //Если событие активно и время больше, чем время запуска + время работы.
    {
      switch(i) 
      {
        case 1:   // выключение события 1
          timeState[i]=!timeState[i];
        break;
        case 2:   // выключение события 1
          // off
          timeState[i]=0;
          timeT[i]=timeT[0];
        break;
      }
    }
  }
}

void oprosVikl() //опрос выключателей. Событие 1
{
   if(digitalRead(2))
   {
    if(digitalRead(3)) rezVent = 2; //автоматический режим усиленный
    else rezVent = 1;               // автоматический режим 
   }
   else rezVent = 0;                //отключение вентиляции
}
	

int podgProvetr() //алгоритм подготовки проветривания
{
  pinMode (vneshZasl, OUTPUT);
  pinMode (vnutrZasl, OUTPUT);
  switch(rezVent) //Включение вентилятора согласно выбранному режиму
    {
     case 0:
     return 0;
     break;
     case 1:
     digitalWrite(vent1,0);
     digitalWrite(vent2,1);
     break;
     case 2:
     digitalWrite(vent2,0);// проверить, возиможнео оба
     digitalWrite(vent1,1);
     break;
    }
    
  if(massDat[5] < -5) {Zima = 1;} //Зима или нет?
  if(massDat[5] > 16) {Zima = 0;}

  if (Zima == 1)
  {
    digitalWrite(vnutrZasl, 1);
    oprosDat();
    if(massDat[6] < massDat[1]-1) //Опасный сука момент!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    {
      #ifdef Serial.println("Недостаточный прогрев рекуператора, продолжение вентиляции в режиме 100%"); #endif
      oprosDat();
      delay(500);
      return;
    }   
    analogWrite(vneshZasl, 255); //Выставляем заслонки в указанный в ТЗ начальный режим
    analogWrite(vnutrZasl, 130);
    delay(30000); //добавить прерывание
}

   

void pdProvetr()
{
  int polog1 = 255;
  int polog2 = 135;
  oprosDat();
  analogWrite(vneshZasl, polog1-massDat[ ]); //Выставляем заслонки в указанный в ТЗ начальный режим
  analogWrite(vnutrZasl, polog2);
  
}

void oprosDat()  //опрос всез датчиков
{
  for(int i=54;i<61;i++)
  {
    OneWire ds(i);
    byte data[2];
    ds.reset(); 
    ds.write(0xCC);
    ds.write(0x44);
    delay(100);
    ds.reset();
    ds.write(0xCC);
    ds.write(0xBE);
    data[0] = ds.read(); 
    data[1] = ds.read();
    int Temp = (data[1]<< 8)+data[0];
    Temp = Temp>>4;
    massDat[i-54]= Temp;
  }
  #ifdef DEBUG
    Serial.print("Значения датчиков:");
    for(int i=0;i<7;i++)
    {
      Serial.print("Датчик номер: ");
      Serial.print(i);
      Serial.print(" ");
      Serial.print(massDat[i-54]);
      Serial.println(" ");
    }
  #endif
}




void setup() {
  if(DEBUG) {Serial.begin(9600);} //активируем связь с ПК для отладки
 // pinMode (vikluch1, INPUT_PULLUP);
 // pinMode (vikluch2, INPUT_PULLUP);
  time.begin();
  //time.settime(0,51,21,27,10,15,2);  // 0  сек, 51 мин, 21 час, 27, октября, 2015 года, вторник
}

void loop() {
  if ( checkV() == 0) 	//проверка внешних выключаталей
  {
	Serial.println("Вентиляция отключена");
    stateV = 0;
  }
  else if ( checkV() == 2 ) 	//проверка внешних выключаталей
  {
    stateV = 1;
	rezimV = 1; //турбовентиляция
	Serial.println("Принудительная вентиляция с выключателя");
	// обновлять время последнего включения
  }
  else if ( checkCO2() )	//проверка датчика СО2
  {
	Serial.println("Принудительная вентиляция из-за СО2");
    stateV = 1;
	rezimV = 0;
  }
  else if ( checkTime() == 1)	//проверка на включение по времени
  {
	Serial.println("Вентиляция из-за времени");
    stateV = 1;
	rezimV = 0;
  }
  else if ( checkTime() == 2)	//проверка на выключение по времени
  {
	Serial.println("Вентиляция из-за времени");
    stateV = 0;
	rezimV = 0;
  }
  
  if ( stateV && oldStateV && !stateZapV)
  {
	  //Вентиляция работает
  }
  else if ( !stateV && oldStateV || stateZapV )
  {
	  //Вентиляция запускается 
  }
  else if ( stateV && !oldStateV )
  {
	  //Вентиляция выключается
  }
  else if ( stateV && !oldStateV )
  {
	  //Вентиляция выключена
  }
  oldStateV=stateV;
}

int oprosVikl() //опрос выключателей {
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
	return rezVent;
}

	Serial.println("Принудительная вентиляция с выключателя");
  else if ( checkCO2() )	//проверка датчика СО2
	Serial.println("Принудительная вентиляция из-за СО2");
  else if ( checkTime() )	//проверка датчика СО2
	Serial.println("Вентиляция из-за времени");

	
  stateV
  oldStateV
  prinud - статус принудительной вентиляции
  stateZapV
  */
