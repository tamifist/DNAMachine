// контроллер элемента Пельтье

#include <MsTimer2.h>
#include <avr/wdt.h>
#include <OneWire.h>

#define PWM_PIN 9

#define powerU  12.41     // напряжение источника питания
#define koeffU  0.01689 // массштабный коэффициент напряжения,
                        // koeffU = 1.1 / R1 * (R1 + R2) / 1024 = 1.067/820*(820 + 12000)/1024 =0.01629 В/ед. АЦП 
#define koeffI  0.02148 // массштабный коэффициент тока
                        // koeffI =  1,067 / R8 / 1024 = 0.02084 А/ед. АЦП

#define MEASURE_PERIOD 10  // время периода измерения (* 2 мс)

#define DEAD_TIME 8     // мертвое время ШИМ (* 62,5 нс)
#define MAX_PWM 255

#define koeffRegPwrInt 1 // интегральный коэффициент регулятора мощности

#define koeffRegTmpInt 1 // интегральный коэффициент регулятора температуры   
//#define koeffRegTmpInt 0.002 // интегральный коэффициент регулятора температуры
#define koeffRegTmpPr 0.5 // пропорциональный коэффициент регулятора температуры   
#define koeffRegTmpDif 50 // дифференцирующий коэффициент регулятора температуры  

#define MAX_POWER 60.   // максимальная выходная мощность контроллера

unsigned int  pwm;

float measureU; // измеренное напряжение на нагрузке, В
float measureI; // измеренный ток потребления регулятора, А
float measureP; // измеренная мощность на нагрузке, Вт

byte  interruptCount=0; // счетчик циклов прерываний
unsigned int  sumU, sumI; // переменные для суммирования кодов АЦП
unsigned int  averageU, averageI; // средние значения кодов АЦП
boolean flagReady;  // признак готовности данных измерения
byte  cycle20mcCount; // счетчик циклов 20 мс
byte  cycle1secCount; // счетчик циклов 1 сек
boolean denFlag;

float measureTempRad;  // измеренная температура в камере (< -200 - ошибка)
float measureTempEngine;  // измеренная температура радиатора (< -200 - ошибка)

OneWire sensTempRad (16);  // датчик температуры в камере подключен к выводу 16 (A2)
OneWire sensTempEngine (17);  // датчик температуры радиатора подключен к выводу 17 (A3)
byte bufData[9];  // буфер данных

float setPower; // заданная мощность
float regPwrInt=0; // интегральное звено регулятора мощности

float maxSetPower=MAX_POWER; // максимальная заданная мощность
float regTmpInt=0; // интегральное звено регулятора температуры
float regTmpPr; // пропорциональное звено регулятора температуры
float regTmpDif; // дифференциальное звено регулятора температуры
float regTmpErr; // ошибка рассогласования температуры
float regTmpErrPrev; // предыдущая ошибка рассогласования температуры

float setTempEngine; // заданная температура в камере

void setup() {
 // установка ШИМ 8 разрядов, 62,5 кГц
  TCCR1A = TCCR1A & 0xe0 | 1;
  TCCR1B = TCCR1B & 0xe0 | 0x09; 

  Serial.begin(19200);  // инициализируем последовательный порт, скорость 19200
  analogReference(INTERNAL); // опорное напряжение 1,1 В
  MsTimer2::set(2, timerInterupt); // прерывания по таймеру с периодом 2 мс 
  MsTimer2::start();              // разрешение прерывания  
  wdt_enable(WDTO_15MS); // разрешение работы сторожевого таймера, тайм-аут 15 мс    
  setTempEngine= 50.;  // временно заданная температура
  //setPower = 15;
}

void loop() {
  if (flagReady == true) 
  {
    flagReady= false; 

    measureU = powerU - (float)(averageU / MEASURE_PERIOD) * koeffU;
    if (measureU < 0)
    {
      measureU=0;
    }

    measureI = (float)(averageI / MEASURE_PERIOD) * koeffI;
    
    measureP = powerU * measureI;

    if ( setPower >= 0)
    {
      regPwrInt = regPwrInt + (setPower - measureP) * koeffRegPwrInt; 
      if (regPwrInt < 0)
      {
        regPwrInt=0;
      }
      else if (regPwrInt > MAX_PWM)
      {
        regPwrInt=MAX_PWM;
      }
      
      pwm = (unsigned int)regPwrInt;  // перевод в ШИМ
      if (pwm < DEAD_TIME)
      { 
        pwm=0;
      }
      else if (pwm > (MAX_PWM - DEAD_TIME))
      { 
        pwm = MAX_PWM;    
      }
    }
    else 
    {            
      // выключение
      regPwrInt = 0;
      pwm = 0;    
    }
    
    analogWrite(PWM_PIN, pwm); // ШИМ  
    
    //--------------- выполнение распределенных операций в цикле 1 сек ---------------
    cycle20mcCount++;   // счетчик циклов 20 мс
    if (cycle20mcCount >= 50) 
    { 
      cycle20mcCount = 0;
      cycle1secCount++;

      Serial.print("U="); Serial.print(measureU, 2);  // напряжение
      Serial.print(" I="); Serial.print(measureI, 2);  // ток
      Serial.print(" P="); Serial.print(measureP, 2);  // мощность
      Serial.print(" W="); Serial.print(setPower, 2);  // заданная мощность
      Serial.print(" p="); Serial.print(pwm);  // ШИМ
      //Serial.print(" t="); Serial.print(measureTempRad, 2);  // температура радиатора
      Serial.print(" t="); Serial.print(measureTempEngine, 2);  // температура в термоблоке
      Serial.println(" ");
    }

    if (cycle20mcCount == 0) 
    {
//      sensTempRad.reset();  // сброс шины 1-Wire
//      sensTempRad.write(0xCC, 1); // пропуск ROM
//      sensTempRad.write(0x44, 1); // инициализация 

      sensTempEngine.reset();  // сброс шины 1-Wire
      sensTempEngine.write(0xCC, 1); // пропуск ROM
      sensTempEngine.write(0x44, 1); // инициализация измерения
    }

//    if (cycle20mcCount == 45) {
//      //--------------------------- интервал 45, чтение датчика температуры термоблока
//      sensTempRad.reset();  // сброс шины 1-Wire
//      sensTempRad.write(0xCC, 1); // пропуск ROM  
//      sensTempRad.write(0xBE, 1); // команда чтения памяти датчика  
//      sensTempRad.read_bytes(bufData, 9);  // чтение памяти датчика, 9 байтов
//
//      if (OneWire::crc8(bufData, 8) == bufData[8] ) {  // проверка CRC
//        // правильно
//        measureTempRad= (float)((int)bufData[0] | (((int)bufData[1]) << 8)) * 0.0625 + 0.03125;   
//      }
//      //else measureTempRad= -300.;   // ошибка измерения
//    }

    if (cycle20mcCount % 5) {
      //--------------------------- интервал 46, чтение датчика температуры радиатора
      sensTempEngine.reset();  // сброс шины 1-Wire
      sensTempEngine.write(0xCC, 1); // пропуск ROM  
      sensTempEngine.write(0xBE, 1); // команда чтения памяти датчика  
      sensTempEngine.read_bytes(bufData, 9);  // чтение памяти датчика, 9 байтов

      if (OneWire::crc8(bufData, 8) == bufData[8] ) 
      {
        //measureTempEngine= (float)((int)bufData[0] | (((int)bufData[1]) << 8)) * 0.5 + 0.25;  
        measureTempEngine= (float)((int)bufData[0] | (((int)bufData[1]) << 8)) * 0.0625 + 0.03125; 

        regTmpErr= setTempEngine - measureTempEngine;  // вычисление ошибки рассогласования

        regTmpInt= regTmpInt + regTmpErr * koeffRegTmpInt; // интегральная часть  
        if (regTmpInt > maxSetPower)
        {
          regTmpInt= maxSetPower; // ограничение сверху
        }
        else if (regTmpInt < 0 )
        {
          regTmpInt= 0;                    // ограничение снизу
        }
  
        setPower= regTmpInt;

//        sensTempRad.reset();  // сброс шины 1-Wire
//        sensTempRad.write(0xCC, 1); // пропуск ROM
//        sensTempRad.write(0x44, 1); // инициализация 
  
        sensTempEngine.reset();  // сброс шины 1-Wire
        sensTempEngine.write(0xCC, 1); // пропуск ROM
        sensTempEngine.write(0x44, 1); // инициализация измерения
        
        if(setTempEngine - measureTempEngine < 1 && denFlag == false)
        {
          Serial.print("DEN=");
          Serial.print(cycle1secCount);
          Serial.println(" ");
          cycle1secCount = 0;
          denFlag = true;
        }
      }
    }
  }
}

//------------------------------ обработка прерывания по таймеру 2 мс
void  timerInterupt() {
  wdt_reset();  // сброс сторожевого таймера  
  interruptCount++; // счетчик циклов прерываний

  // измерение напряжения и тока
  sumU += analogRead(A1);  // суммирование выборок АЦП
  sumI += analogRead(A0);  // суммирование выборок АЦП
  // проверка числа выборок усреднения
  if ( interruptCount >= MEASURE_PERIOD ) {
    interruptCount= 0;    
    averageU = sumU; // перегрузка среднего значения 
    averageI = sumI; // перегрузка среднего значения     
    sumU= 0;
    sumI= 0;
    flagReady= true;  // признак результат измерений готов
  }  
}
