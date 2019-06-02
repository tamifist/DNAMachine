// контроллер элемента Пельтье

#include <MsTimer2.h>
#include <avr/wdt.h>

#define TR1_PIN A3
#define TR_NOMINAL 10000
#define koeffR 0.00335 // (1/To)
#define TR_B 3988
#define SERIES_R 10000

#define I_PIN A0
#define U_PIN A1

#define PWM_PIN 9

#define powerU  12.41     // напряжение источника питания
#define koeffU  0.05 // массштабный коэффициент напряжения,
                        // koeffU = 1.1 / R1 * (R1 + R2) / 1024 = 1.067/820*(820 + 12000)/1024 =0.01629 В/ед. АЦП 0.0152677
#define koeffI  0.06445 // массштабный коэффициент тока
                        // koeffI =  1,067(3.3) / R8 / 1024 = 0.02084 А/ед. АЦП

#define MEASURE_PERIOD 10  // время периода измерения (* 2 мс)

#define DEAD_TIME 8     // мертвое время ШИМ (* 62,5 нс)
#define MAX_PWM 255

#define koeffRegPwrInt 1 // интегральный коэффициент регулятора мощности

#define koeffRegTmpInt 1 // интегральный коэффициент регулятора температуры   
//#define koeffRegTmpInt 0.002 // интегральный коэффициент регулятора температуры
#define koeffRegTmpPr 0.5 // пропорциональный коэффициент регулятора температуры   
#define koeffRegTmpDif 50 // дифференцирующий коэффициент регулятора температуры  

#define MAX_POWER 60.   // максимальная выходная мощность контроллера

#define H_PIN1 2
#define H_PIN2 4
#define FAN_PIN 3

float measureTemp;
int sumR;
float averageR;

float measureU; // измеренное напряжение на нагрузке, В
float measureI; // измеренный ток потребления регулятора, А
float measureP; // измеренная мощность на нагрузке, Вт

unsigned int  pwm;

byte  interruptCount=0; // счетчик циклов прерываний
unsigned int  sumU, sumI; // переменные для суммирования кодов АЦП
unsigned int  averageU, averageI; // средние значения кодов АЦП
boolean flagReady;  // признак готовности данных измерения
byte  cycle20mcCount; // счетчик циклов 20 мс
byte  cycle1secCount; // счетчик циклов 1 сек
boolean denFlag;

float setPower; // заданная мощность
float regPwrInt=0; // интегральное звено регулятора мощности

float maxSetPower=MAX_POWER; // максимальная заданная мощность
float regTmpInt=0; // интегральное звено регулятора температуры
float regTmpPr; // пропорциональное звено регулятора температуры
float regTmpDif; // дифференциальное звено регулятора температуры
float regTmpErr; // ошибка рассогласования температуры
float regTmpErrPrev; // предыдущая ошибка рассогласования температуры

float setTempEngine; // заданная температура в камере

boolean isHeating;

void setup() 
{
 // установка ШИМ 8 разрядов, 62,5 кГц
  TCCR1A = TCCR1A & 0xe0 | 1;
  TCCR1B = TCCR1B & 0xe0 | 0x09; 

  Serial.begin(19200);  // инициализируем последовательный порт, скорость 19200
  //analogReference(INTERNAL); // опорное напряжение 1,1 В
  analogReference(EXTERNAL);
  MsTimer2::set(2, timerInterupt); // прерывания по таймеру с периодом 2 мс 
  MsTimer2::start();              // разрешение прерывания  
  wdt_enable(WDTO_15MS); // разрешение работы сторожевого таймера, тайм-аут 15 мс    
  setTempEngine= 40.;  // временно заданная температура
  //setPower = 15;

  pinMode(H_PIN1, OUTPUT);
  pinMode(H_PIN2, OUTPUT);
  
  pinMode(FAN_PIN, OUTPUT);
}

void loop() 
{  
  if (flagReady == true) 
  {
    flagReady= false; 

    measureTemp = getTemp(averageR);

    if(setTempEngine - measureTemp < 1 && denFlag == false)
    {
      Serial.print("DEN=");
      Serial.print(cycle1secCount);
      Serial.println(" ");

      denFlag = true;
      cycle1secCount = 0;
    }

regTmpErr = setTempEngine - measureTemp;  // вычисление ошибки рассогласования
//    if(measureTemp < setTempEngine)
//    {
//      setHeating();
//      regTmpErr = setTempEngine - measureTemp;  // вычисление ошибки рассогласования
//    }
//    else
//    {
//      setCooling();
//      regTmpErr = measureTemp - setTempEngine;  // вычисление ошибки рассогласования
//    }
    
    regTmpInt= regTmpInt + regTmpErr * koeffRegTmpInt; // интегральная часть  
    if (regTmpInt > maxSetPower)
    {
      regTmpInt = maxSetPower; // ограничение сверху
    }
    else if (regTmpInt < 0 )
    {
      regTmpInt = 0;                    // ограничение снизу
    }

    setPower= regTmpInt;

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

    if(isHeating)
    {
      analogWrite(FAN_PIN, 0);
    }
    else
    {
      analogWrite(FAN_PIN, pwm);
    }

    cycle20mcCount++;
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
      Serial.print(" t="); Serial.print(measureTemp, 2);  // температура в термоблоке
      Serial.println(" ");
    }
  }
}

//------------------------------ обработка прерывания по таймеру 2 мс
void  timerInterupt() {
  wdt_reset();  // сброс сторожевого таймера  
  interruptCount++; // счетчик циклов прерываний

  sumI += analogRead(I_PIN);  // суммирование выборок АЦП
  sumU += analogRead(U_PIN);  // суммирование выборок АЦП
  sumR += analogRead(TR1_PIN);  // суммирование выборок АЦП
  // проверка числа выборок усреднения
  if ( interruptCount >= MEASURE_PERIOD ) {
    interruptCount= 0;    
    averageU = sumU; // перегрузка среднего значения 
    averageI = sumI; // перегрузка среднего значения     
    averageR = sumR;
    sumU = 0;
    sumI = 0;
    sumR = 0;
    flagReady= true;  // признак результат измерений готов
  }  
}

float getTemp(float average)
{
  average /= MEASURE_PERIOD;

  // конвертируем значение в сопротивление
  
  average = 1023 / average - 1;
  
  average = SERIES_R / average;
  
  float steinhart;
  
  steinhart = average / TR_NOMINAL; // (R/Ro)
  
  steinhart = log(steinhart); // ln(R/Ro)
  
  steinhart /= TR_B; // 1/B * ln(R/Ro)
  
  steinhart += koeffR; // + (1/To)
  
  steinhart = 1.0 / steinhart; // инвертируем
  
  steinhart -= 273.15; // конвертируем в градусы по Цельсию

  return steinhart;
}

void setHeating()
{
  if(isHeating)
  {
    return;
  }

  analogWrite(PWM_PIN, 0);
  
  digitalWrite(H_PIN1, LOW);
  digitalWrite(H_PIN2, LOW);

  isHeating = true;
}

void setCooling()
{
  if(!isHeating)
  {
    return;
  }

  analogWrite(PWM_PIN, 0);
  
  digitalWrite(H_PIN1, HIGH);
  digitalWrite(H_PIN2, HIGH);

  isHeating = false;
}
