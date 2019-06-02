#include <MsTimer2.h>
#include <avr/wdt.h>

#define powerU  12.41     // напряжение источника питания
#define koeffU  0.01689 // массштабный коэффициент напряжения,
                        // koeffU = 1.1 / R1 * (R1 + R2) / 1024 = 1.067/820*(820 + 12000)/1024 =0.01629 В/ед. АЦП 
#define koeffI  0.02148 // массштабный коэффициент тока
                        // koeffI =  1,067 / R8 / 1024 = 0.02084 А/ед. АЦП

#define MEASURE_PERIOD 10  // время периода измерения (* 2 мс)

float measureU; // измеренное напряжение на нагрузке, В
float measureI; // измеренный ток потребления регулятора, А
float measureP; // измеренная мощность на нагрузке, Вт

byte  interruptCount=0; // счетчик циклов прерываний
unsigned int  sumU, sumI; // переменные для суммирования кодов АЦП
unsigned int  averageU, averageI; // средние значения кодов АЦП
boolean flagReady;  // признак готовности данных измерения
byte  cycle20mcCount; // счетчик циклов 20 мс

float setPower; // заданная мощность
float regPwrInt=0; // интегральное звено регулятора мощности
#define koeffRegPwrInt 0.1 // интегральный коэффициент регулятора мощности
#define DEAD_TIME 8     // мертвое время ШИМ (* 62,5 нс)
#define MAX_PWM 255     

void setup() {
 // установка ШИМ 8 разрядов, 62,5 кГц
  TCCR1A = TCCR1A & 0xe0 | 1;
  TCCR1B = TCCR1B & 0xe0 | 0x09; 

  Serial.begin(19200);  // инициализируем последовательный порт, скорость 19200
  analogReference(INTERNAL); // опорное напряжение 1,1 В
  MsTimer2::set(2, timerInterupt); // прерывания по таймеру с периодом 2 мс 
  MsTimer2::start();              // разрешение прерывания  
  wdt_enable(WDTO_15MS); // разрешение работы сторожевого таймера, тайм-аут 15 мс 
  setPower = 5; // временно заданная мощность 5 Вт
}

//------------------------------- основной цикл -------------------------------
void loop() {

//------------------------------- цикл 20 мс ( регулятор мощности )
  if (flagReady == true) {
    flagReady= false; 

    //---------------- вычисление измеренных значений
    // вычисление напряжения
    measureU = powerU - (float)(averageU / MEASURE_PERIOD) * koeffU;
    if (measureU < 0) measureU=0;
        
    // вычисление тока
    measureI = (float)(averageI / MEASURE_PERIOD) * koeffI;
         
    // вычисление мощности
    measureP = powerU * measureI;

    regPwrInt = regPwrInt + (setPower - measureP) * koeffRegPwrInt;

     //------------------ регулятор мощности
    if ( setPower != 0) {
      regPwrInt = regPwrInt + (setPower - measureP) * koeffRegPwrInt; 
      if (regPwrInt < 0) regPwrInt=0;     // ограничение снизу
      if (regPwrInt > MAX_PWM) regPwrInt=MAX_PWM; // ограничение сверху
      // мертвое время ШИМ
      unsigned int  pwm = (unsigned int)regPwrInt;  // перевод в ШИМ
      if (pwm < DEAD_TIME) pwm=0;
      if (pwm > (MAX_PWM - DEAD_TIME)) pwm=MAX_PWM;    
      analogWrite(9, pwm); // ШИМ
    }
    else {            // выключение
      regPwrInt=0;
      analogWrite(9, 0); // ШИМ      
    }

    //--------------- выполнение распределенных операций в цикле 1 сек ---------------
    cycle20mcCount++;   // счетчик циклов 20 мс
    if (cycle20mcCount >= 50) cycle20mcCount= 0;  // время цикла 1 сек

    if (cycle20mcCount == 25) {
      //--------------------------- интервал 25, передача информации на компьютер
      Serial.print("U="); Serial.print(measureU, 2);  // напряжение
      Serial.print(" I="); Serial.print(measureI, 2);  // ток
      Serial.print(" P="); Serial.print(measureP, 2);  // мощность
      Serial.print(" PWM="); Serial.print(regPwrInt, 2);  // интегральное звено регулятора мощности
      Serial.println(" ");
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
