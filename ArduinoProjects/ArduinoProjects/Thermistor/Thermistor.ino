// к какому аналоговому пину мы подключены

#define THERMISTORPIN A3
#define THERMISTORNOMINAL 10000
#define koeffR 0.00335 // (1/To)
#define BCOEFFICIENT 3988
#define SERIESRESISTOR 10000

#define NUMSAMPLES 5

int samples[NUMSAMPLES];

void setup(void) {

Serial.begin(9600);
//analogReference(INTERNAL);
analogReference(EXTERNAL);

}

void loop(void) 
{
  uint8_t i;
  
  float average;
  
  // сводим показания в вектор с небольшой задержкой между снятием показаний
  
  for (i=0; i< NUMSAMPLES; i++) {
  
  samples[i] = analogRead(THERMISTORPIN);
  
  delay(10);
  
  }
  
  // рассчитываем среднее значение
  
  average = 0;
  
  for (i=0; i< NUMSAMPLES; i++) {
  
  average += samples[i];
  
  }
  
  float temp = getTemp(average);
  
  Serial.print("Temperature ");
  
  Serial.print(temp);
  
  Serial.println(" *C");
  
  delay(1000);

}

float getTemp(float average)
{
  average /= NUMSAMPLES;

Serial.print("Average analog reading ");

Serial.println(average);

  // конвертируем значение в сопротивление
  
  average = 1023 / average - 1;
  
  average = SERIESRESISTOR / average;

Serial.print("Thermistor resistance ");

Serial.println(average);
  
  float steinhart;
  
  steinhart = average / THERMISTORNOMINAL; // (R/Ro)
  
  steinhart = log(steinhart); // ln(R/Ro)
  
  steinhart /= BCOEFFICIENT; // 1/B * ln(R/Ro)
  
  steinhart += koeffR; // + (1/To)
  
  steinhart = 1.0 / steinhart; // инвертируем
  
  steinhart -= 273.15; // конвертируем в градусы по Цельсию

  return steinhart;
}
