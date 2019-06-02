#include <OneWire.h>

#define POWER_MODE  0 // режим питания, 0 - внешнее, 1 - паразитное

byte bufData[9];  // буфер данных

OneWire sensTemp1 (16);  // датчик подключен к выводу 16(A2)
float temperature1;  // измеренная температура

OneWire sensTemp2 (17);  // датчик подключен к выводу 17(A3)
float temperature2;  // измеренная температура

void setup() {
  Serial.begin(9600); // инициализируем порт, скорость 9600  
}

void loop() { 

  sensTemp1.reset();  // сброс шины
  sensTemp1.write(0xCC, POWER_MODE); // пропуск ROM
  sensTemp1.write(0x44, POWER_MODE); // инициализация измерения

  sensTemp2.reset();  // сброс шины
  sensTemp2.write(0xCC, POWER_MODE); // пропуск ROM
  sensTemp2.write(0x44, POWER_MODE); // инициализация измерения
  
  delay(900);  // пауза 0,9 сек
  
  sensTemp1.reset();  // сброс шины
  sensTemp1.write(0xCC, POWER_MODE); // пропуск ROM  
  sensTemp1.write(0xBE, POWER_MODE); // команда чтения памяти датчика  
  sensTemp1.read_bytes(bufData, 9);  // чтение памяти датчика, 9 байтов
  if (OneWire::crc8(bufData, 8) == bufData[8]) {  // проверка CRC
    // данные правильные
    temperature1 = (float)((int)bufData[0] | (((int)bufData[1]) << 8)) * 0.0625 + 0.03125;    
  }

  sensTemp2.reset();  // сброс шины
  sensTemp2.write(0xCC, POWER_MODE); // пропуск ROM  
  sensTemp2.write(0xBE, POWER_MODE); // команда чтения памяти датчика  
  sensTemp2.read_bytes(bufData, 9);  // чтение памяти датчика, 9 байтов
  if (OneWire::crc8(bufData, 8) == bufData[8]) {  // проверка CRC
    // данные правильные
    temperature2 =  (float)((int)bufData[0] | (((int)bufData[1]) << 8)) * 0.5 + 0.25;               
  }

  // передача температуры на компьютер
    Serial.print("T1=");
    Serial.print(temperature1);

    Serial.print(" T2=");
    Serial.print(temperature2);
       
    Serial.println(" ");
}
