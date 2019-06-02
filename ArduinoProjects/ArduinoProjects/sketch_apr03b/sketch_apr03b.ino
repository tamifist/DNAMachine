bool isSwitchedOn;

void setup() {
  // put your setup code here, to run once:
  pinMode(2, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(12, OUTPUT);
}

void loop() 
{
  if(isSwitchedOn)
  {
    digitalWrite(2, LOW);
    isSwitchedOn = false;
  }
  else
  {
    digitalWrite(2, HIGH);
    isSwitchedOn = true;
  }
  
  delay(1000);
}
