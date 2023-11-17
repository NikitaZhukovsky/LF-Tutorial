
///////////////////////////////////////////////////////////////
/////////////////////// SETINGS ///////////////////////////////
///////////////////////////////////////////////////////////////
// motor's pins
int PWMA = 5;   // Порт ШИМ для управления скоростью мотора A
int PWMB = 6;  // Порт ШИМ для управления скоростью мотора B
int AIN_1 = 2;  // Порт для управления направлением мотора A
int AIN_2 = 4;  // Порт для управления направлением мотора A
int BIN_1 = 8;  // Порт для управления направлением мотора B
int BIN_2 = 7;  // Порт для управления направлением мотора B

// Массив сенсоров, дальше передается в analogRead()
int sensor[] = { 0, 1, 2, 3, 4, 5, 6, 7};

// Объявление переменной для скорости нашего робота
float max = 140;

// Объявление переменных для коэффициентов PID-регулятора
float KP = 0.065;
float KD = 0.84;
float KI = 0.0001;

#define FIRST_FILTRE 550         // Первый фильтр значений
#define SECOND_FILTRE 110        // Второй фильтр значений

// Определение константы для задержки запуска
const int START_DELAY = 1000;    // Задержка запуска

///////////////////////////////////////////////////////////////

// Объявление констант для светодиода и кнопки
#define LED 13
#define BUTTON_COLIB 12
#define BUTTON_START 1
//опредления массива с количеством сенсоров робота
const byte NUM_SENS = 8;

float lastError;
float _lastPosition;
float leftSpeed, rightSpeed;
int integral, last_proportional;

int calibratedMinimum[NUM_SENS];
int calibratedMaximum[NUM_SENS];
int sensorMin[NUM_SENS];
int sensorMax[NUM_SENS];
int sensorValues[NUM_SENS];



// functions declaration
bool pressButton_colib();
bool pressButton_start();

void setup() {
  pinMode(AIN_1, OUTPUT);
  pinMode(AIN_2, OUTPUT);
  pinMode(BIN_1, OUTPUT);
  pinMode(BIN_2, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(BUTTON_START, INPUT_PULLUP);
  pinMode(BUTTON_COLIB, INPUT_PULLUP);
  digitalWrite(LED, LOW);
  Serial.begin(9600);

  while (true)
  {
    if (pressButton_colib())
    {
      delay(500);
      LINE_COLIB();
      break;
    }
  }

}

void loop() {
  if (pressButton_start())
  {
    delay(START_DELAY);
    PID();
  }
}


//**********************************************************

//функция проверки нажатия кнопки
bool pressButton_colib()
{
  return !digitalRead(BUTTON_COLIB);
}
bool pressButton_start()
{
  return !digitalRead(BUTTON_START);
}
// Функция для калибровки линии
void LINE_COLIB()
{
  delay(500);
  for (int i = 0; i < 8; i++)
  {
    sensorMin[i] = 4096;
  }
  int startColobTime = millis();
  while (millis() < (startColobTime + 5000))
  {
    drive(50, -50);

    // Grabs incoming data from the photosensor
    for (int i = 0; i < 8; i++)
    {
      sensorValues[0] = analogRead(A0);
      if ( sensorValues[0] > sensorMax[0]) {
        sensorMax[0] = sensorValues[0];
      }
      if ( sensorValues[0] < sensorMin[0]) {
        sensorMin[0] = sensorValues[0];
      }

      sensorValues[1] = analogRead(A1);
      if ( sensorValues[1] > sensorMax[1]) {
        sensorMax[1] = sensorValues[1];
      }
      if ( sensorValues[1] < sensorMin[1]) {
        sensorMin[1] = sensorValues[1];
      }

      sensorValues[2] = analogRead(A2);
      if ( sensorValues[2] > sensorMax[2]) {
        sensorMax[2] = sensorValues[2];
      }
      if ( sensorValues[2] < sensorMin[2]) {
        sensorMin[2] = sensorValues[2];
      }

      sensorValues[3] = analogRead(A3);
      if ( sensorValues[3] > sensorMax[3]) {
        sensorMax[3] = sensorValues[3];
      }
      if ( sensorValues[3] < sensorMin[3]) {
        sensorMin[3] = sensorValues[3];
      }

      sensorValues[4] = analogRead(A4);
      if ( sensorValues[4] > sensorMax[4]) {
        sensorMax[4] = sensorValues[4];
      }
      if ( sensorValues[4] < sensorMin[4]) {
        sensorMin[4] = sensorValues[4];
      }
      sensorValues[5] = analogRead(A5);
      if ( sensorValues[5] > sensorMax[5]) {
        sensorMax[5] = sensorValues[5];
      }
      if ( sensorValues[5] < sensorMin[5]) {
        sensorMin[5] = sensorValues[5];
      }

      sensorValues[6] = analogRead(A6);
      if ( sensorValues[6] > sensorMax[6]) {
        sensorMax[6] = sensorValues[6];
      }
      if ( sensorValues[6] < sensorMin[6]) {
        sensorMin[6] = sensorValues[6];
      }

      sensorValues[7] = analogRead(A7);
      if ( sensorValues[7] > sensorMax[7]) {
        sensorMax[7] = sensorValues[7];
      }
      if ( sensorValues[7] < sensorMin[7]) {
        sensorMin[7] = sensorValues[7];
      }
    }
    drive(0, 0);
  }
}

// Функция для работы PID-регулятора
void PID() {
  while (true) {
    if (pressButton_start()) {
      while (true)
      {
        drive(0, 0);
        digitalWrite(LED, 1);
        delay(1000);
        digitalWrite(LED, 0);
        delay(1000);
      }
    }
    unsigned int position = bot_position();

    int proportional = ((int)position - 3500);
    int derivative = proportional - last_proportional;
    integral += proportional;
    last_proportional = proportional;

    float power_difference = proportional * KP + derivative * KD;
    leftSpeed = max + power_difference;
    rightSpeed = max - power_difference;
    if (leftSpeed > 255)
      leftSpeed = 255;

    if (rightSpeed > 255)
      rightSpeed = 255;

    drive(leftSpeed, rightSpeed);

  }
}

// Функция для определения положения робота на линии
float bot_position()
{
  bool onLine = false;
  uint32_t avg = 0; // this is for the weighted total
  uint16_t sum = 0; // this is for the denominator, which is <= 64000



  sensorValues[0] = analogRead(A0);
  sensorValues[0] = map(sensorValues[0], sensorMin[0], sensorMax[0], 0, 1000);
  sensorValues[0] = constrain(sensorValues[0], 0, 1000);

  sensorValues[1] = analogRead(A1);
  sensorValues[1] = map(sensorValues[1], sensorMin[1], sensorMax[1], 0, 1000);
  sensorValues[1] = constrain(sensorValues[1], 0, 1000);

  sensorValues[2] = analogRead(A2);
  sensorValues[2] = map(sensorValues[2], sensorMin[2], sensorMax[2], 0, 1000);
  sensorValues[2] = constrain(sensorValues[2], 0, 1000);

  sensorValues[3] = analogRead(A3);
  sensorValues[3] = map(sensorValues[3], sensorMin[3], sensorMax[3], 0, 1000);
  sensorValues[3] = constrain(sensorValues[3], 0, 1000);

  sensorValues[4] = analogRead(A4);
  sensorValues[4] = map(sensorValues[4], sensorMin[4], sensorMax[4], 0, 1000);
  sensorValues[4] = constrain(sensorValues[4], 0, 1000);

  sensorValues[5] = analogRead(A5);
  sensorValues[5] = map(sensorValues[5], sensorMin[5], sensorMax[5], 0, 1000);
  sensorValues[5] = constrain(sensorValues[5], 0, 1000);

  sensorValues[6] = analogRead(A6);
  sensorValues[6] = map(sensorValues[6], sensorMin[6], sensorMax[6], 0, 1000);
  sensorValues[6] = constrain(sensorValues[6], 0, 1000);

  sensorValues[7] = analogRead(A7);
  sensorValues[7] = map(sensorValues[7], sensorMin[7], sensorMax[7], 0, 1000);
  sensorValues[7] = constrain(sensorValues[7], 0, 1000);

  //Serial.println();
  for (uint8_t i = 0; i < 8; i++)
  {
    uint16_t value = sensorValues[i];

    // keep track of whether we see the line at all
    if (value > FIRST_FILTRE) {
      onLine = true;
    }

    // only average in values that are above a noise threshold
    if (value > SECOND_FILTRE)
    {
      avg += (uint32_t)value * (i * 1000);
      sum += value;
    }
  }

  if (!onLine)
  {
    // If it last read to the left of center, return 0.
    if (_lastPosition < (8 - 1) * 1000 / 2)
    {
      return 0;
    }
    // If it last read to the right of center, return the max.
    else
    {
      return (8 - 1) * 1000;
    }
  }

  _lastPosition = avg / sum;
  return _lastPosition;
}
// Функция для управления двиuгателями
void drive(int leftS, int rightS)
{

  int left = leftS; // Инициализация левой скорости
  int right = rightS; // Инициализация правой скорости

  left = constrain(left, -255, 255);   //Ограничение левой скорости в заданном диапазоне
  right = constrain(right, -255, 255); // Ограничение правой скорости в заданном диапазоне


  if (left >= 0)
  {
    digitalWrite(BIN_1, LOW); // Установка направления вращения мотора A
    digitalWrite(BIN_2, HIGH); // Установка направления вращения мотора A
  }
  else
  {
    digitalWrite(BIN_1, HIGH); // Установка направления вращения мотора A
    digitalWrite(BIN_2, LOW); // Установка направления вращения мотора A

  }

  if (right >= 0)
  {
    digitalWrite(AIN_1, LOW); // Установка направления вращения мотора B
    digitalWrite(AIN_2, HIGH); // Установка направления вращения мотора B
  }
  else
  {
    digitalWrite(AIN_1, HIGH); // Установка направления вращения мотора B
    digitalWrite(AIN_2, LOW); // Установка направления вращения мотора B
  }


  analogWrite(PWMA, abs(right)); // Управление скоростью мотора A
  analogWrite(PWMB, abs(left)); // Управление скоростью мотора B
}
