#include <Wire.h>

// 1. Entwurf mit Impulsgeber und Lego-Motor A und Lego-Motor B.
//    Vgl. auch https://joy-it.net/de/products/SEN-Speed
//
// Anm. Berechtigung mit: sudo chmod a+rw /dev/ttyACM0

#define LED_PIN_A 11
#define LED_PIN_B 12
#define RASPI_PIN 13

// Digitale Pins fuer Interrupt-Nutzung auf dem  Uno: Pin 2 und Pin 3
// Pin 2: Motor A: 
#define ENCODER_PIN_MA 2
// Pin 3: Motor B:
#define ENCODER_PIN_MB 3 

// I2C-Adresse des Arduino...
#define SLAVE_ADDRESS 0x08

// Periodendauer von (1,5 s) erfordert (65636 - 1,5 * 34286) = 14107
// Periodendauer von (1 s) erfordert (65636 - 34286) = 31350 Takte:
// Periodendauer von (0,5 s) erfordert 15675 Takte, damit Initial (65636 - 15675) = 49961
// Periodendauer von (0,33 s) erfordert 31350 / 3 = 10450 Takte;
//                            damit 65636 - 10450 = 55186 Anfangswert.
// Periodendauer von (0,25 s) erfordert 31350 / 4 = 7837,5 Takte;
//                           damit 65636 - 7838 = 57798 Takte, (=> Anfangswert Zaehler)
// Periodendauer von (0,2 s) erfordert 31350 / 5 = 6270 Takte;
//                           damit 65636 - 6270 = 59366 Takte; (=> Anfangswert Zaehler)

////////////////////////////////////////////////////////////////////
// Die folgenden Werte gelten fuer ein Prescale-Wert von 256...
// Prescale 256 =>  TCCR1B |= (1 << CS12);
// const static int INIT_COUNT = 14107;          // T = 1,5 (s)
const static int INIT_COUNT = 31350;          // T = 1 (s)
// const static int INIT_COUNT = 49961;          // T = 0,5 (s)
// const static int INIT_COUNT = 55186;          // T = 0,33 (s)
// const static int INIT_COUNT = 57798;            // T = 0,25 (s)
// const static int INIT_COUNT = 59366;          // T = 0,2 (s)
////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////
// Die folgenden Werte gelten fuer ein Prescale-Wert von 1024...
// Prescale 1024 => TCCR1B |= (1 << CS12) | (1 << CS10); 
// const static int INIT_COUNT = 57064;          // T = 1 (s); Diff. zu 65636: ca. 34286 / 4 = 8572
// const static int INIT_COUNT = 48492;            // T = 2,185...


// counter ist der Taktzaehler des Zeitgebers
volatile unsigned long counter = 0;

// Impulse von Motor A:
volatile unsigned int counterMA = 0;
unsigned long previousMicrosMA = 0L;

// Impulse vom Motor B:
volatile unsigned int counterMB = 0;
unsigned long previousMicrosMB = 0L;

// token ist eine Kennung zur Synchronisation des Raspi
volatile unsigned long token = 0;

volatile byte led_status = LOW;

//////////////////////////////////////////////////////////////////
#define SIZE_DATA 32
// dataReceive[SIZE_DATA]: Buffer fuer Daten vom Raspi...
byte dataReceive[SIZE_DATA];
// dataRequest[SIZE_DATA]: Buffer fuer die Daten zum Raspi...
byte dataRequest[SIZE_DATA];
//////////////////////////////////////////////////////////////////

typedef enum { INITIAL = 'I', SUCCESS = 'S', ERROR = 'E', NOP = 'N' } Status;

volatile Status status = NOP;

void setup() 
{
  pinMode(LED_PIN_A, OUTPUT);
  pinMode(LED_PIN_B, OUTPUT);
  pinMode(RASPI_PIN, OUTPUT);

  // Encoder-Pins als Input... 
  pinMode(ENCODER_PIN_MA, INPUT);
  pinMode(ENCODER_PIN_MB, INPUT);

  // Interrupt-Routinen Moror A und Motor B einrichten...
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_MA), countMA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_MB), countMB, RISING);

  // I2C-einrichten...
  Wire.begin(SLAVE_ADDRESS);

  Wire.onReceive(receiveData);
   
  Wire.onRequest(requestData);

  // disable interrupts...
  noInterrupts();
  
  // Timer 1 zuruecksetzen...
  
  TCCR1A = 0;
  TCCR1B = 0;
  
  TCNT1 = INIT_COUNT;

  TCCR1B |= (1 << CS12);                    // 256 als Prescale-Wert
  
//  TCCR1B |= (1 << CS12) | (1 << CS10);        // 1024 als Prescale-Wert?

  // Set the CTC-Mode...
  TIMSK1 |= (1 << TOIE1);       // Timer Overflow Interrupt aktivieren...

  // allows interrupts...
  interrupts();
}

void loop() 
{
}

/************************************
 *  countMA() - Interrupt-Routine
 * Zaehlen der Impulse Motor A
 ************************************/
void countMA()
{
  if (micros() - previousMicrosMA > 700)
  {
    counterMA++;
    previousMicrosMA = micros();
  }
}

/************************************
 *  countMB() - Interrupt-Routine
 * Zaehlen der Impulse Motor B
 ************************************/
void countMB()
{
  if (micros() - previousMicrosMB > 700)
  {
    counterMB++;
    previousMicrosMB = micros();
  }
}

/************************************
 * receiveData(int byteCount)
 * Uebertragung Raspi => Arduino
 ************************************/
void receiveData(int byteCount)
{
  ///////////////////////////////////////////////////////////
  // Buffer zuruecksetzen und einlesen...
  memset(dataReceive, 0, sizeof(dataReceive));
  int temp = 0;
  while(Wire.available())
  {
    // Daten vom Raspi in den Datenbereich (Buffer) einlesen:
    dataReceive[temp++] = Wire.read(); 
  }
  ///////////////////////////////////////////////////////////
  
  // dataReceive[4]: status (Status)
  Status statusReceive = dataReceive[4];

  if (statusReceive == INITIAL)
  {
    // Im Zustand INITIAL wird token = 1L initial gesendet.
    token = 1L;
    counterMA = 0;
    counterMB = 0;
    status = SUCCESS;
  }
  else if (statusReceive == SUCCESS)
  {
    // dataReceive[0]...dataReceive[3]:  token (unsigned long)
    unsigned long tokenReceive = getLong(dataReceive);
    boolean isTokenSuccess = (tokenReceive == token);
    // Status ist SUCCESS und token stimmt ueberein...
    if (isTokenSuccess)
    {
      status = SUCCESS;
      token++;
    }
    else
    {
      status = ERROR;
      token = 0;
    }
  }
  else if (statusReceive == NOP)
  {
    status = NOP;
    token = 0;  
  }
  else
  {
    status = NOP;
    token = 0; 
  }
}

/************************************
 * requestData()
 * Uebertragung Arduino => Raspi 
 * 
 * Aufbau dataRequest:
 * dataRequest[0] ... dataRequest[3]: token (unsigned long, 4 Byte)
 * dataRequest[4]: status (char, 1 Byte)
 * dataRequest[5], dataRequest[6]: counterMA 
 * dataRequest[7], dataRequest[8]: counterMB
 ************************************/
void requestData()
{
  memset(dataRequest, 0, sizeof(dataRequest));
  setData(dataRequest, token);
  dataRequest[4] = status;
  
  ////////////////////////////////////////////
  // Andere Option: Uebertragung der Position
  // Ablage von unsigned long position: 
  // setData(&dataRequest[5], position);
  ////////////////////////////////////////////
  
  setData(&dataRequest[5], counterMA);
  setData(&dataRequest[7], counterMB);
  
  Wire.write((byte*)dataRequest, sizeof(dataRequest));
}

/************************************
 * getLong(byte *pData) - Wandlung
 ************************************/
unsigned long getLong(byte *pData)
{
  // Arduino und Raspi sind "little-endian"...
  // als Array: (((((data[3]<<8)+data[2])<<8)+data[1])<<8)+data[0];
  return (((((pData[3]<<8)+pData[2])<<8)+pData[1])<<8)+pData[0];
}

/************************************
 * setData() - Ablage value in pData
 * Ablage der value-Daten immer erst
 * NWT dann HWT...
 * Hier Ablage unsigned int als
 * 2 Bytes beginnend mit dem NWT...
 ************************************/
void setData(byte *pData, unsigned int value)
{
    pData[0] = (byte)(value & 0xff);
    value >>= 8;
    pData[1] = (byte)(value & 0xff);
}

/************************************
 * setData() - Ablage value in pData
 * Ablage der value-Daten immer erst
 * NWT dann HWT...
 * Hier Ablage unsigned long als
 * 4 folgende Bytes beginnend mit dem 
 * NWT...
 ************************************/
void setData(byte *pData, unsigned long value)
{
    pData[0] = (byte)(value & 0xff);
    value >>= 8;
    pData[1] = (byte)(value & 0xff);
    value >>= 8;
    pData[2] = (byte)(value & 0xff);
    value >>= 8;
    pData[3] = (byte)(value & 0xff);
}

/************************************
 * setData() - Ablage value in pData
 * Ablage der value-Daten immer erst
 * NWT dann HWT...
 * Hier Ablage long als 4 folgende
 * Bytes beginnend mit dem NWT
 * (vgl Ablage von unsigned long)...
 ************************************/
void setData(byte *pData, long value)
{
    pData[0] = (byte)(value & 0xff);
    value >>= 8;
    pData[1] = (byte)(value & 0xff);
    value >>= 8;
    pData[2] = (byte)(value & 0xff);
    value >>= 8;
    pData[3] = (byte)(value & 0xff);
}

ISR(TIMER1_OVF_vect) 
{
  // Timer 1 interrupt To-Do-Code...
  TCNT1 = INIT_COUNT;
  digitalWrite(LED_PIN_A, led_status);
  digitalWrite(LED_PIN_B, !led_status);
  digitalWrite(RASPI_PIN, led_status);  
  led_status = !led_status;             // LED ein, aus...
  counter++;
}
