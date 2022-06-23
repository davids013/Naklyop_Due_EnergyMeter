//#include <LiquidCrystal.h>
#include <LiquidCrystal_1602_RUS.h>

#define aInPin1     10                         //num of voltage analog input pin
#define dInPin1     31                         //num of voltage digital input pin
#define outHighPin  29                         //num of high voltage out pin    
#define outLowPin   33                         //num of low voltage out pin                     
#define syncPin     40                         //num of synchropulse input pin
#define statusPin   44                         //num of status indicator out pin

const bool isVoltageShown = 0;                //show calculated voltage (1) or not (0)
const float coeffEperV = 2.95;                //energy (Joules) per 1.0V
const float coeffVperSerial = 0.014707;       //energy (Joules) per 1.0 from serial 
const float coeffResDivider = 2.322;          //input resistor divider ratio
const float mVer = 3300 / 1023;               //mV per 1 ADC bit
const unsigned int iters = 30;                //count of measure steps
const unsigned int delayBeforeMeasure = 1;    //microseconds delay before measurements
const unsigned int delayAfterMeasure = 5000;  //milliseconds min to next pulse
const byte dataSize = 3;                      //size of data buffer for RX/TX
const byte checkCounts = 5;                  //counts of logic level of pin (~5 counts/us)
uint8_t symbol_emis[8] = { B00100, B10101, B01110, B11011, B01110, B10101, B00100, B00000 };
struct Status {
  const String OK = "ok";
  const String ERRORS = "ошибка";
  const String LOW_SIGNAL = "слаб.сигн.";
  const String WARNING = "внимание";
  const String OVERPOWER = "ОПАСНАЯ Е!";
};
const Status State;

bool isMeasureMode = true;
int ar[iters];                                //array of analogReadDirect measurings 
unsigned int pulses = 0;                      //counter of sync pulses from power on
int offset = 0;                               //offset for analog amplitude measurings

//LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
LiquidCrystal_1602_RUS lcd(8, 9, 4, 5, 6, 7);


void setup() {
  adc_setup();
  Serial.begin(9600);
  pinMode(outHighPin, OUTPUT);
  digitalWrite(outHighPin, HIGH);
  pinMode(outLowPin, OUTPUT);
  digitalWrite(outLowPin, LOW);
  pinMode(statusPin, OUTPUT);
  digitalWrite(statusPin, LOW);
  pinMode(syncPin, INPUT);
  pinMode(dInPin1, INPUT_PULLUP);
  isMeasureMode = digitalReadDirect(dInPin1);
  if (!isMeasureMode) Serial1.begin(9600);
  offset = analogRead(aInPin1);
  lcd.begin(16, 2);
  lcd.print(L"Монитор энергии");
  lcd.setCursor(0, 1);
  if (isMeasureMode) {
    lcd.print(L"     измер.");
  } else 
    lcd.print(L"     печат.");
  lcd.createChar(7, symbol_emis);
  lcd.setCursor(11, 1);
  lcd.blink();
}

void loop() {
  if (!isMeasureMode) {
    int dataSum = getSerial1Int();
    sendDataToSerial1();
    if (dataSum > 0 && dataSum <= 255) {
      double voltage = dataSum * coeffVperSerial;
      double energy = dataSum * coeffVperSerial * coeffEperV;
      pulses++;
      String state = getState(voltage, energy); 
      statusToLCD(energy, voltage, state);
      statusToSerial(energy, voltage, state);
      lcdBlink(false, 0, 0);
      delay(delayAfterMeasure);
      lcd.setCursor(15, 0);
      lcdBlink(true, 15, 0);
    } else delay(100);
    return;
  }
  offset = analogRead(aInPin1);
  if (isInputStable(syncPin, HIGH)) {
    delayMicroseconds(delayBeforeMeasure);
    readInput();
    if (isInputStable(syncPin, HIGH)) {
      delay(50);
      return;
    }
    float voltage = (findMaxADCcount() - offset) * mVer * coeffResDivider / 1000.0;
    float energy = voltage * coeffEperV;
    if (voltage >= 0.01) {
      pulses++;
      String state = getState(voltage, energy); 
      statusToLCD(energy, voltage, state);
      statusToSerial(energy, voltage, state);
      lcdBlink(false, 0, 0);
      delay(delayAfterMeasure);
      lcd.setCursor(15, 0);
      lcdBlink(true, 15, 0);
    }
  }
}

void adc_setup() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID37;
  ADC->ADC_CR = ADC_CR_SWRST;
//  ADC->ADC_MR 
//              |= ADC_MR_TRGEN_EN 
//              | ADC_MR_PRESCAL(1) 
//              | ADC_MR_TRGSEL_ADC_TRIG3
//  ;
  ADC->ADC_IDR = ~(0ul);
  ADC->ADC_CHDR = ~(0ul);
//  for (int i = 0; i < nbr_channels; i++) {
//    ADC->ADC_CHER |= ADC_CHER_CH0 << channels[i];
//  }
//  ADC->ADC_IER |= ADC_IER_EOC0 << channels[nbr_channels - 1];
  ADC->ADC_PTCR |= ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS;
  NVIC_EnableIRQ(ADC_IRQn);
}

inline void digitalWriteDirect(int pin, boolean val) {
  if (val)  g_APinDescription[pin].pPort -> PIO_SODR = g_APinDescription[pin].ulPin;
  else      g_APinDescription[pin].pPort -> PIO_CODR = g_APinDescription[pin].ulPin;
}

inline int digitalReadDirect(int pin) {
  return !!(g_APinDescription[pin].pPort -> PIO_PDSR & g_APinDescription[pin].ulPin);
}

int sumADCcounts() {
  unsigned long acc = ar[0];
  for (int i = 1; i < iters; i++) {
    acc += ar[i];
  }
  return acc / iters;
}

int findMaxADCcount() {
  byte averCounts = 2;
  byte factCounts = 0;
  if (iters < averCounts) averCounts = iters;
  for (int i = 1; i < iters - 1; i++) {
    if (
        (ar[i] < ar[i+1] * 0.7 && ar[i] < ar[i-1] * 0.7) || 
        (ar[i] > ar[i+1] / 0.7 && ar[i] / ar[i-1] / 0.7)
       ) ar[i] = -1;
  }
  unsigned int sum = 0;
  //for (int i = 0; i < averCounts; i++) {
  for (int i = 0; i < iters; i++) {
    for (int j = i + 1; j < iters; j++) {
      if (ar[i] < ar[j]) {
        int temp = ar[i];
        ar[i] = ar[j];
        ar[j] = temp;
      }
    }
    
    //if (i > 0 && ar[i] > ar[i-1]*2) ar[i] = ar[i-1];
    //sum += ar[i];
    //factCount++;
  }
  for (int i = 0; i < averCounts; i++) {
    if (ar[i] < 0) break;
    if (ar[i] > ar[i+1] / 0.7) {
      for (int j = i; j < iters - 1; j++) {
        ar[j] = ar[j+1];
      }
      ar[iters - 1] = -1;
      i--;
    }
    else {
      sum += ar[i];
      factCounts++;
    }
  }
  return sum / factCounts;
}

bool isInputStable(int pin, bool state) {
  digitalWriteDirect(statusPin, HIGH);
  for (int i = 0; i < checkCounts; i++) {
    if (digitalReadDirect(pin) != state) {
      digitalWriteDirect(statusPin, LOW);
      return false;
    }
  }
  digitalWriteDirect(statusPin, LOW);
  return true;
}

void readInput() {
  digitalWriteDirect(statusPin, HIGH);
  for (int i = 0; i < iters; i++) {
//    digitalWriteDirect(statusPin, HIGH);
    ar[i] = analogRead(aInPin1);
//    digitalWriteDirect(statusPin, LOW);
  }
  digitalWriteDirect(statusPin, LOW);
}

String getState(double voltage, double energy) {
  if (energy >= 10.0) { return State.OVERPOWER; }
    else if (voltage <= 0.1) { return State.LOW_SIGNAL; }
    else return State.OK; 
}

void statusToLCD(float energy, float voltage, String state) { 
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(L"E = ");
  lcd.print(energy);
  if (voltage < 10.0) lcd.print("_");
  lcd.print(L"Дж");
  if (isVoltageShown) {
    lcd.setCursor(11, 1);
    lcd.print(voltage);
    lcd.print(L"В");
  }
  if (pulses >= 100) { lcd.setCursor(12, 0); }
    else if (pulses >= 10) { lcd.setCursor(13, 0); }
    else lcd.setCursor(14, 0);
  lcd.print(pulses);
  lcd.write(7);
  //lcd.print("p");
  if (!state.equals(State.OK)) {
    lcd.setCursor(0, 1);
    lcd.print(state);
  }
}

void lcdBlink(bool isBlink, byte index, byte row) {
  if (!isBlink) {
    lcd.noBlink();
  } else {
    lcd.setCursor(index, row);
    lcd.blink();
  }
}

void statusToSerial(float energy, float voltage, String state) {
  if (isVoltageShown) {
    Serial.print("U = ");
    Serial.print(voltage);
    Serial.println(" V");
  }
  Serial.print("E = ");
  Serial.print(energy);
  Serial.println(" J");
  Serial.print("Pulses: ");
  Serial.println(pulses);
  if (!state.equals(State.OK)) {
    Serial.print("Status: ");
    Serial.println(state); 
  }
  Serial.println();
  Serial.flush();
}

int getSerial1Int() {
  if (Serial1.available() > 0) {
    Serial.print("Recieved RX -> ");
    byte readBuf[dataSize];
    readBuf[dataSize - 1] = 0;
    Serial1.readBytes(readBuf, dataSize);
    byte b = readBuf[dataSize - 1];
    if (b == 0) return -1;
    if (dataSize >= 3) {
      byte maxByte = readBuf[0];
      byte minByte = readBuf[1];
      if (maxByte - minByte != b) {
        Serial.print(" MATH ERROR:  ");
        Serial.print(maxByte);
        Serial.print(" - ");
        Serial.print(minByte);
        Serial.print(" = ");
        Serial.print(maxByte - minByte);
        Serial.print(" != (");
        Serial.print(b);
        Serial.print(") from RX");
        return -2;
      }
      Serial.print(" max=");
      Serial.print(int(maxByte));
      Serial.print(" min=");
      Serial.print(int(minByte));
    }
    Serial.print(" amplitude=");
    Serial.println(int(b));
    Serial.flush();
    return int(b);
  }
  return -3;
}

void sendDataToSerial1() {
  if (Serial.available() > 0) {
    char b = Serial.read();
    Serial1.print(b);
    Serial1.flush();
    Serial.print("Data sent: '");
    Serial.write(b);
    Serial.print("' ");
    Serial.print(b, BIN);
    Serial.print(" (0x");
    Serial.print(b, HEX);
    Serial.print(") = ");
    Serial.print(int(b));
    Serial.println();
    Serial.flush();
    if (false) {
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print("Sent:");
      lcd.setCursor(0, 1);
      lcd.print("'");
      lcd.print(char(b));
      lcd.print("' ");
      lcd.print(b, BIN);
      lcd.print(" 0x");
      lcd.print(b, HEX);
    }
  }
}
