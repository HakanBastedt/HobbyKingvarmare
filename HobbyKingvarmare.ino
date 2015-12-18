/**
 *
 * PID kontroll av Hobbykings Universal Heater.
 * =============================================
 * INGÅNGAR
 * ========
 * SET-value: BÖR-värde: Potentiometer på A1
 * Batterispänning: Spänningsdelare mäts på A0. 10K + 4K7 motstånd.
 * ÄR-Värde: Temp-sensor på A2 10 mV/C
 * PÅ/AV toggle: Finns ej. Apparaten är alltid på. 
 * 
 * UTGÅNGAR:
 * =========
 * Värmarspänning: PWM output på D4
 * LED Input: Digital output på D5
 * LED Värmarspänning: PWM output på D6
 * +5V till Potentiometern på Vcc 6.5kOhm, 5V => < 1 mA
 *
 * LED INPUT
 * ============
 * Vid start:
 * Blink 1s-0.5s  x gånger för x antal celler funna
 *
 * Under drift:
 * Fade-in fade-out led
 * Normal = i drift
 * Långsam = Temp < 5 grader = noll (fast den reglerar för det)
 * Normal fade-in fade-out = Normadrift
 *
 * Underspänning
 * =============
 * Både led blinkar x (kort - kortare) plus paus för x antal celler
 * upptäckt i början. På första raden kod kan tröskelspänningen sättas.
 *
 * LED värmare
 * ===========
 * Start: avstängd.
 *
 * I drift:
 * Intensitet motsvarar strömmen ut till värmetråden.
 * Värdet kommer från PID och regleras med SoftPWM
 * 
 * Vid underspänning:
 * Processorn sätts i PowerDown-mode och det är i princip bara spänningsregulatorn som drar ström 10 mA.
 * Ett blink var 8-e sekund drar lite ström, men i medel kanske den drar 15 mA?
 */

#define BATTERY_CELL_LOWEST_VOLTAGE  3.3  // Gräns för en cells lägsta spänningsnivå.

#include <PID_v1.h>
#include <SoftPWM.h>
#include <EEPROM.h>
#include <LowPower.h>

//Debug genom serial monitor Sätt till 1 för att få utskrift på serieporten
#if 0
#define Serialprint(x)
#define Serialprintln(x)
#define Serialbegin(x)
#define Serialprint2(x)
#define Serialprintln2(x)
#else
#define Serialbegin(x)    Serial.begin(x)
#if 0
#define Serialprint(x)
#define Serialprintln(x)
#else
#define Serialprint(x)    Serial.print(x)
#define Serialprintln(x)  Serial.println(x)
#endif
#define Serialprint2(x)   Serial.print(x)
#define Serialprintln2(x) Serial.println(x)
#endif

#define setValuePin     A1 // Analog input från POT     
#define tempValuePin    A2 // Analog input från LM35DZ  
#define batVoltagePin   A0 // Analog input  från spänningsdelare 
#define heaterOutputPin 4  // SoftPWM för att styra denna digitala output 
#define inputLedPin     5  // Digital output "Heater"- ledden 
#define heaterLedPin    6  // SoftPWM för blink mjukt och fint 
#define potPin          7  // 5V till potenitiometer

double  readBatteryVoltage();
double  readSetTempValue();
double  readTempValue();
void    blinkCells(uint8_t n);
void    checkVoltage(double voltage);
void    fadeBlink(unsigned long delta);
uint8_t numberOfCells = 0;


double Setpoint, Input, Output;             // Dessa tre är de viktiga för PID-regleringen
double Kp = 5.0, Ki = 0.3, Kd = 0.0;        // Tuning-parametrar för PID. Justera om du vill
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

#define DEFAULT_BAT_VOLT_FAKTOR 0.0153
double BatVoltFaktor;
// BatVoltFaktor = Omvandlar inläst värde av AD-omvandlaren till ett riktigt spännings värde.
// Ska ligga kring 0.0153 men det beror på exakta värden på motstånden i spänningsdelaren
// Ska helst justeras in, se kod i setup()

void setup()
{
  Serialbegin(115200);

  uint8_t adr = 0;
  EEPROM.get(adr, BatVoltFaktor);                 // Läs BatVoltFaktor från EEPROM
  Serialprint2("Laste BatVoltFaktor*1000000=");
  Serialprint2(BatVoltFaktor * 1000000);
  Serialprint2(" fran EEPROM adr=");
  Serialprintln2(adr);
  if (isnan(BatVoltFaktor)|| BatVoltFaktor < 0.01 || BatVoltFaktor > 0.02) { // Inget vettigt värde fanns, 
    BatVoltFaktor = DEFAULT_BAT_VOLT_FAKTOR;                                 // sätt ett närmevärde
    EEPROM.put(adr, BatVoltFaktor);
    Serialprint2("Skrev till EEPROM BatVoltFaktor*1000000=");
    Serialprint2(BatVoltFaktor * 1000000);
    Serialprint2(" adr=");
    Serialprintln2(adr);
  }
#if 0
  // Kod för att justera värdet på BatVoltFaktor i eeprom
  BatVoltFaktor = 0.01533; // Justera detta så att multimeter och detta program ger samma spänning från batteriet.
  EEPROM.put(adr, BatVoltFaktor);
  Serialprint2("Skrev till EEPROM BatVoltFaktor*1000000=");
  Serialprint2(BatVoltFaktor * 1000000);
  Serialprint2(" adr=");
  Serialprintln2(adr);
  Serialprintln2("Glom inte att andra detta sa att EEPROM endast lases!");
  // Glöm inte att sätta till 1 ovan när du är klar!
#endif

  pinMode(heaterOutputPin,      OUTPUT);
  pinMode(inputLedPin,          OUTPUT);
  pinMode(heaterLedPin,         OUTPUT);
  digitalWrite(heaterOutputPin, LOW);     // Off
  digitalWrite(inputLedPin,     LOW);
  digitalWrite(heaterLedPin,    LOW);
  pinMode(potPin,               OUTPUT);
  digitalWrite(potPin,          HIGH);    // 5V till POT

  double voltage = readBatteryVoltage();
  if      (voltage > 6   && voltage < 8.6) numberOfCells = 2;
  else if (voltage > 8.5 && voltage < 13)  numberOfCells = 3;// Kan tillåta lite extra, 12.6 är väl normalt max.
  else if (voltage < 6.1)                  numberOfCells = 1;
  else                                     numberOfCells = 4; // eller mer. Obs att 15V är absolut max för ams1117-5.0 regulatorn
  Serialprint("Voltage=");
  Serialprint(voltage);
  Serialprint(" Cell#=");
  Serialprintln(numberOfCells);

  blinkCells(numberOfCells);        // Är upptäckt antal celler fel så stannar programmet här inne!

  SoftPWMBegin();                   // Starta SoftPWM som sköter led fade-in-out och PWM till heater.
  SoftPWMSet(heaterLedPin, 0);      // Avstängt
  SoftPWMSet(inputLedPin, 0);       // Avstängt

  Input    = readTempValue();       //initialisera variablerna för PID-reglering med första värden.
  Setpoint = readSetTempValue();
  myPID.SetMode(AUTOMATIC);         // Starta PID-reglering
  myPID.SetOutputLimits(0, 60);     // Max 60% effect räcker mer än väl. Överbelasta inte MOSFET-en, ligg på 60% eller mindre
}

void loop()
{
  double voltage = readBatteryVoltage(); // Kolla att spänningen är OK så att inte batteriet dras ur fullständigt.
  checkVoltage(voltage);                 // Har den väl blivit för låg så får man starta om.

  Input = readTempValue();
  Setpoint = readSetTempValue();

  Serialprint("Spanning: ");
  Serialprint(voltage);
  Serialprint(" Set temp: ");
  Serialprint(Setpoint);
  Serialprint(" Aktuell temp: ");
  Serialprint(Input);

  if (abs(Input - Setpoint) < 5) {   // Här är en fix som rampar upp lite snabbare.
    myPID.SetOutputLimits(0, 20);    // Minska max utslag och förstärkning närmare setpoint
    myPID.SetTunings(Kp, Ki, Kd);
  } else {
    myPID.SetOutputLimits(0, 60);     // Mer utslag och mer förstärkning långt ifrån. Vi vill ha MAX fart
    myPID.SetTunings(4 * Kp, 4 * Ki, Kd); // dock inte mer än 60% - överbelasta inte MOSFETen
  }

  myPID.Compute();
  SoftPWMSetPercent(heaterOutputPin, Output);
  SoftPWMSetPercent(heaterLedPin, Output);
  Serialprint(" PID Heat: ");
  Serialprint(Output);
  Serialprintln(" ");

  if (Setpoint < 5) {       // Blinka sakta
    fadeBlink(4000);
  } else {                  // Normaldrift
    fadeBlink(600);
  }
}

double readBatteryVoltage()
{
  // Läs spänningen inom 0-5V range
  double val = analogRead(batVoltagePin);
  return BatVoltFaktor * val;         // Returnera 0 - 16-ish Volt
}

double readSetTempValue()
{
  // Range 0-1024 på ADCn, denna är okej.
  // Låt säga 0-60 oC. Här kan man ändra om man tycker potentiometern inte gör som man vill.
#define POT_FAKTOR (60.0/1024.0)
  double val = analogRead(setValuePin); // Läser spänningen över POT-en.
  return POT_FAKTOR * val;
}

double readTempValue()
{
#define nReads 10
#define TEMP_FAKTOR (100.0*(5.0/1024.0)/(1.0*nReads))
  // Denna faktor stämmer, spänningen genereras av LM35DZ med 10mV/grad
  // Detta behöver bara skalas om till 5V och 1024 bitars upplösning.
  // Översamplar 10 gånger
  double val = 0;
  for (int i = 0; i < nReads; i++)
    val += analogRead(tempValuePin);
  return TEMP_FAKTOR * val;
}

void blinkCells(uint8_t cells)
{
  if (cells == 2 || cells == 3) {
    digitalWrite(inputLedPin, HIGH);    // On
    delay(1000);
    digitalWrite(inputLedPin, LOW);     // Off
    delay(500);
    digitalWrite(inputLedPin, HIGH);    // On
    delay(1000);
    if (cells == 3) {
      digitalWrite(inputLedPin, LOW);   // Off
      delay(500);
      digitalWrite(inputLedPin, HIGH);  // On
      delay(1000);
    }
    digitalWrite(inputLedPin, LOW);     // Off, paus innan fortsätter
    delay(1000);
    return;                             // Allt ok, fortsätt.
  } else {
    stopLoop();                         // Fel antal celler upptäckt
  }
}

void stopLoop()
{
  Serialprintln2("STOPP pga fel spanning");
  TIMSK2 = 0;                               // SoftPWM avstängt. Pinnarna är normala efter detta.
  pinMode(heaterLedPin, OUTPUT);           
  pinMode(inputLedPin, OUTPUT);                
  pinMode(heaterOutputPin, OUTPUT);            
  digitalWrite(heaterOutputPin, LOW);       // Stäng av värmen.
  digitalWrite(potPin,          LOW);
  digitalWrite(heaterLedPin,    LOW);
  digitalWrite(inputLedPin,     LOW);
  
  while (1) {                               // Här är en liten oändlig loop om underspänning inträffar
    LowPower.powerDown(SLEEP_4S,            // Sätter processorn i Low Power mode 
                       ADC_OFF, 
                       BOD_OFF); 
    digitalWrite(inputLedPin, HIGH);        // On
    digitalWrite(heaterLedPin, HIGH);       // On
    delay(20);
    digitalWrite(inputLedPin, LOW);        // Off
    digitalWrite(heaterLedPin, LOW);       // Off
    LowPower.powerDown(SLEEP_120MS,                 // Sätter processorn i Low Power mode 
                       ADC_OFF, 
                       BOD_OFF); 
    digitalWrite(inputLedPin, HIGH);        // On
    digitalWrite(heaterLedPin, HIGH);       // On
    delay(20);
    digitalWrite(inputLedPin, LOW);        // Off
    digitalWrite(heaterLedPin, LOW);       // Off
  }
}

void checkVoltage(double voltage)
{
  if (voltage >= numberOfCells * BATTERY_CELL_LOWEST_VOLTAGE)
    return;                                 // Ok
  Serialprint2("Fel spanning ar=");
  Serialprint2(voltage);
  Serialprint2(" gransen=");
  Serialprintln2(numberOfCells * BATTERY_CELL_LOWEST_VOLTAGE);
  stopLoop();                               // Underspänning. Stopp.
}

unsigned long oldDelta = -1;
void fadeBlink(unsigned long delta)
{
  unsigned long time = millis();
  uint8_t onOff = ((time / delta) + 1) & 0x1; // Start with 1
  if (oldDelta != delta) {
    oldDelta = delta;
    SoftPWMSetFadeTime(inputLedPin, delta, delta);
  }
  SoftPWMSetPercent(inputLedPin, onOff * 20); // 20 är kosmetiskt, ger en lugn blink
  Serialprint(" ");
  Serialprint(onOff);
  Serialprint(" ");
}
