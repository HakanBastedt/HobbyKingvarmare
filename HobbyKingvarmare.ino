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
 * Ett blink var 4-e sekund drar lite ström, men i medel kanske den drar 15 mA?
 *
 * PROGRAMMERING AV LÄGSTA CELL_SPÄNNING
 * =====================================
 * Vid uppstart kan lägsta tillåtna cell-spänning sättas på samma sätt
 * som en ESC typ. Gäller för 2S och 3S. Detta styr nivån för underspänning.
 *
 * 1. Vrid ratten till max värme (80% och mer av max räknas).
 *    Under programmeringen är värmen helt avslagen.
 * 2. Sätt i batteriet
 * 3. Vänta i fem sekunder.
 * 4. Värmaren blinkar med 5 korta blink följt av kort paus.
 * 5. Värmaren blinkar långsamt x gånger för inställd cellspänning.
 *    1 blink = 3.1 V, 2 blink = 3.2 V 3 blink = 3.3 V osv
 * 6. Värmaren blinkar med 5 korta blink (igen)
 * 7. Efter en kort paus startar en uppräkning med en blink i taget
 *    1 blink = 3.1 V
 *    2 blink = 3.2 V
 *    3 blink = 3.3 V osv
 *    När man nått den nivå man vill ha vrider man värmeratten till minimum (0-20% räknas)
 *    Då sparas detta värde och används i efterföljande användning av värmaren.
 *    Liten extra blink kvitterar att värdet lagrats.
 *    9 blink = 3.9 V
 * 8. Om ratten fortfarande står kvar på max 2 sekunder efter 9 blink så
 *    startar vanliga driften med PID-reglering osv.
 * 9. Om ratten vrids till < 80% under någon av punkterna utom 7 så startas
 *    direkt driften med gamla värden.

 */

#define DEFAULT_BATTERY_CELL_LOWEST_VOLTAGE  3  // Decimalen för gräns för en cells lägsta spänningsnivå.
// OBS att denna är bara allra första om ens det. Värde sätts via ratten och sparas/läses från EEPROM.
// 3 + 0.1* decimalen = gräns => 3+0.3
#include <PID_v1.h>
#include <SoftPWM.h>
#include <EEPROM.h>
#include <LowPower.h>

//Debug genom serial monitor Sätt till 0 för att få utskrift på serieporten
// Den får stå på, kansek kan vara kul för nån att titta på vad som händer
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
void    programmeraLowestCellVolt();

double Setpoint, Input, Output;             // Dessa tre är de viktiga för PID-regleringen
double Kp = 5.0, Ki = 0.3, Kd = 0.0;        // Tuning-parametrar för PID. Justera om du vill
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

#define MAX_TEMP 60     // Hur många grader som max utslag på poten betyder.
#define DEFAULT_BAT_VOLT_FAKTOR 0.0153
uint8_t lowestCellVoltDecimal;
double lowestVoltage=0;
uint8_t numberOfCells = 0;
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
  if (isnan(BatVoltFaktor) || BatVoltFaktor < 0.01 || BatVoltFaktor > 0.02) { // Inget vettigt värde fanns,
    BatVoltFaktor = DEFAULT_BAT_VOLT_FAKTOR;                                 // sätt ett närmevärde
    EEPROM.put(adr, BatVoltFaktor);
    Serialprint2("Skrev till EEPROM BatVoltFaktor*1000000=");
    Serialprint2(BatVoltFaktor * 1000000);
    Serialprint2(" adr=");
    Serialprintln2(adr);
  }

  adr = sizeof(BatVoltFaktor);                            // Spara på platsen efter BatVoltFaktor
  EEPROM.get(adr, lowestCellVoltDecimal);                 // Läs decimaldelen från EEPROM
  Serialprint2("Laste lowestCellVoltDecimal=");
  Serialprint2(lowestCellVoltDecimal);
  Serialprint2(" fran EEPROM adr=");
  Serialprintln2(adr);
  if (isnan(lowestCellVoltDecimal) || lowestCellVoltDecimal < 1 || lowestCellVoltDecimal > 9) { // Inget vettigt värde fanns,
    lowestCellVoltDecimal = DEFAULT_BATTERY_CELL_LOWEST_VOLTAGE;
    EEPROM.put(adr, lowestCellVoltDecimal);
    Serialprint2("Skrev till EEPROM lowestCellVoltDecimal = ");
    Serialprint2(lowestCellVoltDecimal);
    Serialprint2(" adr=");
    Serialprintln2(adr);
  }

  pinMode(heaterOutputPin,      OUTPUT);
  pinMode(inputLedPin,          OUTPUT);
  pinMode(heaterLedPin,         OUTPUT);
  digitalWrite(heaterOutputPin, LOW);     // Off
  digitalWrite(inputLedPin,     LOW);
  digitalWrite(heaterLedPin,    LOW);
  pinMode(potPin,               OUTPUT);
  digitalWrite(potPin,          HIGH);    // 5V till POT

  programmeraLowestCellVolt();            // Se om decimal-värdet ska ändras

  double voltage = readBatteryVoltage();
  if      (voltage > 6   && voltage < 8.6) numberOfCells = 2;
  else if (voltage > 8.5 && voltage < 13)  numberOfCells = 3;// Kan tillåta lite extra, 12.6 är väl normalt max.
  else if (voltage < 6.1)                  numberOfCells = 1;
  else                                     numberOfCells = 4; // eller mer. Obs att 15V är absolut max för ams1117-5.0 regulatorn
  Serialprint("Voltage=");
  Serialprint(voltage);
  Serialprint(" Cell#=");
  Serialprintln(numberOfCells);

  lowestVoltage = numberOfCells*(3+0.1*lowestCellVoltDecimal);
  
  blinkCells(numberOfCells);        // Är upptäckt antal celler fel så stannar programmet här inne!

  SoftPWMBegin();                   // Starta SoftPWM som sköter led fade-in-out och PWM till heater.

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

#define nReads 10

double readSetTempValue()
{
  // Range 0-1024 på ADCn, denna är okej.
  // Låt säga 0-60 oC. Här kan man ändra om man tycker potentiometern inte gör som man vill.
#define POT_FAKTOR (MAX_TEMP/1024.0/(1.0*nReads))
  double val = 0;
  for (int i = 0; i < nReads; i++)
    val += analogRead(setValuePin); // Läser spänningen över POT-en.
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
    digitalWrite(inputLedPin, HIGH);        // On
    digitalWrite(heaterLedPin, HIGH);       // On
    delay(20);
    digitalWrite(inputLedPin, LOW);        // Off
    digitalWrite(heaterLedPin, LOW);       // Off
    LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF); // Sätter processorn i Low Power mode
    digitalWrite(inputLedPin, HIGH);        // On
    digitalWrite(heaterLedPin, HIGH);       // On
    delay(20);
    digitalWrite(inputLedPin, LOW);        // Off
    digitalWrite(heaterLedPin, LOW);       // Off
    LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF); // Sätter processorn i Low Power mode
    // Processorn drar under 0.1 mA i detta läge.
    // Under 4 sekunder är LED-arna på 40 ms, dvs 1%. De drar ca 40 mA när på, dvs medel 0.4 mA
    // LDOn drar ca 5-10 mA, så den dominerar
  }
}

void checkVoltage(double voltage)
{
  if (voltage >= lowestVoltage)
    return;                                 // Ok
  Serialprint2("Fel spanning ar=");
  Serialprint2(voltage);
  Serialprint2(" gransen=");
  Serialprintln2(lowestVoltage);
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
  SoftPWMSetPercent(inputLedPin, onOff * 25); // 25 är kosmetiskt, ger en lugn blink
  Serialprint(" ");
  Serialprint(onOff);
  Serialprint(" ");
}

#define LEDS_ON    digitalWrite(inputLedPin,HIGH);digitalWrite(heaterLedPin,HIGH);
#define LEDS_OFF   digitalWrite(inputLedPin,LOW);digitalWrite(heaterLedPin,LOW);
#define CHECK(ms)   {unsigned long et=millis() + ms; while(millis() < et) { if (readSetTempValue() < 0.8*MAX_TEMP) return; } }
// CHECK hoppar ur rutinen så fort den upptäcker att ratten vridits tillbaks

void  programmeraLowestCellVolt()
{
  // 1. Vrid ratten till max (> 80%)
  // 2. Sätt i batteriet. Där börjar det.
  // 3. Fem sekunder med minst 80% av max på poten.
  CHECK(5000);

  // 4. Blinka 5 korta blink följt av paus
  for (int i = 0; i < 5; i++) {
    LEDS_ON;
    CHECK(30);
    LEDS_OFF;
    CHECK(150);
  }
  CHECK(2000);  // 2 sek paus
 
  // 5. Blinka ut hur många decimaler som är satt idag.
  for (int i = 0; i < lowestCellVoltDecimal; i++) {
    LEDS_ON;
    CHECK(500);
    LEDS_OFF;
    CHECK(1000);
  }
  CHECK(2000);  // 2 sek paus

  // 6. Blinka 5 korta blink följt av paus igen
  for (int i = 0; i < 5; i++) {
    LEDS_ON;
    CHECK(30);
    LEDS_OFF;
    CHECK(150);
  }
  CHECK(2000); // 2 sek paus igen

  // 7. Börja uppräkning, om ratten vrids ner nu så sätt detta värde.
  for (uint8_t decimaler = 1; decimaler < 10; decimaler++) {
    LEDS_ON;
    delay(200);
    LEDS_OFF;
    unsigned long endtime = millis() + 3000;     // 3 s paus
    while (millis() < endtime) {
      if (readSetTempValue() < 0.8 * MAX_TEMP) { // Ratten nerdragen. Sätt detta värde.
        lowestCellVoltDecimal = decimaler;       // Här är programmeringen
        int adr = sizeof(double);                // Spara på platsen efter BatVoltFaktor
        EEPROM.put(adr, lowestCellVoltDecimal);  
        for (int i = 0; i < 10; i++) {           // Får flasha lite för att visa seger!
          LEDS_ON;
          delay(60);
          LEDS_OFF;
          delay(90);
        }
        return;
      }
    }
  }
 // Nä det blev inget, bara fortsätt.
}
