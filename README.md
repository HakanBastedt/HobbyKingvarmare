# HobbyKingvarmare
Min förbättrade version av Hobbykings värmare för sändarpåse.

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

