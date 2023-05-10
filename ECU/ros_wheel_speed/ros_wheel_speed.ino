

#include <Arduino.h>

volatile unsigned long rpmtime;
float rpmfloat;
unsigned int rpm = 0;
volatile bool tooslow = 1;
volatile unsigned int rpm_ticks = 0;

void setup()
{
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1B |= (1 << CS12);  // Prescaler 256
    TIMSK1 |= (1 << TOIE1); // enable timer overflow
    
    pinMode(2, INPUT);
    attachInterrupt(0, RPM, FALLING);

    Serial.begin(115200);
    delay(300); // .3 sec dela
}

ISR(TIMER1_OVF_vect)
{
    tooslow = 1;
}

void loop()
{
    delay(1000);
    if (tooslow == 1)
    {
        Serial.println("SLOW!");
    }
    else
    {
        rpmfloat = 60 / (rpmtime / 31250.00);
        rpm = round(rpmfloat);
        Serial.println(rpmtime, DEC);
        rpm_ticks = 0;
    }
}

void RPM()
{
    rpm_ticks ++;
    rpmtime = TCNT1;
    TCNT1 = 0;
    tooslow = 0;
}
