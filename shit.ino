#include <EEPROM.h>
#define LED_PIN LED_BUILTIN
#define CLK_HZ 16000000
#define PRE_SCL_MS 1024
#define COM_RAT_MS 1000
#define COM_RAT_DATA 500
#define PRE_SCL_DATA 1024
#define PRE_SCL_RPM 64
#define INPUT_BUTTON 3
#define EDGES 44
#define THR_MIN 0
#define THR_MAX 1
#define MAX_WAIT 10000
#define D2 2

char mode = 0; //mode 0 - prompt, mode 1 - periodic
volatile long periodms = 400; //time for periodic mode
int mafavg = 0;
long mafsum = 0;
long mafavgn = 0;
int thravg = 0;
long thrsum = 0;
long thravgn = 0;
int rpmavg = 0;
volatile long rpmsum = 0;
volatile long rpmavgn = 0;

long teethtime = 0;
volatile long over = 0;
volatile long over2 = 0;
unsigned short int throttle = 0;
unsigned short int maf = 0;
volatile byte seq = 0;
unsigned short int thrmin = 0;
unsigned short int thrmax = 1023;
char started = 0;

void setup()
{
  cli();
  Serial.begin(57600);
  pinMode(LED_PIN, OUTPUT);
  
  //Timer 0 (Periodic send) setup
  TCCR0A = 0;
  TCCR0B = 0;
  TCNT0 = 0;
  TCCR0A |= (1 << WGM01);
  OCR0A = (CLK_HZ / ((long) PRE_SCL_MS * (long) COM_RAT_MS)) - 1;
  TCCR0B |= (1 << CS02) | (1 << CS00); //1024 prescale, but wait to start
  TIMSK0 = (1 << OCIE0A);

  //Timer 1 (RPM) setup
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  //TCCR1B |= (1 << CS11) | (1 << CS10); //64 prescale, but wait to start clock
  TIMSK1 = (1 << TOIE1); //only overflow

  //Timer 2 (take data periodically) setup
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  OCR2A = (CLK_HZ / ((long) PRE_SCL_DATA * (long) COM_RAT_DATA)) - 1;
  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS22) | (1 <<CS20); //1024 prescale
  TIMSK2 = (1 << OCIE2A);

  //Pin change interrupts (D2, RPM)
  DDRD &= ~(1 << DDD2);
  PORTD |= (1 << PORTD2);
  PCICR |= (1<<PCIE2);
  PCMSK2 |= (1<<PCINT18);

  EEPROM.get(0, thrmin);
  EEPROM.get(sizeof(thrmin), thrmax);
  sei();
}

void loop()
{
  if (Serial.available() > 0)
  {
    char command = Serial.read();
    long wait = 0;
    char eject = 0;
    int i = 0;
    long newperiod = 0;
    int newnum = 0;
    switch (command)
    {
      case 't' :
        wait = 0;
        eject = 0;
        while (Serial.peek() == -1)
        {
          if (wait >= MAX_WAIT)
          {
            eject = 1;
            break;
          }
          wait++;
        }
        if (eject == 0)
        {
          command = Serial.read();
          switch (command)
          {
            case 'o' :
              set(THR_MAX);
              break;
            case 'c' :
              set(THR_MIN);
              break;
          }
          command = 0;
        }
        break;
      case '?' :
        whichhelp();
        break;
      case 'h' :
        help();
        break;
      /*case 'a' :
        ayuda();
        break;*/
    }
  }
}

double timetorpm(int x)
{
  if(x > 0)
  {
    return 1.0 / ((1.0 / ((double) CLK_HZ / (double) PRE_SCL_RPM)) * (double) x) * 60.0;
  }
  else
  {
   return 0; 
  }
}
double maffunc(unsigned short int mafarg)
{
  double mafd = (double) mafarg;
  return (1.06881684773019e-09 * pow(mafd, 4)) - (1.72720057772642e-06 * pow(mafd, 3)) + (1.35629802010136e-03 * pow(mafd, 2)) - (3.82406978824069e-01 * mafd) + (3.84639348245117e+01);
}
#define thradj(x) (((double) x - (double) thrmin) / ((double) thrmax - (double) thrmin))


ISR(PCINT2_vect) //VR interrupt
{
  if (!started) //Now start it
  {
    TCCR1B |= (1 << CS11) | (1 << CS10);
    started = 1;
  }
  else
  {
    seq++;
    seq %= EDGES;
    teethtime += TCNT1;
    TCNT1 = 0;
    if (seq == 0)
    {
      rpmavgn++;
      //rpmavg = avgaddnew(rpmavg, teethtime, rpmavgn);
      rpmsum += teethtime;
      teethtime = 0;
    }
  }
}

/*int avgaddnew(int oldavg, int newpoint, long newn)
{
  if (newn == 1)
  {
    return newpoint;
  }
  else if (newn > 1)
  {
    int newavg = oldavg;
    newavg -= oldavg / newn;
    newavg += newpoint / newn;
    return newavg;
  }
  else
  {
    return 0;
  }
}*/

ISR(TIMER1_OVF_vect) //If timer1 overflows, engine is not running
{
  TCNT1 = 0;
  started = 0;
  rpmavg = 0;
  rpmavgn = 0;
}

ISR (TIMER0_COMPA_vect) //Timer0 interrupt, sends data every... 2ms?
{
  if (periodms > 0)
  {
    over++;
  }
  else
  {
    return;
  }
  if (over >= periodms)
  {
    over = 0;
    Serial.print("{");
    Serial.print(timetorpm(rpmsum / rpmavgn));
    Serial.print(",");
    Serial.print(maffunc(mafsum / mafavgn));
    Serial.print(",");
    Serial.print(thradj(thrsum / thravgn));
    Serial.println("},");
    thravg = 0;
    thrsum = 0;
    thravgn = 0;
    mafavg = 0;
    mafsum = 0;
    mafavgn = 0;
    rpmsum = 0;
    rpmavg = 0;
    rpmavgn = 0;
  }
}

ISR (TIMER2_COMPA_vect) //Interrupt from TIMER2, takes data every so often (2 ms?)
{
  //thravg = avgaddnew(thravg, analogRead(A1), thravgn);
  thrsum += analogRead(A1);
  thravgn++;
  //mafavg = avgaddnew(mafavg, analogRead(A2), mafavgn);
  mafsum += analogRead(A2);
  mafavgn++;
}

void whichhelp()
{
  Serial.println("For help in English, send me \"h\".");
  Serial.println("Para ayuda en Espanol, mandame \"a\". No puedo usar tildes, perdoname.");
}
void help()
{
  Serial.println("ask ryan I aint got enough mem");
}
void set(char openclosed)
{
  if (openclosed == THR_MIN)
  {
    thrmin = analogRead(A1);
    EEPROM.put(0, thrmin);
  }
  else if (openclosed == THR_MAX)
  {
    thrmax = analogRead(A1);
    EEPROM.put(sizeof(thrmin), thrmax);
  }
  else
  {
    return;
  }
}

