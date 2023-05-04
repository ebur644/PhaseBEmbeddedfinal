#define BUTTON_1 2
#define BUTTON_2 3
#define GREEN_LED 4
#define RED_LED 5


#define DATA 9   //74HC595  pin 8 DS
#define LATCH 8  //74HC595  pin 9 STCP
#define CLOCK 7  //74HC595  pin 10 SHCP
#define BUZZER 6

#define DIGIT_4 10
#define DIGIT_3 11
#define DIGIT_2 12
#define DIGIT_1 13

unsigned char table[] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, 0x00 };

unsigned char setnum = 90;
unsigned char gcount = setnum;

unsigned int lastButtonState = 0;     // variable for reading the pushbutton status
unsigned int currentButtonState = 0;  // variable for reading the pushbutton status
unsigned int buttonState = 0;         // variable for reading the pushbutton status

volatile unsigned char isr_1_flag = 0;
volatile unsigned char isr_2_flag = 0;
volatile unsigned char isr_3_flag = 0;

volatile bool gpause = 0;

#define LED 13

volatile unsigned char gISRFlag2   = 0;

unsigned int gReloadTimer1 = 100;   // corresponds to 0.4ms

#define BUFF_SIZE 20
char  gIncomingChar;
char  gCommsMsgBuff[BUFF_SIZE];
int   iBuff = 0;
byte  gPackageFlag = 0;
byte  gProcessDataFlag = 0;

char compareArray(char a[], char b[], int size)
{
  int i;
  char result = 1;  // default: the arrays are equal
  
  for(i = 0; i<size; i++)
  {
    if(a[i]!=b[i])
    {
      result = 0;
      break;
    }
  }
  return result;
}

void setup() {
  pinMode(LED, OUTPUT);
  
  Serial.begin(9600);

  // LEDs Pins
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  // Button Pins
  pinMode(BUTTON_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_1), Button_1_ISR, CHANGE);
  pinMode(BUTTON_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_2), Button_2_ISR, CHANGE);

  // 7-Seg Display
  pinMode(DIGIT_1, OUTPUT);
  pinMode(DIGIT_2, OUTPUT);
  pinMode(DIGIT_3, OUTPUT);
  pinMode(DIGIT_4, OUTPUT);

  // Shift Register Pins
  pinMode(LATCH, OUTPUT);
  pinMode(CLOCK, OUTPUT);
  pinMode(DATA, OUTPUT);

  //disp_on();

  // initialize timer1
  noInterrupts();  // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 62500;            // compare match register 16MHz/256
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts

   // initialize timer2
 noInterrupts();  // disable all interrupts
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  OCR2A = 65535;            // compare match register 
  TCCR2B |= (1 << WGM22);   // CTC mode
  TCCR2B |= (1 << CS22);    // 256 prescaler
  TIMSK2 |= (1 << OCIE2A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}


void Display1(unsigned char num) {
  disp_off();
  digitalWrite(DIGIT_4, LOW);
  digitalWrite(LATCH, LOW);
  shiftOut(DATA, CLOCK, MSBFIRST, table[num]);
  digitalWrite(LATCH, HIGH);
}
void Display2(unsigned char num) {
  disp_off();
  digitalWrite(DIGIT_3, LOW);
  digitalWrite(LATCH, LOW);
  shiftOut(DATA, CLOCK, MSBFIRST, table[num]);
  digitalWrite(LATCH, HIGH);
}
void Display3(unsigned char num) {
  disp_off();
  digitalWrite(DIGIT_2, LOW);
  digitalWrite(LATCH, LOW);
  shiftOut(DATA, CLOCK, MSBFIRST, table[num]);
  digitalWrite(LATCH, HIGH);
}
void Display4(unsigned char num) {
  disp_off();
  digitalWrite(DIGIT_1, LOW);
  digitalWrite(LATCH, LOW);
  shiftOut(DATA, CLOCK, MSBFIRST, table[num]);
  digitalWrite(LATCH, HIGH);
}

void Display_Numbers() { //switches between the didgits quickly
  Display1(gcount % 10);
  delay(5);
  Display2(gcount / 10 % 6);
  delay(5);
  Display3(gcount / 60 % 10);
  delay(5);
  Display4(gcount / 600 % 10);
  delay(5);
}


void disp_on() {//turns all didgits on
  digitalWrite(DIGIT_1, LOW);
  digitalWrite(DIGIT_2, LOW);
  digitalWrite(DIGIT_3, LOW);
  digitalWrite(DIGIT_4, LOW);
}
void disp_off() {//turns all digits off
  digitalWrite(DIGIT_1, HIGH);
  digitalWrite(DIGIT_2, HIGH);
  digitalWrite(DIGIT_3, HIGH);
  digitalWrite(DIGIT_4, HIGH);
}

void Button_1_ISR() {//button 1 isr
  // Set ISR Flag
  isr_1_flag = 1;
}

void Button_2_ISR() {//button 2 isr
  // Set ISR Flag
  isr_2_flag = 1;
}

ISR(TIMER2_COMPA_vect)  // Timer1 interrupt service routine (ISR)
{
  if(Serial.available()>0)//communication with iot
  {
    gISRFlag2 = 1;
  }
}



ISR(TIMER1_COMPA_vect)  // timer compare interrupt service routine
{
  // Set ISR Flag
  isr_3_flag = 1;
}

void activeBuzzer() {// turns on the timer
  unsigned char i;
  unsigned char sleepTime = 1;  // ms

  for (i = 0; i < 100; i++) {
    digitalWrite(BUZZER, HIGH);
    delay(sleepTime);  //wait for 1ms
    digitalWrite(BUZZER, LOW);
    delay(sleepTime);  //wait for 1ms
  }
}

void loop() {
  Display_Numbers();
  lastButtonState = currentButtonState;        // save the last state
  currentButtonState = digitalRead(BUTTON_2);  // read new state

  if (lastButtonState == HIGH && currentButtonState == LOW) {//button debounce
    if (gpause == 0) {
      gpause = 1;
      digitalWrite(GREEN_LED, LOW);
      TCCR1B |= (1 << CS12);  // 256 prescaler
      TIMSK1 |= (1 << OCIE1A);
    } else {
      gpause = 0;
      digitalWrite(GREEN_LED, HIGH);
      TCCR1B &= 0b11111000;  // stop clock
      TIMSK1 = 0;            // cancel clock timer interrupt
    }
  }


  if (gpause == 1) {
    // Attend Button 1
    if (isr_1_flag == 1) {
      // Reset ISR Flag
      isr_1_flag = 0;

      // Code
      buttonState = digitalRead(BUTTON_1);
      digitalWrite(RED_LED, buttonState);
      gcount++;
    }

    // Attend Button 2
    if (isr_2_flag == 1) {
      // Reset ISR Flag
      isr_2_flag = 0;
    }

    // Attend 7-Seg Display
    if (isr_3_flag == 1 & gcount != 0) {
      // Reset ISR Flag
      isr_3_flag = 0;

      // Code
      gcount--;
    }
    if (gcount == 0) {
      // Reset ISR Flag
      activeBuzzer();
    }
  }
  if(gpause == 0 && gcount == 0){// resets the counter
    gcount = setnum;
  }
char  auxMsgBuff[BUFF_SIZE];
  int auxCount = 0;
  unsigned char auxDigit = '0';
  
  // Attend Timer1 flag - receive commands through serial
  if(gISRFlag2 == 1)
  {    
    // Reset ISR Flag
    gISRFlag2 = 0;

    // Read serial
    gIncomingChar = Serial.read();

    // If normal character from package
    if(gPackageFlag == 1)
    {
      gCommsMsgBuff[iBuff] = gIncomingChar;
      iBuff++;

      // Safety mechanism in case "\n" is never sent
      if(iBuff == BUFF_SIZE)
      {
        gPackageFlag = 0;
        gProcessDataFlag = 1;
      }
    }

    // If start of the package
    if(gIncomingChar == '$')
    {    
      gPackageFlag = 1;  // Signal start of package
      
      // Clear Buffer
      for(int i=0; i<BUFF_SIZE; i++)
      {
        gCommsMsgBuff[i] = 0;
      }

      // set gCommsMsgBuff Index to zero
      iBuff = 0;
    }

    // If end of package
    if( (gIncomingChar == '\n') && (gPackageFlag == 1) )
    {
      // Signal end of package
      gPackageFlag = 0;
      gProcessDataFlag = 1;
    }
  }

  // Process serial commands
  if(gProcessDataFlag == 1)
  {
    gProcessDataFlag = 0;
    
    if(compareArray(gCommsMsgBuff, "STR", 3) == 1)
    {
      // Start timer function
      digitalWrite(LED, HIGH);
      Serial.print("start\n");
    }
  
    if(compareArray(gCommsMsgBuff, "STP", 3) == 1)
    {
      // Stop timer function
      digitalWrite(LED, LOW);
      Serial.print("stop\n");
    }

    if(compareArray(gCommsMsgBuff, "GET", 3) == 1)
    {
      // Send clock status
      Serial.print("$00:01\n");
    }
    
  }
}  

