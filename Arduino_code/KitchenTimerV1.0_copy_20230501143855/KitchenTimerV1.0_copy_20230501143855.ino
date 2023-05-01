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
unsigned char count = setnum;

unsigned int lastButtonState = 0;     // variable for reading the pushbutton status
unsigned int currentButtonState = 0;  // variable for reading the pushbutton status
unsigned int buttonState = 0;         // variable for reading the pushbutton status

volatile unsigned char isr_1_flag = 0;
volatile unsigned char isr_2_flag = 0;
volatile unsigned char isr_3_flag = 0;

volatile bool pause = 0;

void setup() {

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

void Display_Numbers() {
  Display1(count % 10);
  delay(5);
  Display2(count / 10 % 6);
  delay(5);
  Display3(count / 60 % 10);
  delay(5);
  Display4(count / 600 % 10);
  delay(5);
}


void disp_on() {
  digitalWrite(DIGIT_1, LOW);
  digitalWrite(DIGIT_2, LOW);
  digitalWrite(DIGIT_3, LOW);
  digitalWrite(DIGIT_4, LOW);
}
void disp_off() {
  digitalWrite(DIGIT_1, HIGH);
  digitalWrite(DIGIT_2, HIGH);
  digitalWrite(DIGIT_3, HIGH);
  digitalWrite(DIGIT_4, HIGH);
}

void Button_1_ISR() {
  // Set ISR Flag
  isr_1_flag = 1;
}

void Button_2_ISR() {
  // Set ISR Flag
  isr_2_flag = 1;
}



ISR(TIMER1_COMPA_vect)  // timer compare interrupt service routine
{
  // Set ISR Flag
  isr_3_flag = 1;
}

void activeBuzzer() {
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
  // Combination of Pooling and Interrupt
  lastButtonState = currentButtonState;        // save the last state
  currentButtonState = digitalRead(BUTTON_2);  // read new state

  if (lastButtonState == HIGH && currentButtonState == LOW) {
    if (pause == 0) {
      pause = 1;
      digitalWrite(GREEN_LED, LOW);
      TCCR1B |= (1 << CS12);  // 256 prescaler
      TIMSK1 |= (1 << OCIE1A);
    } else {
      pause = 0;
      digitalWrite(GREEN_LED, HIGH);
      TCCR1B &= 0b11111000;  // stop clock
      TIMSK1 = 0;            // cancel clock timer interrupt
    }
  }


  if (pause == 1) {
    // Attend Button 1
    if (isr_1_flag == 1) {
      // Reset ISR Flag
      isr_1_flag = 0;

      // Code
      buttonState = digitalRead(BUTTON_1);
      digitalWrite(RED_LED, buttonState);
      count++;
    }

    // Attend Button 2
    if (isr_2_flag == 1) {
      // Reset ISR Flag
      isr_2_flag = 0;
    }

    // Attend 7-Seg Display
    if (isr_3_flag == 1 & count != 0) {
      // Reset ISR Flag
      isr_3_flag = 0;

      // Code
      count--;
    }
    if (count == 0) {
      // Reset ISR Flag
      activeBuzzer();
      //count = setnum;
    }
  }
  if(pause == 0 && count == 0){
    count = setnum;
  }
}
