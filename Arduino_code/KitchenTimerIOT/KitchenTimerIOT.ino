#define MAX_INPUT_LENGTH 20 // Maximum length of the input string

volatile bool new_input = false; // Flag to indicate new input has been received
volatile char input_buffer[MAX_INPUT_LENGTH]; // Buffer to store the input string
volatile uint8_t input_index = 0; // Index to keep track of the current character being received
volatile bool timer_flag = false; // Flag to indicate timer has expired

void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 baud rate
  
  // Configure Timer 2 to trigger an interrupt every 0.4ms
  TCCR2A = 0;
  TCCR2B = (1 << CS21) | (1 << CS22); // Set prescaler to 256
  OCR2A = 156; // Set compare match value to achieve 0.4ms interval
  TIMSK2 = (1 << OCIE2A); // Enable Timer 2 compare match interrupt
  
  sei(); // Enable global interrupts
}

void loop() {
  if (new_input) { // If new input has been received
    char input_string[MAX_INPUT_LENGTH + 1]; // Create a temporary buffer to hold the input string
    strncpy(input_string, input_buffer, MAX_INPUT_LENGTH); // Copy the input buffer to the temporary buffer
    input_string[MAX_INPUT_LENGTH] = '\0'; // Add null terminator to the end of the string
    
    processInputString(input_string); // Process the input string
    new_input = false; // Reset the new input flag
  }
}

void processInputString(char* input) {
  if (strcmp(input, "$GET\n") == 0) { // If the input string is "$GET\n"
    Serial.println("GET command received"); // Process the "GET" command
  } else if (strcmp(input, "$STR\n") == 0) { // If the input string is "$STR\n"
    Serial.println("STR command received"); // Process the "STR" command
  } else if (strcmp(input, "$STP\n") == 0) { // If the input string is "$STP\n"
    Serial.println("STP command received"); // Process the "STP" command
  } else { // If the input string is not recognized
    Serial.println("Invalid command"); // Print an error message
  }
}

ISR(TIMER2_COMPA_vect) { // Interrupt service routine to handle Timer 2 compare match interrupt
  static uint8_t timer_count = 0;
  
  if (timer_count >= 2) { // Check for "$" every 0.4ms (2 x 0.2ms = 0.4ms)
    timer_count = 0;
    timer_flag = true;
  } else {
    timer_count++;
  }
}

void serialEvent() { // Interrupt service routine to handle new input
  while (Serial.available()) { // Read all available characters
    char c = Serial.read(); // Read the next character
    
    if (c == '$') { // If the character is the start of a new input string
      input_index = 0; // Reset the input index
      input_buffer[input_index] = c; // Store the character in the input buffer
      input_index++; // Increment the input index
    } else if (c == '\n') { // If the character is the end of the input string
      input_buffer[input_index] = c; // Store the character
        input_index++; // Increment the input index
  
  if (input_index >= MAX_INPUT_LENGTH) { // If the input string is too long
    input_index = 0; // Reset the input index
  } else {
    new_input = true; // Set the new input flag
  }
} else { // If the character is part of the input string
  input_buffer[input_index] = c; // Store the character in the input buffer
  input_index++; // Increment the input index
  
  if (input_index >= MAX_INPUT_LENGTH) { // If the input string is too long
    input_index = 0; // Reset the input index
  }
}
}
}