bool on = false;

void setup(){
    TCCR1A = 0; // Reset to 0
    TCCR1B = 0;

    TIMSK1 |= (1 << OCIE1A);  // Pin compare, enable interrupt
    TCCR1B |= (1 << WGM12);   // Clear Timer on Compare mode
    TCCR1B |= (1 << CS12);    // Prescaler : 8
    OCR1A = 20;               // Output Compare Flag

    pinMode(13, OUTPUT);
}

void loop(){
    delay(1000);
}

ISR(TIMER1_COMPA_vect){    // Interrupt routine
  on = on ^ true;
  digitalWrite(13, on);   // Test
}
