#define F_CPU 8000000UL
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


#define LCD_Data_Dir DDRB		/* Define LCD data port direction */
#define LCD_Command_Dir DDRD		/* Define LCD command port direction register */
#define LCD_Data_Port PORTB		/* Define LCD data port */
#define LCD_Command_Port PORTD		/* Define LCD data port */
#define RS PD4				/* Define Register Select (data/command reg.)pin */
#define RW PD5				/* Define Read/Write signal pin */
#define EN PD6				/* Define Enable signal pin */

#define BUTTON_PORT PORTA
#define BUTTON_PIN PINA
#define BUTTON_DDR DDRA
#define HOUR_BUTTON_MAX PA0
#define MINUTE_BUTTON_MAX PA1
#define HOUR_BUTTON_MIN PA2
#define MINUTE_BUTTON_MIN PA3
#define SWITCH PA4

#define LED_PIN PD7
#define BUZZER_PIN PD2 

#define IR_SENSOR_PIN    PD3
#define OBSTACLE_DETECTED_THRESHOLD  10




volatile uint8_t hours = 12;    // Initial hours
volatile uint8_t minutes = 15;   // Initial minutes
volatile uint8_t seconds = 55;   // Initial seconds

volatile uint8_t alarm_hours = 12;
volatile uint8_t alarm_minutes = 31;

volatile uint8_t alarm_triggered = 0;
volatile uint8_t medicineTaken = 0;
volatile uint8_t massageSend = 1;
volatile uint8_t motorRotationDone = 1;

char set[10];


void Button_Init();
void UpdateAlarmOrTime();
void DisplayTime();

void CheckAlarms();
void DisplayAlarms();

void initIRSensor(void);
void checkMedicine(void);

void AlarmFunction();

// LCD Function prototypes
void LCD_Char (unsigned char char_data);
void LCD_Init (void);
void LCD_String (char *str);
void LCD_String_xy (char row, char pos, char *str);
void LCD_Clear();

// GSM Function prototypes
void UART_init(long USART_BAUDRATE);
unsigned char UART_RxChar(void);
void UART_TxChar(char ch);
void UART_SendString(char *str);



int main() {
    LCD_Init();
    Button_Init(); 			// Initialize buttons
    initIRSensor();
    
    DDRA &= ~(1 << SWITCH);
    PORTA |= (1 << SWITCH);
    DDRC |= (1 << PC4) | (1 << PC5) | (1 << PC6) | (1 << PC7); // Set PC4-PC7 as outputs
    int period = 5;

    // Configure Timer1 to generate a 1-second interrupt
    TCCR1B |= (1 << CS12) | (1 << WGM12); // Prescaler 256 and CTC mode
    OCR1A = 31249; 			// Compare value for 1 second at 8MHz
    TIMSK |= (1 << OCIE1A); 		// Enable Timer1A compare match interrupt
    sei(); 				// Enable global interrupts

    while (1) {
        CheckAlarms();
        UpdateAlarmOrTime();
        DisplayTime();
        DisplayAlarms();
        checkMedicine();
        AlarmFunction();
        
        if (PINA & (1 << SWITCH)) {
	    // If the switch is ON, display "Set Alarm"
	    strcpy(set, "Alarm");
	} else {
	    // If the switch is OFF, display "Set Time"
	    strcpy(set, "Time ");
	}
        
        if (!motorRotationDone && !medicineTaken) {
            // Perform a 64 steps rotation
            for (int i = 0; i < 64; i++) {
                PORTC = 0x10;
                _delay_ms(period);
                PORTC = 0x30;
                _delay_ms(period);
                PORTC = 0x20;
                _delay_ms(period);
                PORTC = 0x60;
                _delay_ms(period);
                PORTC = 0x40;
                _delay_ms(period);
                PORTC = 0xC0;
                _delay_ms(period);
                PORTC = 0x80;
                _delay_ms(period);
                PORTC = 0x90;
                _delay_ms(period);
            }
            motorRotationDone = 1;
        } 
        
        if (alarm_triggered && !massageSend) {
		UART_init(9600);		// Initialize UART communication with a baud rate of 9600
		UART_SendString("AT\r\n");	// Send AT command to the GSM module
		_delay_ms(3000);
		UART_SendString("ATE0\r\n");	// Disable command echo
		_delay_ms(3000);
		UART_SendString("AT+CMGF=1\r\n");// Set SMS text mode
		_delay_ms(3000);
		UART_SendString("AT+CMGS=\"+94778797936\"\r\n");// Set recipient phone number
		_delay_ms(3000);
		UART_SendString("It's time to take your medicine");// Send SMS text
		UART_TxChar(26);		// Send Ctrl+Z to indicate end of SMS
		massageSend = 1;
		}
		
    }
    return 0;
}



// Initialize UART with the specified baud rate
void UART_init(long USART_BAUDRATE){
    UCSRB |= (1 << RXEN) | (1 << TXEN); 	// Turn on transmission and reception
    UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); // Use 8-bit character sizes
    UBRRL = BAUD_PRESCALE; 			// Load lower 8-bits of the baud rate value
    UBRRH = (BAUD_PRESCALE >> 8); 		// Load upper 8-bits
}

// Receive a character from UART
unsigned char UART_RxChar(void){
    while ((UCSRA & (1 << RXC)) == 0); 		// Wait till data is received
    return(UDR); 				// Return the received byte
}

// Transmit a character through UART
void UART_TxChar(char ch){
    while (!(UCSRA & (1<<UDRE))); 		// Wait for empty transmit buffer
    UDR = ch;
}

// Send a string through UART
void UART_SendString(char *str){
    unsigned char j=0;
    while (str[j] != 0){ 			// Send string till null terminator
        UART_TxChar(str[j]);
        j++;
    }
}



// LCD
void LCD_Command(unsigned char cmnd)
{
	LCD_Data_Port= cmnd;
	LCD_Command_Port &= ~(1<<RS);	// RS=0 command reg.
	LCD_Command_Port &= ~(1<<RW);	// RW=0 Write operation
	LCD_Command_Port |= (1<<EN);	// Enable pulse
	_delay_us(1);
	LCD_Command_Port &= ~(1<<EN);
	_delay_ms(3);
}

void LCD_Char (unsigned char char_data)	// LCD data write function
{
	LCD_Data_Port= char_data;
	LCD_Command_Port |= (1<<RS);	// RS=1 Data reg.
	LCD_Command_Port &= ~(1<<RW);	// RW=0 write operation
	LCD_Command_Port |= (1<<EN);	// Enable Pulse
	_delay_us(1);
	LCD_Command_Port &= ~(1<<EN);
	_delay_ms(1);
}

void LCD_Init (void)			// LCD Initialize function
{
	LCD_Command_Dir = 0xFF;		// Make LCD command port direction as o/p
	LCD_Data_Dir = 0xFF;		// Make LCD data port direction as o/p
	_delay_ms(50);			// LCD Power ON delay always >15ms
	
	LCD_Command (0x38);		// Initialization of 16X2 LCD in 8bit mode
	LCD_Command (0x0C);		// Display ON Cursor OFF
	LCD_Command (0x06);		// Auto Increment cursor
	LCD_Command (0x01);		// Clear display
	LCD_Command (0x80);		// Cursor at home position
}

void LCD_String (char *str)		// Send string to LCD function
{
	int i;
	for(i=0;str[i]!=0;i++)		// Send each char of string till the NULL
	{
		LCD_Char (str[i]);
	}
}

void LCD_String_xy (char row, char pos, char *str)// Send string to LCD with xy position
{
	if (row == 0 && pos<16)
	LCD_Command((pos & 0x0F)|0x80);	// Command of first row and required position<16
	else if (row == 1 && pos<16)
	LCD_Command((pos & 0x0F)|0xC0);	// Command of first row and required position<16
	LCD_String(str);		// Call LCD string function 
}

void LCD_Clear()
{
	LCD_Command (0x01);		// clear display
	LCD_Command (0x80);		// cursor at home position
}



// Button
void Button_Init() {			// Set HOUR_BUTTON and MINUTE_BUTTON as input with pull-up resistors
    BUTTON_DDR &= ~(1 << HOUR_BUTTON_MAX);
    BUTTON_DDR &= ~(1 << MINUTE_BUTTON_MAX);
    BUTTON_PORT |= (1 << HOUR_BUTTON_MAX);
    BUTTON_PORT |= (1 << MINUTE_BUTTON_MAX);
    
    BUTTON_DDR &= ~(1 << HOUR_BUTTON_MIN);
    BUTTON_DDR &= ~(1 << MINUTE_BUTTON_MIN);
    BUTTON_PORT |= (1 << HOUR_BUTTON_MIN);
    BUTTON_PORT |= (1 << MINUTE_BUTTON_MIN);
}



void UpdateAlarmOrTime() {
    // Check the state of PA4 to determine if you want to set alarms or time
    if (PINA & (1 << SWITCH)) {
    
        if (!(BUTTON_PIN & (1 << HOUR_BUTTON_MAX))) {// Hour button is pressed
        _delay_ms(50);				// Debounce delay
        if (!(BUTTON_PIN & (1 << HOUR_BUTTON_MAX))) {
            alarm_hours = (alarm_hours + 1) % 24;		// Increment hours
            _delay_ms(500);
        }
    }

    if (!(BUTTON_PIN & (1 << MINUTE_BUTTON_MAX))) { // Minute button is pressed
        _delay_ms(50); 				// Debounce delay
        if (!(BUTTON_PIN & (1 << MINUTE_BUTTON_MAX))) {
            alarm_minutes = (alarm_minutes + 1) % 60;	// Increment minutes
            _delay_ms(500); 			// Button press delay
        }
    }
    
    
    if (!(BUTTON_PIN & (1 << HOUR_BUTTON_MIN))) {// Hour button is pressed
        _delay_ms(50);				// Debounce delay
        if (!(BUTTON_PIN & (1 << HOUR_BUTTON_MIN))) {
            alarm_hours = (alarm_hours - 1) % 24;		// Decrement hours
            _delay_ms(500);
        }
    }

    if (!(BUTTON_PIN & (1 << MINUTE_BUTTON_MIN))) { // Minute button is pressed
        _delay_ms(50); 				// Debounce delay
        if (!(BUTTON_PIN & (1 << MINUTE_BUTTON_MIN))) {
            alarm_minutes = (alarm_minutes - 1) % 60;	// Decrement minutes
            _delay_ms(500); 			// Button press delay
        }
    }
    
    
    } else {
    
        if (!(BUTTON_PIN & (1 << HOUR_BUTTON_MAX))) {// Hour button is pressed
        _delay_ms(50);				// Debounce delay
        if (!(BUTTON_PIN & (1 << HOUR_BUTTON_MAX))) {
            hours = (hours + 1) % 24;		// Increment hours
            _delay_ms(500);
        }
    }

    if (!(BUTTON_PIN & (1 << MINUTE_BUTTON_MAX))) { // Minute button is pressed
        _delay_ms(50); 				// Debounce delay
        if (!(BUTTON_PIN & (1 << MINUTE_BUTTON_MAX))) {
            minutes = (minutes + 1) % 60;	// Increment minutes
            _delay_ms(500); 			// Button press delay
        }
    }
    
    
    if (!(BUTTON_PIN & (1 << HOUR_BUTTON_MIN))) {// Hour button is pressed
        _delay_ms(50);				// Debounce delay
        if (!(BUTTON_PIN & (1 << HOUR_BUTTON_MIN))) {
            hours = (hours - 1) % 24;		// Decrement hours
            _delay_ms(500);
        }
    }

    if (!(BUTTON_PIN & (1 << MINUTE_BUTTON_MIN))) { // Minute button is pressed
        _delay_ms(50); 				// Debounce delay
        if (!(BUTTON_PIN & (1 << MINUTE_BUTTON_MIN))) {
            minutes = (minutes - 1) % 60;	// Decrement minutes
            _delay_ms(500); 			// Button press delay
        }
    }
        
    }
}



// Function to display the time
void DisplayTime() {
    char timeStr[30];			// Display hours and minutes on LCD
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d   %s", hours, minutes, seconds, set);
    LCD_String_xy(0, 0, timeStr);
}



//CheckAlarms
void CheckAlarms() {			// Function to check if an alarm is triggered
    if (hours == alarm_hours && minutes == alarm_minutes) {
        alarm_triggered = 1;
        AlarmFunction();		//Call the function for Alarm
    }
}

void DisplayAlarms() {			// Function to display alarms on the LCD
    char alarmStr[16];
    snprintf(alarmStr, sizeof(alarmStr), "Alarm: %02d:%02d", alarm_hours, alarm_minutes);
    LCD_String_xy(1, 0, alarmStr);
}



// IR Sensor
void initIRSensor(void) {
    DDRD &= ~(1 << IR_SENSOR_PIN);	// Set IR sensor pin as input
    PORTD |= (1 << IR_SENSOR_PIN);	// Enable pull-up resistor for IR sensor
    }
    
void checkMedicine(void){
    if (PIND & (1 << IR_SENSOR_PIN)) {
        medicineTaken = 1; 			// No obstacle detected
    } else {
        medicineTaken = 0;			// Obstacle detected
    }
}




void AlarmFunction() {
    static uint8_t alarmActive = 0;  // Flag to indicate if alarm is active

    if (alarm_triggered) {
        if (!alarmActive) {
            alarmActive = 1;
            massageSend = 0;
            motorRotationDone = 0;
            PORTD |= (1 << LED_PIN);  // Turn on the LED
            PORTD |= (1 << BUZZER_PIN);  // Turn on the buzzer
        } else if (seconds >= 10) {  // Check if 10 seconds have passed
            PORTD &= ~(1 << BUZZER_PIN);  // Turn off the buzzer after 30 seconds
        }
    } else {
        alarmActive = 0;  // Reset the alarm active flag when the alarm is not triggered
        PORTD &= ~(1 << LED_PIN);  // Turn off the LED
        PORTD &= ~(1 << BUZZER_PIN);  // Turn off the buzzer
    }

    if (medicineTaken) {
        PORTD &= ~(1 << LED_PIN);  // Turn off the LED if medicine is taken
        PORTD &= ~(1 << BUZZER_PIN);  // Turn off the buzzer if medicine is taken
    }
}




ISR(TIMER1_COMPA_vect) {		// Timer1A compare match interrupt service routine
    seconds = (seconds + 1) % 60;	// Called every 1 second

    if (seconds == 0) {			// If seconds reach 60, increment minutes
        minutes = (minutes + 1) % 60;

        if (minutes == 0) {		// If minutes reach 60, increment hours
            hours = (hours + 1) % 24;   
            }
        }
}
