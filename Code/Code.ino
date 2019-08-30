/*
    Filename: Code.ino
    Author: The real Spider Tronix 2019
    Purpose: First years' uCON  workshop (2019)
 */

//REQUIRED HEADERS
#include <avr/interrupt.h>
#include <avr/io.h>

#define OCR1_2ms 3999
#define OCR1_1ms 1800
#define left_servo_bottom_angle 10
#define left_servo_top_angle 80 // Constants
#define right_servo_bottom_angle 70
#define right_servo_top_angle 10
#define middle_servo_left_angle 0
#define middle_servo_right_angle 30

//CONSTANS FOR BOT DIRECTIONS
#define DIR_NONE 0
#define DIR_FRONT 1
#define DIR_LEFT 2
#define DIR_RIGHT 3

volatile float time0 = 0;
float duty_time; ///(min - 1ms, max -2ms)
////////////////
// UART class //
////////////////
class usart
{
public:
    void init(unsigned long baud = 9600)
    {
        if (baud == 9600)
        {
            UCSR0A = 0X00;
            UCSR0B = (1 << RXEN0) | (1 << TXEN0);
            UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
            UBRR0 = 103; //Set baud rate to 9600
        }
    }
    void print(String data)
    {
        for (int i = 0; data[i] != '\0'; i++)
        {
            while (!(UCSR0A & (1 << UDRE0)))
                ;
            UDR0 = data[i];
        }
    }
    char read()
    {
        if ( (UCSR0A & (1 << RXC0)) )
        {
          char temp=UDR0;
          if(temp!='\n' && temp!='\r')
            return temp;
        }
          
        
        return 0;
    }
};
usart serial;

ISR(TIMER0_OVF_vect)
{
    time0 += 0.128; // Calculate time (min - 0ms, max - 2ms)
    if (time0 >= 20)
    {
        PORTD |= 1 << 6; // At 20 ms reset time0 to 0 and pull the pin back high
        time0 = 0;
    }
    else if (time0 >= duty_time)
    {
        PORTD = 0; // At time0 = duty_time make pin low for getting desired duty cycle
    }
}
////////////////
// servo class //
////////////////
class servo
{
    int pin;

public:
    // Initialize servo at given pin
    void begin(int select_pin)
    {
        pin = select_pin;
        if (pin == 9)
        {
            /*Set pre-scaler of 8 with Fast PWM (Mode 14 i.e TOP value as ICR1)  non-inverting mode */
            DDRB |= 1 << PINB1;
            TCCR1A |= (1 << WGM11) | (1 << COM1A1);
            TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11);
            ICR1 = 39999; // Set pwm period as 20ms
        }
        if (pin == 6)
        {
            DDRD |= 1 << 6;
        }
        if (pin == 10)
        {
            /*Set pre-scaler of 8 with Fast PWM (Mode 14 i.e TOP value as ICR1)  non-inverting mode */
            DDRB |= 1 << PINB2;
            TCCR1A |= (1 << WGM11) | (1 << COM1B1);
            TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11);
            ICR1 = 39999; // Set pwm period as 20ms
        }
    }
    // Write the servo's angle
    void write(float angle)
    {
        if (pin == 9)
            OCR1A = map(angle, 0, 180, OCR1_1ms, OCR1_2ms); // Map angle to OCR1 value
        if (pin == 6)
        {
            duty_time = 1 + (float)(angle / 180);
            TCCR0B = 1 << CS01; //8 bit prescale gives 0.128ms per overflow
        }
        if (pin == 10)
            OCR1B = map(angle, 0, 180, OCR1_1ms, OCR1_2ms); // Map angle to OCR1B
    }
};
servo left_servo;
servo right_servo; // Model servos as objects of type servo
servo middle_servo;

/*
 * timer0 compare match a
 */
volatile int timer0_counter = 0;

/*
overflows counts number of overflows of Timer2
*/
volatile int overflows = 0;
ISR(TIMER2_OVF_vect)
{
    ++overflows;
}
/*
Using timer 2 to cause a delay of a given time
*/
void delayinms(int time_)
{
    overflows = 0;

    TCCR2B = 1 << CS21 | 1 << CS20;    //32 bit prescale gives 0.512ms per overflow
    while (overflows * 0.512 <= time_) // wait till count of overflows equals time_
        ;
    TCCR2B = 0; // turn timer off after use
}

// Enable Overflow interrupt
void timer_init()
{
    TIMSK2 = 1 << TOIE2;
    TIMSK0 = 1 << TOIE0;
}
////////////////
// BOT class //
////////////////
class bot
{
public:
    // Initialize position of legs for forward motion
    void move_forward_init()
    {
        middle_servo.write(middle_servo_right_angle);
        left_servo.write(left_servo_bottom_angle);
        right_servo.write(right_servo_top_angle);
        delayinms(500);
    }
    // Complete one full cycle of forward motion gait
    void move_forward()
    {
        middle_servo.write(middle_servo_left_angle);
        delayinms(200);
        left_servo.write(left_servo_top_angle);
        right_servo.write(right_servo_bottom_angle);
        delayinms(500);

        middle_servo.write(middle_servo_right_angle);
        delayinms(200);
        left_servo.write(left_servo_bottom_angle);
        right_servo.write(right_servo_top_angle);
        delayinms(500);
    }
    // Initialize position of legs for left rotation
    void turn_left_init()
    {
        middle_servo.write(middle_servo_right_angle);
        left_servo.write(left_servo_top_angle);
        right_servo.write(right_servo_top_angle);
        delayinms(500);
    }
    // Complete one full cycle of left rotation gait
    void turn_left()
    {
        middle_servo.write(middle_servo_left_angle);
        left_servo.write(left_servo_bottom_angle);
        right_servo.write(right_servo_bottom_angle);
        delayinms(500);
        middle_servo.write(middle_servo_right_angle);
        left_servo.write(left_servo_top_angle);
        right_servo.write(right_servo_top_angle);
        delayinms(500);
    }
    // Initialize position of legs for right rotation
    void turn_right_init()
    {
        middle_servo.write(middle_servo_left_angle);
        left_servo.write(left_servo_top_angle);
        right_servo.write(right_servo_top_angle);
        delayinms(500);
    }
    // Complete one full cycle of right rotation gait
    void turn_right()
    {
        middle_servo.write(middle_servo_right_angle);
        left_servo.write(left_servo_bottom_angle);
        right_servo.write(right_servo_bottom_angle);
        delayinms(500);
        middle_servo.write(middle_servo_left_angle);
        left_servo.write(left_servo_top_angle);
        right_servo.write(right_servo_top_angle);
        delayinms(500);
    }

    /*
        Use: Initialize movement in a particular direction 
        Note: To be called only once when there is a change in the direction of movement
     */
    void move_init(int direction)
    {
        switch (direction)
        {
        case DIR_FRONT:
            move_forward_init();
            break;
        case DIR_LEFT:
            turn_left_init();
            break;
        case DIR_RIGHT:
            turn_right_init();
            break;
        }
    }

    /*
        Use: To be looped infinitely to move the bot
     */
    void move(int direction)
    {
        switch (direction)
        {
        case DIR_FRONT:
            move_forward();
            break;
        case DIR_LEFT:
            turn_left();
            break;
        case DIR_RIGHT:
            turn_right();
            break;
        }
    }
};

int main()
{
    timer_init();
    sei(); //Enable global interrupts
    serial.init(9600);
    left_servo.begin(9);
    middle_servo.begin(6);
    right_servo.begin(10);
    bot spider_bot;
    spider_bot.move_forward_init();
    delayinms(1000);

    int direction = DIR_NONE;

    while (1)
    {
       /* if (USART::bufferReady())
        {
            char temp[2];
            USART::readBuffer(temp); //the buffer is expected to receive '0', '1', '2', '3', etc..
            direction = temp[0] - '0';  //Convert ASCII to absolute value 
            spider_bot.move_init(direction);
            
        }
        USART::sendByte(direction + '0');*/
        char t = serial.read();
        if(t)
        {
          direction = t- '0';
          serial.print("change\n");
          spider_bot.move_init(direction);
        }
        spider_bot.move(direction);
    }
    return 0;
}
