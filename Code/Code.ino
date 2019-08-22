/*
    Filename: Code.ino
    Author: The real Spider Tronix 2019
    Purpose: First years'  workshop (2019)
 */

//REQUIRED HEADERS
#include <avr/interrupt.h>
#include <avr/io.h>
#define OCR1_2ms 3999
#define OCR1_1ms 1800
#define left_servo_bottom_angle 20
#define left_servo_top_angle 150
#define right_servo_bottom_angle 20
#define right_servo_top_angle 150
#define middle_servo_left_angle 20
#define middle_servo_right_angle 150
////////////////
// UART class //
////////////////
class servo
{
public:
    void begin(int pin)
    {
        if (pin == 9)
        {
            /*Set pre-scaler of 8 with Fast PWM (Mode 14 i.e TOP value as ICR1)  non-inverting mode */
            DDRB |= 1 << PINB1;
            TCCR1A |= (1 << WGM11) | (1 << COM1A1);
            TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11);

            ICR1 = 39999; // Set pwm period as 2ms
        }
    }
    void write(int angle, int offset = 800)
    {
        OCR1A = map(angle, 0, 180, OCR1_1ms - offset, OCR1_2ms + offset);
    }
};
servo left_servo;
servo right_servo;
servo middle_servo;
volatile int overflows = 0;
ISR(TIMER2_OVF_vect)
{
    ++overflows;
}
void delayinms(int time_)
{
    overflows = 0;
    TCCR2B = 1 << CS21 | 1 << CS20;    //32 bit prescale gives 0.512ms per overflow
    while (overflows * 0.512 <= time_) // wait till count of overflows equals time_
        ;
    TCCR2B = 0; // turn timer off after use
}
void timer_init()
{
    TIMSK2 = 1 << TOIE2;
    sei();
}
class bot
{
    public:
    void move_forward_init()
    {
        middle_servo.write(middle_servo_right_angle);
        left_servo.write(left_servo_bottom_angle);
        right_servo.write(right_servo_top_angle);
        delayinms(500);
    }
    void move_forward()
    {
        middle_servo.write(middle_servo_left_angle);
        left_servo.write(left_servo_top_angle);
        right_servo.write(right_servo_bottom_angle);
        delay(500);
        middle_servo.write(middle_servo_right_angle);
        left_servo.write(left_servo_bottom_angle);
        right_servo.write(right_servo_top_angle);
        delay(500);
    }
    void turn_left_init()
    {
        middle_servo.write(middle_servo_right_angle);
        left_servo.write(left_servo_top_angle);
        right_servo.write(right_servo_top_angle);
        delayinms(500);
    }
    void turn_left()
    {
        middle_servo.write(middle_servo_left_angle);
        left_servo.write(left_servo_bottom_angle);
        right_servo.write(right_servo_bottom_angle);
        delay(500);
        middle_servo.write(middle_servo_right_angle);
        left_servo.write(left_servo_top_angle);
        right_servo.write(right_servo_top_angle);
        delay(500);
    }
    void turn_right_init()
    {
        middle_servo.write(middle_servo_left_angle);
        left_servo.write(left_servo_top_angle);
        right_servo.write(right_servo_top_angle);
        delayinms(500);
    }
    void turn_right()
    {
        middle_servo.write(middle_servo_right_angle);
        left_servo.write(left_servo_bottom_angle);
        right_servo.write(right_servo_bottom_angle);
        delay(500);
        middle_servo.write(middle_servo_left_angle);
        left_servo.write(left_servo_top_angle);
        right_servo.write(right_servo_top_angle);
        delay(500);
    }
};
class USART
{
public:
    volatile static char USART_DATA_BUFFER[32];
    volatile static uint8_t USART_DATA_INDEX;
    volatile static bool USART_BUFFER_READY;

    /*
        Input: none
        Output: none
        Logic: Initialize serial communication
    */
    static void init(int baud)
    {
        int baudRate = 1000000l / baud - 1;
        UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);               //asynchronous, 8 bit data
        UBRR0 = baudRate;                                     //set baud rate
        UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0); //enable RX and TX
    }

    /*
        Input:  uint8_t data: the data to be sent
        Output: none
        Logic: Send 1 byte of data to the serial monitor
    */
    static void sendByte(uint8_t data)
    {
        while (!(UCSR0A & (1 << UDRE0)))
            ;
        UDR0 = data;
    }

    /*
        Input:  char* data: the data (string) to be sent
        Output: none
        Logic: Send a string to the serial monitor
    */
    static void sendData(const char *data)
    {
        while (*data)
        {
            sendByte(*data);
            data++;
        }
    }

    /*
        Input:  uint8_t* data : pointer to store received byte
        Output: bool : true if a byte was received and put in data
        Logic: By checking the UCSR0A.RXC0 flag availability of data in UDR0 is known,
                if data is available, it is stored and true is returned
        Example call: 
        uint8_t data;
        if (USART::getByte(&data)) {
            //do something with data
        } 
    */
    static bool getByte(uint8_t *data)
    {
        if (UCSR0A & (1 << RXC0))
        {
            *data = UDR0;
            return true;
        }
        return false;
    }

    /*
        Input:  char *dest: memory to be filled with received data (capacity 32 bytes)
        Output: none 
        Logic:  read the received data that is stored in the buffer
        Example call: 
        char data[32];
        if (USART::bufferReady()) USART::readBuffer(data);
    */
    static void readBuffer(char *dest)
    {
        strcpy(dest, USART_DATA_BUFFER);
        USART_DATA_INDEX = 0;
        USART_BUFFER_READY = false;
    }

    static bool bufferReady()
    {
        return USART_BUFFER_READY;
    }

    static void updateBuffer(char data)
    {
        if (data == '\n' || data == '\r' || USART_DATA_INDEX == 31)
        {
            data = '\0';
            USART_BUFFER_READY = true;
        }
        USART_DATA_BUFFER[USART_DATA_INDEX] = data;
        USART_DATA_INDEX++;
    }
};

volatile static char USART::USART_DATA_BUFFER[32] = "";
volatile static uint8_t USART::USART_DATA_INDEX = 0;
volatile static bool USART::USART_BUFFER_READY = false;

/*
    Function name: ISR(USART_RX_vect)
    Logic: Whenever data is available it is appended to the buffer if the buffer has already been read 
 */
ISR(USART_RX_vect)
{
    if (USART::bufferReady() == false)
    {
        char data = UDR0;
        USART::updateBuffer(data);
    }
}

int main()
{
    timer_init();
    sei(); //Enable global interrupts
    USART::init(9600);
    char data[35];
    while (1)
    {
        //USART::sendData("Shyam\n");
        if (USART::bufferReady())
        {
            USART::readBuffer(data);
            USART::sendData(data);
            USART::sendByte('\n');
        }
    }
    return 0;
}
int main()
{
    D9servo.begin(9);
    
    while (1)
    {
        serial.print("0 start\n");
        D9servo.write(0);
        serial.print("0 end\n");
        delayinms(2000);
        serial.print("180 start\n");
        D9servo.write(180);
        serial.print("180 end\n");
        delayinms(2000);
    }
    return 0;
}
