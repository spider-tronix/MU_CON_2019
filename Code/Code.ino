/*
    Filename: Code.ino
    Author: The real Spider Tronix 2019
    Purpose: First years'  workshop (2019)
 */

//REQUIRED HEADERS
#include <avr/interrupt.h>
#include <avr/io.h>

////////////////
// UART class //
////////////////

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
    sei(); //Enable global interrupts
    USART::init(9600);
    char data[35];

    while (1)
    {
        //USART::sendData("Shyam\n");
        if (USART::bufferReady()) {
            USART::readBuffer(data);
            USART::sendData(data);
            USART::sendByte('\n');
        }
    }
    return 0;
}