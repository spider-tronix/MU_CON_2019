/*
    Filename: Code.ino
    Author: The real Spider Tronix 2019
    Purpose: First years'  workshop (2019)
 */

//REQUIRED HEADERS
#include<avr/interrupt.h>
#include<avr/io.h>

//for async USART
volatile char USART_DATA_BUFFER[32] = "";
volatile uint8_t USART_DATA_INDEX = 0;
volatile bool USART_BUFFER_READY = false;

//////////////////
// UART methods //
//////////////////
/*
    Function name: initUSART
    Input: none
    Output: none
    Logic: Initialize serial communication
    Example call: initUSART();
*/
void initUSART() {
    UCSR0C = (1 << UCSZ00) | (1 << UCSZ01); //asynchronous, 8 bit data
    UBRR0 = 103; //set baud rate = 9600 bps
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0); //enable RX and TX
}

/*
    Function name: USART_sendByte
    Input:  uint8_t data: the data to be sent
    Output: none
    Logic: Send 1 byte of data to the serial monitor
    Example call: USART_sendByte('A');
*/
void USART_sendByte(uint8_t data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

/*
    Function name: USART_sendData
    Input:  char* data: the data (string) to be sent
    Output: none
    Logic: Send a string to the serial monitor
    Example call: USART_sendData("Hello world!\n");
*/
void USART_sendData(const char* data) {
    while(*data) {
        USART_sendByte(*data);
        data++;
    }
}

/*
    Function name: USART_sendData
    Input:  float data: the data to be sent
    Output: none
    Logic: Send a float to the serial monitor
    Example call: USART_sendData(10.0);
*/
void USART_sendData(float data) {
    char temp[10];
    dtostrf(data, 8, 2, temp);
    USART_sendData(temp);
}

/*
    Function name: USART_sendData
    Input:  uint16_t data: the data to be sent
    Output: none
    Logic: Send a uint16_t to the serial monitor
    Example call: USART_sendData((uint16_t)10));
*/
void USART_sendData(uint16_t data) {
    char temp[10];
    sprintf(temp, "%d", data);
    USART_sendData(temp);
}

/*
    Function name: USART_getByte
    Input:  uint8_t* data : pointer to store received byte
    Output: bool : true if a byte was received and put in data
    Logic: By checking the UCSR0A.RXC0 flag availability of data in UDR0 is known,
            if data is available, it is stored and true is returned
    Example call: 
    uint8_t data;
    if (USART_getByte(&data)) {
        //do something with data
    } 
*/
bool USART_getByte(uint8_t *data) {
    if (UCSR0A & (1 << RXC0)) {
        *data = UDR0;
        return true;
    }
    return false;
}

/*
    Function name: USART_readBuffer 
    Input:  char *dest: memory to be filled with received data (capacity 32 bytes)
    Output: none 
    Logic:  read the received data that is stored in the buffer
    Example call: 
    char data[32];
    if (USART_BUFFER_READY) USART_readBuffer(data);
 */
void USART_readBuffer(char *dest) {
    strcpy(dest, USART_DATA_BUFFER);
    USART_DATA_INDEX = 0;
    USART_BUFFER_READY = false;
}

/*
    Function name: ISR(USART_RX_vect)
    Logic: Whenever data is available it is appended to the buffer if the buffer has already been read 
 */
ISR(USART_RX_vect) {
    if (USART_BUFFER_READY == false) {
        char data = UDR0;
        if (data == '\n' || data == '\r' || USART_DATA_INDEX == 31) 
        {
            data = '\0';
            USART_BUFFER_READY = true;
        }
        USART_DATA_BUFFER[USART_DATA_INDEX] = data;
        USART_DATA_INDEX++;
    }
}


int main() {
    sei(); //Enable global interrupts
    return 0;
}