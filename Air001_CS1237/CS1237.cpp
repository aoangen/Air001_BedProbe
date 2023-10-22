//#include "wiring_constants.h"
#include "CS1237.h"
#define PULSE_PIN PB_2

#define READ_ADC_CONFIG 0x56
#define WRITE_ADC_CONFIG 0x65

uint32_t pin_sck;
uint32_t pin_dout;

static void send_clk_pulses(uint8_t count)
{
    for (uint8_t i = 0; i < count; i++)
    {
        digitalWrite(pin_sck, HIGH);
        delayMicroseconds(1);
        digitalWrite(pin_sck, LOW);
        delayMicroseconds(1);
    }
    return;
}

void CS1237_init(uint32_t sck, uint32_t dout)
{
    pin_sck = sck;
    pin_dout = dout;

    pinMode(pin_sck, OUTPUT);
    digitalWrite(pin_sck, LOW);
    pinMode(pin_dout, INPUT_PULLUP);
}

int32_t CS1237_read(void)
{
    int32_t data = 0;

    digitalWrite(pin_sck, LOW);

    //wait for data ready
    while (digitalRead(pin_dout));

    digitalWrite(PULSE_PIN, HIGH);

    for (uint8_t i = 0; i < 24; i++)
    {
        digitalWrite(pin_sck, HIGH);
        delayMicroseconds(1);
        data |= digitalRead(pin_dout) << (23 - i);
        digitalWrite(pin_sck, LOW);
        delayMicroseconds(1);
    }
    send_clk_pulses(3);

    digitalWrite(PULSE_PIN, LOW);

    if (data > 0x7FFFFF)
        data -= 0xFFFFFF;
    return data;
}

static uint8_t configure(bool write, uint8_t gain, uint8_t speed, uint8_t channel)
{
    uint8_t data;

    CS1237_read();

    pinMode(pin_dout, OUTPUT);
    send_clk_pulses(2);

    data = (write) ? WRITE_ADC_CONFIG : READ_ADC_CONFIG;
    for (uint8_t i = 0; i < 7; i++)
    {
        digitalWrite(pin_sck, HIGH);
        delayMicroseconds(1);
        digitalWrite(pin_dout, ((data >> (6 - i)) & 0b00000001));
        digitalWrite(pin_sck, LOW);
        delayMicroseconds(1);
    }

    send_clk_pulses(1);

    pinMode(pin_dout, (write) ? OUTPUT : INPUT_PULLUP);
    data = (write) ? (gain | speed | channel) : 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        digitalWrite(pin_sck, HIGH);
        delayMicroseconds(1);
        if (write)
            digitalWrite(pin_dout, ((data >> (7 - i)) & 0b00000001));
        else
            data |= digitalRead(pin_dout) << (7 - i);
        digitalWrite(pin_sck, LOW);
        delayMicroseconds(1);
    }

    send_clk_pulses(1);

    pinMode(pin_dout, INPUT_PULLUP);
    return data;
}

int CS1237_configure(uint8_t gain, uint8_t speed, uint8_t channel)
{
    int ret;

    uint8_t retry = 3;
    uint8_t write_value = 0;
    uint8_t read_value = 0;
    for (uint8_t i = 0; i < retry; i++) {
        write_value = configure(true, gain, speed, channel);
        read_value = configure(false, gain, speed, channel);
        if (write_value != read_value)
            ret = -1;
        else {
            ret = 0;
            break;
        }
    }
    return ret;
}
