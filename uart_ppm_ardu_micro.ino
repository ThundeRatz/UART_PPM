#include <avr/io.h>
#include <util/delay.h>

#define TOP_PPM 250 // até 255
#define MID_PPM 50  // top/2
#define MIN_PPM 0

#define PPML OCR1A
#define PPMR OCR1B

#define PPM_DELAY_US 200

#define reset_timeout() (timeout = millis())
#define get_timeout()   (millis() - timeout)

int led = 13;
uint8_t auton = 0;
int vel_esq = 0, vel_dir = 0;

uint32_t timeout = 0;
uint16_t fail = 0;

void setup() {
    Serial.begin(9600);

    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << COM1A0) | (1 << COM1B0) | (1 << WGM11); // Clear OCR1A/B on compare match, clear at top | Fast PWM mode
    TCCR1B = (1 << WGM13)  | (1 << WGM12)  | (1 << CS11);  // Preescaler at clk(I/O)/8.
    OCR1B  = 3000;
    OCR1A  = 3000;
    ICR1   = 40000;

    DDRD &= ~(1 << PD2);
    DDRB |=  (1 << PB5) | (1 << PB6);

    EICRA = (1 << ISC00);
    EIMSK = (1 << INT0);

    pinMode(led, OUTPUT);
}

void loop() {
    if (auton >= 220 && Serial.available()) {
        parse_speed();
        fail = 0;
    } else {
        fail++;
    }

    if (fail > 100) {
        fail = 101;
        digitalWrite(led, LOW);
        vel_esq = vel_dir = 0;
        ppm_send(30, 0);
    } else {
        digitalWrite(led, HIGH);
        ppm_send(vel_esq, vel_dir);
    }
}

ISR(INT0_vect) {
    // Borda de subida
    if (PIND & (1 << PD2)) {
        TCNT3 = 0;
        TCCR3B |= (1 << CS32) | (1 << CS30);
        return;
    }
    // Borda de descida
    uint8_t reading = TCNT3;
    TCCR3B = 0;

    //  if (TIFR2 & (1 << TOV2)) {
    //    TIFR2 |= (1 << TOV2);
    //    reading = 255;
    //  }
    auton = reading;
}

uint8_t packet_buffer[8] = { 0 };

uint8_t parse_speed() {
    int temp;

    temp = Serial.read();
    if (!(temp == 0xFF))
        return 255;

    packet_buffer[0] = temp;

    // 6 bytes: inicio, dir_esq, vel_esq, dir_dir, vel_dir, fim
    for (int i = 1; i <= 5; i++) {
        reset_timeout();
        while ((temp = Serial.read()) == -1) {
            if (get_timeout() >= 10)
            return 254;
        }

        packet_buffer[i] = temp;
    }

    if (packet_buffer[5] != 0xFE || (packet_buffer[1] > 1) || (packet_buffer[3] > 1))
        return 255;

    vel_esq = packet_buffer[1] == 0 ? (int)(packet_buffer[2]) : -(int)(packet_buffer[2]);
    vel_dir = packet_buffer[3] == 0 ? (int)(packet_buffer[4]) : -(int)(packet_buffer[4]);

    return 1;
}

void ppm_send(int vel_esq, int vel_dir) {
	vel_esq = constrain(vel_esq, -TOP_PPM, TOP_PPM);
	vel_dir = constrain(vel_dir, -TOP_PPM, TOP_PPM);

	uint16_t vL = map(vel_esq, -TOP_PPM, TOP_PPM, 2000, 4000);
	uint16_t vR = map(vel_dir, -TOP_PPM, TOP_PPM, 2000, 4000);

	vL = constrain(vL, 2000, 4000);
	vR = constrain(vR, 2000, 4000);

	while (PPML != vL || PPMR != vR) {
		if (PPML > vL)
			PPML--;

		else if (PPML < vL)
			PPML++;

		if (PPMR > vR)
			PPMR--;

		else if (PPMR < vR)
			PPMR++;

		_delay_us(PPM_DELAY_US);
	}
}
