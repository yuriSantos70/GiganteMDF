#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Definir as portas
#define MOTOR_PORT    PORTB
#define MOTOR_DDR     DDRB
#define ENABLE12      PB1   // Usaremos PB1 para PWM do motor - PINOS ENABLE12 DA PONTE H
#define ENABLE34      PB2   // Usaremos PB2 para PWM do motor - PINOS ENABLE34 DA PONTE H
#define INPUT1		  PB3	// INPUT PARA A PONTE H -- Definir como OUTPUT
#define INPUT2		  PB4	// INPUT PARA A PONTE H -- Definir como OUTPUT
#define INPUT4		  PB5	// INPUT PARA A PONTE H -- Definir como OUTPUT


#define LASER_PORT    PORTD
#define LASER_DDR     DDRD
#define INPUT3		  PD3 	// INPUT PARA A PONTE H -- Definir como OUTPUT


#define LED_PORT      PORTC
#define LED_DDR       DDRC
#define RECEPTOR_PIN  PC0   // Pino do receptor de laser 
#define LED_PIN_1     PC2   // LEDs para indicar vida
#define LED_PIN_2     PC3
#define LED_PIN_3     PC4
#define LASER_PIN     PC5   // Usaremos PC5 para o controle do laser -- Definir como OUTPUT


volatile uint8_t vidas = 3;  // Inicialmente 3 vidas
volatile uint8_t motorAtivo = 1;

// Função para configurar os pinos e PWM
void setup() {
    // Configurar pinos do motor e laser como saída
    MOTOR_DDR |= (1 << ENABLE12);
    MOTOR_DDR |= (1 << ENABLE34);
	MOTOR_DDR |= (1 << INPUT1);
	MOTOR_DDR |= (1 << INPUT2);
	MOTOR_DDR |= (1 << INPUT4);
	LASER_DDR |= (1 << INPUT3);
	LED_DDR   |= (1 << LASER_PIN);
    
    // Configurar LEDs como saída
    LED_DDR |= (1 << LED_PIN_1) | (1 << LED_PIN_2) | (1 << LED_PIN_3);
    LED_PORT |= (1 << LED_PIN_1) | (1 << LED_PIN_2) | (1 << LED_PIN_3);  // Inicialmente, todos LEDs ligados
   
    // Configurar receptor como entrada
    DDRD &= ~(1 << RECEPTOR_PIN);
    PORTD |= (1 << RECEPTOR_PIN);  // Habilitar pull-up
    
    // Configurar Timer1 para PWM (modo fast PWM, não invertido)
    TCCR1A |= (1 << COM1A1) | (1 << WGM11);  // PWM não invertido no OC1A
    TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11);  // Prescaler de 8
    ICR1 = 19999;  // Freqüência de PWM em 50Hz para controle do motor
    
    // Configurar Timer0 para o controle do laser (prescaler 1024)
    TCCR0A |= (1 << WGM01);  // Modo CTC
    TCCR0B |= (1 << CS02) | (1 << CS00);  // Prescaler 1024
    OCR0A = 156;  // Aproximadamente 1 segundo
    
    // Habilitar interrupção no receptor (INT0)  --  Reset dos LEDS
    EICRA |= (0b10 << ISC00);  // Interrupção na borda de descida
    EIMSK  |= (1 << INT0);   // Habilitar INT0

	// Habilitar interrupção no receptor (PCINT8) --  INTERRUPÇAO DO LDR
    PCICR  |= (1 << PCIE1);
	PCMSK1 |= (1 << PCINT8);

    // Habilitar interrupção do Timer0 para o laser
    TIMSK0 |= (1 << OCIE0A);
    
    sei();  // Habilitar interrupções globais
}

// Função para controle do laser
ISR(TIMER0_COMPA_vect) {
    LASER_PORT ^= (1 << LASER_PIN);  // Alterna o estado do laser a cada 1 segundo
	
}

// Função para detectar laser no receptor
ISR(PCINT8_vect) {
    if (vidas > 0) {
        vidas--;
        if (vidas == 2) {
            LED_PORT &= ~(1 << LED_PIN_1);  // Apagar 1º LED
        } else if (vidas == 1) {
            LED_PORT &= ~(1 << LED_PIN_2);  // Apagar 2º LED
        } else if (vidas == 0) {
            LED_PORT &= ~(1 << LED_PIN_3);  // Apagar 3º LED
        }
        motorAtivo = 0;  // Desativar motor
        _delay_ms(5000);  // Pausa de 5 segundos
        motorAtivo = 1;  // Reativar motor
    }
}

ISR(INT0_vect){
	cli();
	LED_PORT |= (1 << LED_PIN_1) | (1 << LED_PIN_2) | (1 << LED_PIN_3);
	sei();
}

// Função principal
int main(void) {
    setup();
	MOTOR_PORT |= (1 << INPUT4 );
	MOTOR_PORT |= (1 << INPUT1 );
	LED_PORT   |= (1 << LASER_PIN );

    while (1) {
		
        if (motorAtivo) {
            OCR1A = 1500;  // Controle do motor via PWM (exemplo de valor)
        } else {
            OCR1A = 0;  // Desativar motor
			
        }
		
    }
}