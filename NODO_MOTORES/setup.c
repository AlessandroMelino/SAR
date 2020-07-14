#include <LPC21xx.H> 
#include "LPC_FullCAN_SW.h"
#include "setup.h"
#include "funciones.h"

unsigned int volatile gTimerTick;
unsigned int volatile encoderITimerTick=0,encoderDTimerTick=0;
unsigned int CanRX;
unsigned int Timer1, T1, Active_Timer1=1,T1ticks=T1_MSEGUNDOS*10;  //10msde carga
unsigned int Timer_freno, Tfreno, Active_Timerfreno,Tfrenoticks=TFRENO_MSEGUNDOS*10; //750ms  //   de los timer 

// Rutinas de atención a interrupciones
void DefaultISR (void) __irq; 
void Timer0ISR (void) __irq;

/**************************************************************************/
// DefaultISR(): Rutina de Interrupción por defecto
/*************************************************************************/ 
void DefaultISR (void) __irq 
{
  while (1)
  { // no se debe entrar aquí, de hacerlo es un error fatal
  }
}


/**************************************************************************/
//	Timer0ISR(): Rutina de interrupción del Timer0
/***************************************************************************/ 
void Timer0ISR (void) __irq 
{
  //incrementamos el tick
  gTimerTick++;
	encoderITimerTick++;
	encoderDTimerTick++;
  T0IR = 1;						// Reseteamos el flag de interrupción
  
   //Actualizamos los temporizadores software
   
		if (Active_Timer1)			   //Temporizacion 100ms 
			Timer1--;			  
    if (Timer1==0){
			T1=1;
			Active_Timer1=0;  //Timer 1 se desactiva
		}

		if (Active_Timerfreno)				//Temporización 750ms
			Timer_freno--;
    if (Timer_freno==0){
			Tfreno=1;
			Active_Timerfreno=0;    //Requiere reactivación manual
		}
		
		
  VICVectAddr = 0xFFFFFFFFL; // Señalamos el final de la Interrupción
}


/**************************************************************************/
/*********     Funciones de manejo de los timers      *********************/
/**************************************************************************/

void Set_T1(void){
	Timer1=T1ticks;
	T1=0;
	Active_Timer1=1;
}



void Set_Tfreno(void){
	Timer_freno=Tfrenoticks;
	Tfreno=0;
	Active_Timerfreno=1;
}

void Remove_Tfreno(void){
	Timer_freno=Tfrenoticks;
	T1=0;
	Active_Timer1=0;
}

/**************************************************************************/
// short IsTimeExpired(unsigned int): Comprobación de tiempo agotado
/***************************************************************************/ 
short IsTimeExpired (unsigned int timestamp)	//valor a comparar
{
unsigned int time_now;

  time_now = gTimerTick;
  if (time_now > timestamp)
  {
      return 1;
  }
  else
  {
      return 0;
  }
}

/***************************************************************************/
	//Implementación de espera
/***************************************************************************/ 
void SlowDown (	 unsigned int delay ) // número de timeouts de 100us 
  
{
  delay += gTimerTick;
  while (!IsTimeExpired(delay))
  {
  }
}
/******************************************************************************/
	//activación y desactivación de las interrupciones
/******************************************************************************/


//void activoINTS(void){
//	VICIntEnable = 0x00000010L;					//No utilizadas en esta versión
//	VICIntEnable = 0x02000000L;
//	}

//void desactivoINTS(void){
//	VICIntEnable = 0x00000000L;										
//	}

/**************************************************************************
DOES:    Configure PWM peripherals for both motors
***************************************************************************/ 
void Config_PWM(void) {
	
	PINSEL0	|= (1<<15)|(0<<14); 							  // PWM2 en P0.7
	PINSEL0 |= (1<<19)|(0<<18);									// PWM6 en P0.9
	PWMPR = 0;																	// NO Prescaler (60MHz)
	PWMMR0 = MR0_VALUE;			  									// Setup priodo de la PWM, se pasa a segundos
	PWMMR2 = MR0_VALUE/2;		  									// Duty 50% (Motor izquierdo parado), se pasa a segundos
	PWMMR6 = MR0_VALUE/2;		  									// Duty 50% (Motor derecho parado), se pasa a segundos
	PWMMCR |= (0<<0)|(1<<1)|(0<<2);							// Int OFF; Reset ON; Stop OFF **
	PWMLER |= (1<<0)|(1<<2)|(1<<6);							// Carga MR0, MR2 y MR6
	PWMTCR |= (1<<3)|(1<<0);										// PWM ON; Contador ON **
	PWMPCR |= (1<<10)|(1<<14);									// Habilita salida PWM1.3 y PWM1.5 **
}

/******************************************************************************/
	//configuración inicial del microcontrolador
/******************************************************************************/

void config(void){
	
	VICIntEnClr = 0xFFFFFFFFL; // desabilitamos Ints
	//Inicialmente los pines de la PWM estan como entradas
	Config_PWM(); //Se configuran los pines de la PWM
	IODIR0|=(1<<6)|(1<<8); //Pines del brake como salidas
	IOCLR0|=(1<<6)|(1<<8); //Pin 8 brake derecho y pin 6 brake izquierdo
	
	Timer1= T1ticks;  
	T1=0;
		
	Timer_freno= Tfrenoticks; 
	Tfreno=0;


  	VPBDIV = 1; 	// perifericos a la velocidad de reloj (sin divisor)
	
	// Inicialización Vector Interrupt Controller
  	VICIntEnClr = 0xFFFFFFFFL; // desabilitamos Ints
  	VICIntSelect = 0x00000000L;
	// Vector de interrupción por defecto
  	VICDefVectAddr = (unsigned long) DefaultISR;
	
	// Inicialización de interfaz CAN
  	// CAN 1, usa IRQVec0, a 125kbit
  	FullCAN_Init(1,0,CANBitrate001m_12MHz); 
  	//CAN Err ISR a IRQVec2
  	FullCAN_SetErrIRQ(2);  
		
  	// inicialización del Timer
  	T0MR0 = 5999; // 100 microsegundos = 6.000-1 cuentas
  	T0MCR = 3; // Interrupción y Reset en MR0
  	T0TCR = 1;  // Timer0 habilitado
		
	//  Timer0 ISR en IRQVec3
  	VICVectAddr3 = (unsigned long) Timer0ISR; // fijamos vector de interrupción
  	VICVectCntl3 = 0x20 | 4;  // se usa para el Timer0
  	VICIntEnable = 0x00000010L;  //habilitamos interrupción Timer0 
	
	//  CAN RX! ISR en IRQVec4
	  VICVectAddr4 = (unsigned long) FullCAN_CANISR_Rx1; // fijamos vector de interrupción
	  VICVectCntl4 = 0x20 | 26;  // se usa para el CAN RX1 
  	VICIntEnable = 0x02000000L;  //habilitamos interrupción CAN RX1
	
	// fijamos los filtros para el CAN 1
  	FullCAN_SetFilter(1,0x110); 	//JoystickA: valores de dirección
	  FullCAN_SetFilter(1,0x111); 	//JoystickB: modo de funcionamiento
	  FullCAN_SetFilter(1,0x120); 	//PC: valores de dirección
//		FullCAN_SetFilter(1,0x101);//Motores tDabs
//		FullCAN_SetFilter(1,0x102);//Motores tIabs
//  	FullCAN_SetFilter(1,0x210);//Bateria
//    FullCAN_SetFilter(1,0x201);//SensoresA: valores de la mitad de los sensores de posición
// 	  FullCAN_SetFilter(1,0x202);//SensoresB: la otra mitad
// 		FullCAN_SetFilter(1,0x203);//Acelerómero: valores del acelerómetro
	
		IODIR0&=~(1<<22);
		prev_A_der=IO0PIN>>24 & 1;
		prev_B_der=IO0PIN>>22 & 1;
		prev_A_izq=IO0PIN>>4  & 1;
		prev_B_izq=IO0PIN>>5  & 1;

  	SlowDown(1000);					//esperamos 10ms
}
