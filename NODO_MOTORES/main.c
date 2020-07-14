/* ASIGNACION DE PINES KATE 
  PINOUT básico del driver:
		Pin BRAKE del driver a GND. Para desabilitarlo según LM18200 TRUTH-TABLE
	VERSION CON PWM BIPOLAR (Motor parado con D=50%) del driver:
		Pin PWM del driver a VDD (digital, 3.3V) y 
		Pin DIR del driver recibe señal PWM procedente del LPC1768. 
		Para motor izquierdo se usa PWM2, saliente por pin P0.7 
		Para motor derecho se usa PWM6, saliente por pin P0.9
		
	PINOUT PARA ENCODERS, según config en este fuente:
		Pin CHA del encoder izquierdo al pin P0.4 (GPIO)
		Pin CHB del encoder izquierdo al pin P0.5 (GPIO)
		Pin CHA del encoder derecho al pin P0.24	(GPIO)
		Pin CHB del encoder derecho al pin P0.28 (GPIO)
	
	PINOUT PARA CAN
	  Pines 23 y 25 del header J3 para CANH-0 Y CANL-0 respectivamente
		P0.25 tambien es usado por CAN
*/

#include <LPC21xx.H> 
#include "LPC_FullCAN_SW.h"
#include "setup.h"
#include "funciones.h"

enum tipo_de_estado_motores estado_motores;
int cont=0;
 int main(void){
	 
	estado_motores=INICIALIZANDO;
	 
	Pulsos_Der=0;																//Se inicaliza a 0 antes de empezar a contar	
	Pulsos_Izq=0;
	encoderDabs=0;
	encoderIabs=0;
	EncD=0;
	EncI=0;
	cuentaI=0;
	cuentaD=0;
	turno=0; 
	config();			  //configuración del micro
	SlowDown(10000); //Tiempo de espera de 1 segundo hasta puesta en marcha
		
	while(1){
		
		Cuenta_Pulsos();	//Cuenta los pulsos continuamente
		
  	if(CanRX){		 // si hemos recibido un mensaje CAN...
				 			
		lee_can();		//Lectura del CAN, hace distinción entre modo PC y otros
		CanRX--;	//Reducimos la cuenta demensajes CAN pendientes.
  	}//Fin if(CanRX==1) 
			
			switch (estado_motores){	  //dependiendo del estado del serial
				
				case INICIALIZANDO: 
//					if (modo==0)
//						estado_motores=PARADO;//Se queda el lazo que estuviera
					if (((modo==3)&&(lazoPC==LA)))
						estado_motores=CONFIG_LA;//Solo modo joystick y PC en lazo abierto
					if (modo==1 || ((modo==3)&&(lazoPC==LC)))
							estado_motores=CONFIG_LC;//Resto de modos en lazo cerrado
					break;
					
			  case PARADO: 
					if (((modo==3)&&(lazoPC==LA)))
							estado_motores=CONFIG_LA;//Solo modo joystick en lazo abierto
					if (modo==1 || ((modo==3)&&(lazoPC==LC)))
							estado_motores=CONFIG_LC;//Resto de modos en lazo cerrado	  
					break;
					
				case CONFIG_LC:
//					Configura_Lazocerrado();
					estado_motores=LC;//El lazo cerrado ha sido satisfactoriamente configurado
					break;
					
				case CONFIG_LA:
//					Configura_Lazoabierto();
					estado_motores=LA;//El lazo abierto ha sido satisfactoriamente configurado
					break;
					
				case LC:
					if((modo==3 && lazoPC==LA))
						estado_motores=CONFIG_LA;
					break;
					
			  case LA:
					if( (modo==1) || (lazoPC!=LA && modo==3) )
						estado_motores=CONFIG_LC;
					break;
					
			  }//Fin switch
					
				
		if(T1==1){			  //cada 10ms se realiza el control
	    
			Set_T1();
							

			if(turno){
				lee_encoderI();
				envio_canI();
				turno=0;
			}else{ 
				lee_encoderD();	
				envio_canD();
				turno=1;
			}
				
			switch (modo)	{	  //modo 1 = joystick, modo 3 = PC

				case 1: 
					if(estado_motores==LC)
						Controla_Motores(auxI,auxD);	 //joystick: datos a la tarjeta motores
					datoI=0;
					datoD=0;

				
				break;
				
//			  case 2:
//					if (estado_motores==LC)
//						envio_datos();	  //soplido: datos a la tarjeta motores
//					break;
				
				case 3:
					if ( (estado_motores==LA && lazoPC==LA) || (estado_motores==LC && lazoPC==LC) )
						Controla_Motores(datoI,datoD);	 //joystick: datos a la tarjeta motores
					else{
						datoI=0;
						datoD=0;
						Controla_Motores(datoI,datoD);	 //joystick: datos a la tarjeta motores
					}
				break;
				
//				case 0: 
//					auxD=0;	
//					auxI=0;
//					Set_Velocidad_Izq(0);												//Se paran los motores
//					Set_Velocidad_Der(0);			
//					Pulsos_Der=0;																//Se inicaliza a 0 antes de empezar a contar	
//					Pulsos_Izq=0;
//				envio_datos();
//					Controla_Motores(datoI,datoD);	 //joystick: datos a la tarjeta motores
//				break;
				
			  default:
					auxD=0;	
					auxI=0;
					datoI=0;
					datoD=0;
					Set_Velocidad_Izq(0);												//Se paran los motores
					Set_Velocidad_Der(0);			
					Pulsos_Der=0;																//Se inicaliza a 0 antes de empezar a contar	
					Pulsos_Izq=0;
					if (estado_motores==LC)
						Controla_Motores(auxI,auxD);	 //joystick: datos a la tarjeta motores
				break;
			  }//Fin switch
		 }//Fin if(T1==1)
 	}//Fin while(1)
}//Fin main()

