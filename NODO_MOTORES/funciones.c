#include <LPC21xx.H> 
#include "LPC_FullCAN_SW.h"
#include "funciones.h"
#include "setup.h"
#include "math.h"

enum tipo_de_estado_motores lazoPC;
int auxD,auxI,datoD,datoI;
//int realD,realI;	    //valores de aceleración
short modo;									//modo de operación (Joystick, menú...)

/*VARIABLES PARA SONDEO DE FLANCOS*/
short prev_A_izq, prev_B_izq;
short flanco_subida_A_izq, flanco_bajada_B_izq, flanco_bajada_A_izq, flanco_subida_B_izq;
short prev_A_der, prev_B_der;
short flanco_subida_A_der, flanco_bajada_B_der, flanco_bajada_A_der, flanco_subida_B_der;

/*VARIABLES CONTROL MOTORES*/
int Pulsos_Der;
int Pulsos_Izq;
int state_izq=0;
int state_der=0;


//Motor izquierdo
float Velocidad_Izq;										  // Velocidad del motor izquierdo leida
float error_Izq , error_nm1_Izq , u_Izq , u_nm1_Izq;
float Error_Sat_Izq , Consigna_Sat_Izq;

//Motor derecho
float Velocidad_Der;										 	// Velocidad del motor izquierdo leida
float error_Der,error_nm1_Der,u_Der,u_nm1_Der;
float Error_Sat_Der,Consigna_Sat_Der;

int mensajeA,mensajeB,dir;

int encoderIabs,encoderDabs;  //Valores absolutos de los encoder
int EncD, EncI,turno;
int cuentaI,cuentaD;

/*****************************************************************************/
// 	lee_can(): Lectura de mensajes CAN, hace distinción entre modo PC y otros													
/*****************************************************************************/

void lee_can(void){

	FULLCAN_MSG MsgBuf; // mensaje al Buffer

 	// mira si hay mensaje en CAN 1
  if (FullCAN_PullMessage(1,&MsgBuf)){   // Mensaje recibido
		if ((MsgBuf.Dat1 & 0x07FF) == ID_MSG_JOYA)	//mensaje JoystickA
		{
			modo=(short)(MsgBuf.DatB&0xFFFF);
			if(modo!=3){	//todos los modos menos PC 					
				auxD=((signed char)(MsgBuf.DatA&0xFF));	  //guardamos valores
				auxI=((signed char)((MsgBuf.DatA&0xFF00)>>8));
			}
		}
		if (modo==3){	  //modo PC
			if ((MsgBuf.Dat1 & 0x07FF) == ID_MSG_PC)	   //mensaje PC
			{ 
				datoD=(signed char)(MsgBuf.DatA&0xFF);		//guardamos valores
				datoI=(signed char)((MsgBuf.DatA&0xFF00)>>8);
				if((MsgBuf.DatB&0xFF)=='A')
					lazoPC=LA;//Si se recibe una A en el mensaje, LAZO ABIERTO
				else if((MsgBuf.DatB&0xFF)=='C')
					lazoPC=LC;//Si se recibe una C en el mensaje, LAZO CERRADO
				else 
					lazoPC=PARADO;//Si no se ha recibido nada, no se considerará ningún lazo, por seguridad
			}
		}
	}
}

/***************************************************************************/
//	envio_can(): Envío de datos del encoder por CAN
/***************************************************************************/

void envio_can(int datoA, int datoB,int etiqueta){

	FULLCAN_MSG MsgBuf;
	MsgBuf.Dat1 = etiqueta;
  MsgBuf.DatA = datoA;  
	MsgBuf.DatB = datoB; 
  
 	FullCAN_PushMessage(1,&MsgBuf);		
	
}

/****************************************************************************/
//	lee_encoderD():Lectura del valor del encoder derecho
/****************************************************************************/

void lee_encoderD(void)
{
	
	encoderDabs+=EncD;
	EncD=0;
	cuentaD=encoderDTimerTick;
	
}

/***************************************************************************/
//	lee_encoderI():Lectura del valor del encoder izquierdo 
/**************************************************************************/

void lee_encoderI(void)
{
	
	encoderIabs+=EncI;
	EncI=0;
	cuentaI=encoderITimerTick;
	
}

/***************************************************************************/
//	envio_canD(): Envío de datos del encoderD por CAN
/***************************************************************************/

void envio_canD(void){		 
	
	// construcción del mensaje
	FULLCAN_MSG MsgBuf;
	MsgBuf.Dat1 = 0x00080101L;
  MsgBuf.DatA = encoderDabs; // 
	MsgBuf.DatB = cuentaD;      //  
  
	//  Transmisión del mensaje por CAN 1
  FullCAN_PushMessage(1,&MsgBuf);											

}

/***************************************************************************/
//	envio_canI(): Envío de datos del encoderI por CAN
/***************************************************************************/

void envio_canI(void){		 

	// construcción del mensaje	
	FULLCAN_MSG MsgBuf;
	MsgBuf.Dat1 = 0x00080102L;
  MsgBuf.DatA = encoderIabs; // 
	MsgBuf.DatB = cuentaI; //  
  
	// Transmisión del mensaje por CAN 1
  FullCAN_PushMessage(1,&MsgBuf);		
										
}

/****************************************************************************/
//	envio_datos: Envío de datos de control a la tarjeta de motores
/****************************************************************************/

//void envio_datos(void){		

//	realI=auxI;
//	realD=auxD;
//	//Generación y envío de los mensajes de control
//	if ((realI==0)&&(realD==0)) //Ruedas sin velocidad
//	{
//	
//	  Active_Timerfreno=1; //Activa timerfreno, solo será necesario al no mandar velocidad
//		 
//		Avanza_D(0); 		   // No se
//		Avanza_I(0); 		  // manda potencia
//		
//		if(Tfreno==1){	 //Toca activar el freno pasados 750ms sin mandar velocidad
//			Active_Timerfreno=0; //Desactiva timer_freno, estará la silla parada
//			Tfreno=0;   //Freno atendido			
//			Frena();	  //Una vez se activa, se queda el freno puesto 			
//	  }	
//	}
//	else
//	{
//		Active_Timerfreno=0;  //Desactiva la cuenta del timer del freno
//		Tfreno=0;             //Quita el flag por si la cuenta finalizó en ejecución de otro código
//		Timer_freno=Tfrenoticks; //Recarga el timer del freno para cuando vuelva a parar la silla
//			
//		if (realD>0)		   // Rueda derecha avanza:
//		{					   //
//		    Quitafreno();	   // Desacoplo del freno
//			Avanza_D (realD);  // Potencia de la rueda = real D
//		}
//		else if(realD<0)	   //Rueda derecha retrocede
//		{					   //
//			Quitafreno();	   //
//			Retrocede_D(realD);// 
//		}
//		else
//		{
//			Retrocede_D(realD);//Rueda derecha a 0, no se toca freno, no necesario
//		}
//		if (realI>0)		   //Rueda izquierda avanza
//		{					   //
//		    Quitafreno();	   //
//			Avanza_I(realI);   //
//		}
//		else if(realI<0)	   //Rueda izquierda retrocede
//		{					   //
//			Quitafreno();	     //
//			Retrocede_I(realI);// 
//		}
//	 	else
//		{
//			Retrocede_I(realI);//Rueda izquierda parada, no se toca freno, no necesario 
//		}
//	 
//	}
//	    
//}

/***************************************************************************/
//	envio_datospc(): Envío de datos de control desde el PC
/***************************************************************************/

//void envio_datospc(void){

//	if ((datoI==0)&&(datoD==0))
//	{
//		Active_Timerfreno=1; //Activa timerfreno, solo será necesario al no mandar velocidad 

//		Avanza_D(0); 
//		Avanza_I(0); 
//		
//		if(Tfreno==1){
//	    
//			Active_Timerfreno=0; //Desactiva timer_freno, estará la silla parada
//			Tfreno=0;   //Freno atendido
//			Frena();    //Una vez ejecutada, el freno ya se queda puesto
//  			
//		 }		
//	}
//	else
//	{
//		Active_Timerfreno=0;  //Desactiva la cuenta del timer del freno
//		Tfreno=0;             //Quita el flag por si la cuenta finalizó en ejecución de otro código
//		Timer_freno=Tfrenoticks; //Recarga el timer del freno para cuando vuelva a parar la silla                                                                                                                                                                                                		
//			
//		if (datoD>=0)
//		{	
//			Quitafreno();
//			Avanza_D(datoD); 
//		}
//		else if(datoD<0)
//		{
//			Quitafreno();
//			Retrocede_D(datoD); 
//		}
//	
//		if (datoI>=0)
//		{	
//			Quitafreno();
//			Avanza_I(datoI); 
//		}
//		else if(datoI<0)
//		{
//			Quitafreno();
//			Retrocede_I(datoI); 
//		}//Fin else if (datoI<0)	  
//	}//Fin else
//}//Fin envio_datospc

/**************************************************************************
DOES:    Realizar el control de los motores
***************************************************************************/ 

void Controla_Motores(int datosIzq, int datosDer){
		
	if ((datosIzq==0)&&(datosDer==0))
	{
		Active_Timerfreno=1; //Activa timerfreno, solo será necesario al no mandar velocidad 
	
		Set_Velocidad_Der(0);	
		Set_Velocidad_Izq(0);	
		
		if(Tfreno==1){
	    
			Active_Timerfreno=0; //Desactiva timer_freno, estará la silla parada
			Tfreno=0;   //Freno atendido
			Frena();    //Una vez ejecutada, el freno ya se queda puesto
  			
		 }		
	}
	else
	{
		Active_Timerfreno=0;  //Desactiva la cuenta del timer del freno
		Tfreno=0;             //Quita el flag por si la cuenta finalizó en ejecución de otro código
		Timer_freno=Tfrenoticks; //Recarga el timer del freno para cuando vuelva a parar la silla                                                                                                                                                                                                		
		Quitafreno();

	
//Medida de velocidad del motor izquierdo	
	Velocidad_Izq = (Pulsos_Izq*2*PI)/(N*EDGES*Ts*0.001);	// Obtención de la velocidad leida (en rad/s)
	EncI+=Pulsos_Izq;
	Pulsos_Izq=0;

	//Sentencias de control del motor izquierdo
	error_Izq=((datosIzq*3.51)-Velocidad_Izq)-Error_Sat_Izq;
	u_Izq=G*error_Izq-c*G*error_nm1_Izq+u_nm1_Izq;	
	
	Set_Velocidad_Izq(u_Izq);	

	Error_Sat_Izq=(u_Izq-Consigna_Sat_Izq)*Ka;	
	
	error_nm1_Izq=error_Izq;
	u_nm1_Izq=u_Izq;

	//Medida de velocidad del motor derecho	
	Velocidad_Der = (Pulsos_Der*2*PI)/(N*EDGES*Ts*0.001);	// Obtención de la velocidad leida (en rad/s)
	EncD+=Pulsos_Der;
	Pulsos_Der=0;

	//Sentencias de control del motor derecho
	error_Der=((datosDer*3.51)-Velocidad_Der)-Error_Sat_Der;
	u_Der=G*error_Der-c*G*error_nm1_Der+u_nm1_Der;	
		
	Set_Velocidad_Der(u_Der);	

	Error_Sat_Der=(u_Der-Consigna_Sat_Der)*Ka;
	
	error_nm1_Der=error_Der;
	u_nm1_Der=u_Der;
	}
}

/**************************************************************************
DOES:    Aumentar o disminuir los contadores de pulsos
***************************************************************************/ 

void Cuenta_Pulsos(void){										//Conteo de pulsos
	/*
	//Motor izquierdo
	
	switch(state_izq){
		case 0:
                   
			if((IO0PIN>>4 & 1)==1){
				Pulsos_Izq--;
				state_izq=2;
			}
			if((IO0PIN>>5 & 1)==1){
				Pulsos_Izq++;
				state_izq=1;
			}
		break;
		case 1:
			if((IO0PIN>>4 & 1)==1){
				Pulsos_Izq++;
				state_izq=3;
			}
			if((IO0PIN>>5 & 1)==0){
				Pulsos_Izq--;
				state_izq=0;
			}
		break;
		case 2:
			if((IO0PIN>>4 & 1)==0){
				Pulsos_Izq++;
				state_izq=0;
			}
			if((IO0PIN>>5 & 1)==1){
				Pulsos_Izq--;
				state_izq=3;
			}
		break;	
		case 3:
			if((IO0PIN>>4 & 1)==0){
				Pulsos_Izq--;
				state_izq=1;
			}
			if((IO0PIN>>5 & 1)==0){
				Pulsos_Izq++;
				state_izq=2;
			}
		break;
		}

   //Motor derecho       
 	switch(state_der){
		case 0:
                   
			if((IO0PIN>>24 & 1)==1){
				Pulsos_Der++;
				state_der=2;
			}
			if((IO0PIN>>22 & 1)==1){
				Pulsos_Der--;
				state_der=1;
			}
		break;
		case 1:
			if((IO0PIN>>24 & 1)==1){
				Pulsos_Der--;
				state_der=3;
			}
			if((IO0PIN>>22 & 1)==0){
				Pulsos_Der++;
				state_der=0;
			}
		break;
		case 2:
			if((IO0PIN>>24 & 1)==0){
				Pulsos_Der--;
				state_der=0;
			}
			if((IO0PIN>>22 & 1)==1){
				Pulsos_Der++;
				state_der=3;
			}
		break;	
		case 3:
			if((IO0PIN>>24 & 1)==0){
				Pulsos_Der++;
				state_der=1;
			}
			if((IO0PIN>>22 & 1)==0){
				Pulsos_Der--;
				state_der=2;
			}
		break;
		}         
  
	*/
	//Canal A izquierdo	
	if((prev_A_izq == LOW) && ((IO0PIN>>4 & 1) == HIGH))	flanco_subida_A_izq =TRUE;
	if((prev_A_izq == HIGH) && ((IO0PIN>>4 & 1) == LOW))	flanco_bajada_A_izq =TRUE;		
		
	//Canal B izquierdo	
	if((prev_B_izq == LOW) && ((IO0PIN>>5 & 1) == HIGH))	flanco_subida_B_izq =TRUE;
	if((prev_B_izq == HIGH) && ((IO0PIN>>5 & 1) == LOW))	flanco_bajada_B_izq =TRUE;		
		
	//Canal A derecho	
	if((prev_A_der == LOW) && ((IO0PIN>>24 & 1) == HIGH))	flanco_subida_A_der =TRUE;
	if((prev_A_der == HIGH) && ((IO0PIN>>24 & 1) == LOW))	flanco_bajada_A_der =TRUE;		
		
	//Canal B derecho	
	if((prev_B_der == LOW) && ((IO0PIN>>22 & 1) == HIGH))	flanco_subida_B_der =TRUE;
	if((prev_B_der == HIGH) && ((IO0PIN>>22 & 1) == LOW))	flanco_bajada_B_der =TRUE;	
		
	prev_A_izq=IO0PIN>>4 & 1;
	prev_B_izq=IO0PIN>>5 & 1;
	prev_A_der=IO0PIN>>24  & 1;
	prev_B_der=IO0PIN>>22  & 1;		
	
	//Motor izquierdo
	if(flanco_subida_A_izq){								//Flanco de subida del canal A 
		
		flanco_subida_A_izq = FALSE;
		
		if((IO0PIN>>5 & 1) == LOW)							//... y canal B a 0
			Pulsos_Izq--;
		else
			Pulsos_Izq++;
	}
		
	if(flanco_bajada_A_izq){								//Flanco de bajada del canal A 
			
		flanco_bajada_A_izq = FALSE;
			
		if((IO0PIN>>5 & 1) == HIGH)							//... y canal B a 1
			Pulsos_Izq--;
		else
			Pulsos_Izq++;
	}
		
	if(flanco_subida_B_izq){								//Flanco de subida del canal B 
		
		flanco_subida_B_izq = FALSE;
		
		if((IO0PIN>>4 & 1) == HIGH)							//... y canal A a 1
			Pulsos_Izq--;
		else
			Pulsos_Izq++;
	}
	
	if(flanco_bajada_B_izq){								//Flanco de bajada del canal B 
		
		flanco_bajada_B_izq = FALSE;
		
		if((IO0PIN>>4 & 1) == LOW)							//... y canal A a 0
			Pulsos_Izq--;
		else
			Pulsos_Izq++;
	}		
	
	//Motor derecho
	
	if(flanco_subida_A_der){								//Flanco de subida del canal A 
		
		flanco_subida_A_der = FALSE;
		
		if((IO0PIN>>22 & 1) == LOW)							//... y canal B a 0
			Pulsos_Der++;
		else
			Pulsos_Der--;
	}
	
	if(flanco_bajada_A_der){								//Flanco de bajada del canal A 
		
		flanco_bajada_A_der = FALSE;
		
		if((IO0PIN>>22 & 1) == HIGH)						//... y canal B a 1
			Pulsos_Der++;
		else
			Pulsos_Der--;
	}
		
	if(flanco_subida_B_der){								//Flanco de subida del canal B 
		
		flanco_subida_B_der = FALSE;
		
		if((IO0PIN>>24 & 1) == HIGH)					//... y canal A a 1
			Pulsos_Der++;
		else
			Pulsos_Der--;
	}
	
	if(flanco_bajada_B_der){								//Flanco de bajada del canal B 
		
		flanco_bajada_B_der = FALSE;
		
		if((IO0PIN>>24 & 1) == LOW)						//... y canal A a 0
			Pulsos_Der++;
		else
			Pulsos_Der--;
	}
}

/**************************************************************************
DOES:    Establecer la velocidad de los motores
***************************************************************************/ 

/* Velocidad deseada del motor izquierdo */
void Set_Velocidad_Izq(float Velocidad){				// Consigna en rad/s
		
	// Wu_max de consigna a la velocidad en vacio
	if (Velocidad > Wu_max){Velocidad = Wu_max;}	
	else if (Velocidad < -Wu_max){Velocidad = -Wu_max;}

	Consigna_Sat_Izq=Velocidad;
	
	// Carga de nueva consigna
	PWMMR2 = (MR0_VALUE/2)*((Velocidad + Wu_max)/Wu_max);	// Ciclo en alto; además hay que hacer un desplazamiento 
	PWMLER |= (1<<2);																	// Actualiza MR2
}

/* Velocidad deseada del motor derecho */
void Set_Velocidad_Der(float Velocidad){				// Consigna en rad/s
	
	// Wu_max de consigna a la velocidad en vacio
	if (Velocidad > Wu_max){Velocidad = Wu_max;}	// OJO SATURACION. SIMULAR + SOLUCIONAR (antiwindup)
	else if (Velocidad < -Wu_max){Velocidad = -Wu_max;}

	Consigna_Sat_Der=Velocidad;
	
	// Carga de nueva consigna
	PWMMR6 = (MR0_VALUE/2)*((Velocidad + Wu_max)/Wu_max);	// Ciclo en alto; además hay que hacer un desplazamiento 
	PWMLER |= (1<<6);																		// Actualiza MR6
}


/**************************************************************************
DOES:Bloquea el driver para activar freno con los pines correspondientes
***************************************************************************/ 


void Frena (void){
	
	IOSET0|=(1<<6)|(1<<8); //Pin 8 brake derecho y pin 6 brake izquierdo
	
}

/**************************************************************************
DOES:Desbloquea el driver para desactivar freno con los pines correspondientes
***************************************************************************/ 


void Quitafreno (void){
	
	IOCLR0|=(1<<6)|(1<<8); //Pin 8 brake derecho y pin 6 brake izquierdo
	
}

