#include <LPC21xx.H>
#include "funciones.h"
#include "timer.h"

short digincrementoV,digdecrementoV; //Variables Jdigital   
int incrementoV, decrementoV; 		//Incrementos de la logica de aceleracion
int Vanterior;//Este tiempo está medido en veces que el timer interrumpe, se usa en soplido
//static int tiempo=0;//Este tiempo está medido en veces que el timer interrumpe, se usa en soplido

/******************************************************************************************************/
/********************************Control joystick******************************************************/
/******************************************************************************************************/

/******************************************************************************************/
// Adquiere_Analogicos(): Toma los valores de los ADC, se ejecuta en la rutina del timer
/******************************************************************************************/
void Adquiere_Analogicos(void){	//Esta función obtiene los valores de x,y y el potenciomentro del joystick.	

	ADCR = 0x012E0401 ;							            /* Start A/D Conversion; Potenciometro*/
  	do{
  		valorpot = ADDR;                              /* Read A/D Data Register */
  	} while ((valorpot & 0x80000000) == 0);			    /* Wait for end of A/D Conversion */
  
  	ADCR &= ~0x01000000;                          	/* Stop A/D Conversion */
	valorpot = (valorpot >> 6) & 0x03FF;             	/* Extract AIN0 Value */
	
	ADCR = 0x012E0402;                      	  /* Start A/D Conversion; ejeX*/
  	do{
  		valorx = ADDR;                              	/* Read A/D Data Register */
  	} while ((valorx & 0x80000000) == 0);       	  /* Wait for end of A/D Conversion */
  
  	ADCR &= ~0x01000000;                      	    /* Stop A/D Conversion */
  	valorx = (valorx >> 6) & 0x03FF;              	/* Extract AIN1 Value */

   	ADCR = 0x012E0404;                      	/* Start A/D Conversion */	/*ejeY*/
  	do{
    	valory = ADDR;                               /* Read A/D Data Register */
  	} while ((valory & 0x80000000) == 0);     	   /* Wait for end of A/D Conversion */
  
  	ADCR &= ~0x01000000;                     	     /* Stop A/D Conversion; ejeY*/
  	valory = (valory >> 6) & 0x03FF;               /* Extract AIN2 Value */
		
//		ADCR = 0x012E0408 ;							          /* Start A/D Conversion; Soplido*/
//  	do {
//  		valorsop = ADDR;                             /* Read A/D Data Register */
//  	} while ((valorsop & 0x80000000) == 0);			   /* Wait for end of A/D Conversion */
//  
//  	ADCR &= ~0x01000000;                           /* Stop A/D Conversion */
//		valorsop = (valorsop >> 6) & 0x03FF;             /* Extract AIN0 Value */

}

/***********************************************************************************************/
// Velocidad_datomotor(): Entradas en mrad/s Salida en unidades para interpretar por los motores
/***********************************************************************************************/
int Velocidad_datomotor(int velocidad_angular){
	return velocidad_angular*RT*60/(2*PI)*PPR*3/58593750;
}

/******************Velocidad con rampa de aceleracion******************/
void Conversion_Joystick(void){ //Traduce los valores del joystick a velocidad lineal y angular. 
//Retorna realI y realD 	
	
incrementoV= 5*TS_MSEGUNDOS/100;  //Los incrementos dependen del tiempo de muestreo
decrementoV= 10*TS_MSEGUNDOS/100; //Aumentan suavemente la velocidad, la disminuyen menos suavemente

////////////////////////////////////////////
//Valor del eje x (Adelante-Atrás) oscilan entre 1023 y 515//
////////////////////////////////////////////
	if ((valorx>770)&&(valorx<810)){ //Zona muerta
		ejex=0;
	}
	else{
		ejex=valorx-785; //0 en reposo, max positiva 238, min negativa -238 aprox
	}
	
////////////////////////////////////////////
//Valor del eje y (Izquierda-Derecha) oscilan entre 1023 y 501//
////////////////////////////////////////////	
	if ((valory>600)&&(valory<900))	{//Zona muerta
		ejey=0;
	}
	else{
		ejey=valory-785;	//0 en reposo, max positiva 238, min negativa -238 aprox
	}
	
//////////////////////////////////////////////////////////
//// Calculo de velocidad comandada(input) y corregida  //
//////////////////////////////////////////////////////////
	inputV=ejex; //Comandada
	inputR=ejey;
	correctV=inputV; //Corregida
	correctR=inputR; 
	
///////////////////////////////////////////////////
// Comprobación de Anulación del frenazo (Parada)//
///////////////////////////////////////////////////
	if (inputV==0 && inputR==0) 
		Parada=0;//Al poner el joystick en reposo, funcionamos normalmente (Ver más adelante)

	if(!Parada){
		
///////////////////////////////////////////////////
//    Saturación de incrementos de velocidad     //
///////////////////////////////////////////////////
		if(abs(correctV-realV)< decrementoV){ //Se satura cuando llega al maximo y que no oscile
				correctV=abs(correctV-realV);
				decrementoV=incrementoV;
		}
		
		if(abs(correctV-realV)< decrementoV){ 
				incrementoV=abs(correctV-realV);
				decrementoV=incrementoV;
		}
		
///////////////////////////////////////////////////
//        Lógica de aceleración                  //
///////////////////////////////////////////////////
		if (correctV>=0 && realV>=0){			//Comando hacia delante, velocidad hacia delante -> 2 casos: 
			if(correctV>realV)							//Si la velocidad corregida es mayor que la actual mandada a motores...
				realV+=incrementoV;						// La velocidad que se manda a motores se va incrementando hasta la corregida
			else if (correctV<realV)				//Si la velocidad corregida es menor que la actual mandada a motores...
				realV-=decrementoV;						// La velocidad que se manda a motores se va decrementando hasta la corregida
		}
		else if (correctV<0 && realV>0){	//Comando hacia atrás, velocidad hacia delante
			realV-=decrementoV; 						//Deceleramos
		}
		else if (correctV>0 && realV<0){	//Comando hacia delante, velocidad hacia atrás
			realV+=incrementoV;							//Aceleramos
		}
		else if (correctV<=0 && realV<=0){//Comando hacia atrás, velocidad hacia atrás -> 2 casos:
			if(correctV<realV)							//Si la velocidad corregida es mayor que la actual mandada a motores...
				realV-=incrementoV;						// La velocidad que se manda a motores se va incrementando hasta la corregida
			else if(correctV>realV)					//Si la velocidad corregida es menor que la actual mandada a motores...
				realV+=decrementoV;						// La velocidad que se manda a motores se va decrementando hasta la corregida
		}
			
///////////////////////////////////////////////////
//Paso a velocidad angular (Valores del potenciometro entre 13 y 990)//
///////////////////////////////////////////////////
	
	wd=((valorpot*1000)/(RADIO*990))*(realV-((correctR*DISEJES)/2000));
	wi=((valorpot*1000)/(RADIO*990))*(realV+((correctR*DISEJES)/2000));
	
	realD=wd/20;
	realI=wi/20;		
	
	realD=Saturacion(realD,-127,127);			//Saturación de real máx
	realI=(Saturacion(realI,-127,127));
		
	}
	
///////////////////////////////////////////////////
//        Activación parada de emergencia        //
///////////////////////////////////////////////////
	if((realV>0&&inputV<0)||(realV<0&&inputV>0)){
		realV=0;
		realD=0; //PARADA DE EMERGENCIA, pasar de velocidad real hacia una dirección, a velocidad demandada en la contraria 
		realI=0; //La silla se mantiene parada hasta volver el joystick a 0 y demandar otra velocidad
		Parada=1;//Flag de parada
	}
}

/**************************************************************************************************************/
// Conversion_JDigital(): Pasa los datos tomados de los ADC: valorx a dato de incremento de velocidad 
// Se toma valory y se pasa a inputR de forma proporcional.
/**************************************************************************************************************/
//void Conversion_JDigital(void){ //Traduce los valores del joystick a incrementos de velocidad.
//	//digincrementoV muestra a cuanto se irá aumentando la velocidad absoluta en función del valor de joystick.
//	//digdecrementoV muestra cuanto irá disminuyendo, este valor deberá ser significativamente más alto para poder parar más rápido
/////////////////////////////////////////////////////
////      Salidas según estado (Vlineales)         //
/////////////////////////////////////////////////////	
//switch(digestado){
//		case PARADO:
//			inputV=0;//realV=0;
//			inputR=0;
//			break;
//		case ACELFOR:
//			digincrementoV=(1+((valorpot+256)*((valorx-511)/10)/1279)) * TS_MSEGUNDOS/100;//Mínimo 10 mm/s^2 , máx 522mm/s^2
//			inputV+=digincrementoV;//realV se satura más adelante
//		case VCTEFOR:
//			break;//Sin incrementos
//		case DECELFOR:
//			digdecrementoV=5*(-1+((valorpot+256)*((valorx-511)/10)/1279)) * TS_MSEGUNDOS/100;//Mínimo 5 , máx 2610 mm/s^2
//			inputV+=digdecrementoV;
//			if(inputV<0)
//				inputV=0;
//			break;	
//		case ACELBACK:
//			digincrementoV=(1+((valorpot+256)*((valorx-511)/10)/1279)) * TS_MSEGUNDOS/100;
//			inputV+=digincrementoV;
//			break;
//		case VCTEBACK:
//			break;//Sin incrementos
//		case DECELBACK:
//			digdecrementoV=5*(1+((valorpot+256)*((valorx-511)/10)/1279)) * TS_MSEGUNDOS/100;
//			inputV+=digdecrementoV;
//			if(inputV>0)
//				inputV=0;
//			break;
//		}
/////////////////////////////////////////////////////////////////////////////////////////////	
////   Salidas comunes a los estados (V de giro, saturaciones, correcciones por sensores)  //
/////////////////////////////////////////////////////////////////////////////////////////////

//	if ((valory>0x1E0)&&(valory<0x220))	{//Eje izquierda-derecha centrado
//		ejey=0;
//		inputR=0;     //No hay velocidad angular
//	}
//	else{
//		ejey=valory-512;
//		inputR=(valorpot+256)*ejey*R_MAX/511/1279; //
//	}
//	inputV=Saturacion(inputV, -(valorpot+256)*V_MAX/1279, (valorpot+256)*V_MAX/1279);//Saturación de la V de entrada
//	
//	Sensores();//Al corregir inputV por correctV, la inputV permanece y puede ser mayor que la que el usuario cree al estar memorizada
//	if (correctV>correctV1)
//		correctV+=(inputV-correctV)*TS_MSEGUNDOS/8000; //Si la nueva corregida es mayor que la anterior hacemos un suave incremento
//                                                  //Iguala a la memorizada en 8 segundos
//	realV=correctV;
//	realV=Saturacion(realV, -(valorpot+256)*V_MAX/1279, (valorpot+256)*V_MAX/1279); //Saturación de la velocidad final

//	wd=(1000*realV-(correctR*DISEJES/2))/RADIO;
//	wi=(1000*realV+(correctR*DISEJES/2))/RADIO;//Velocidades angulares para cada rueda
//	
//	if(wd==0)														//En lazo cerrado se asignan directamente a los motores con la función correspondiente
//		realD=0;
//	else
//		realD=Velocidad_datomotor(wd);
//	if(wi==0)
//		realI=0;
//	else
//		realI=Velocidad_datomotor(wi);
//	
///////////////////////////////////////
////         Transiciones            //
///////////////////////////////////////
//	switch(digestado){
//		case PARADO://PARADO: La propia palabra lo dice
//			if((valorx>0x1E0)&&(valorx<0x220))//Eje adelante-atrás centrado
//				FlagBloqueo=0;
//			if(!FlagBloqueo){
//				if(valorx>0x220)
//					digestado=ACELFOR;
//				else if (valorx<0x1E0)
//					digestado=ACELBACK;				
//			}
//			break;
//		case ACELFOR://Acelerando hacia adelante
//			if((valorx>0x1E0)&&(valorx<0x220))//Eje adelante-atrás centrado
//				digestado=VCTEFOR;
//			else if(valorx<0x1E0)
//				digestado=DECELFOR;
//			break;
//		case VCTEFOR://Velocidad constante hacia adelante
//			if(valorx>0x220)
//				digestado=ACELFOR;
//			else if(valorx<0x1E0)
//				digestado=DECELFOR;
//			break;
//		case DECELFOR://Deceleración con velocidad positiva
//			if(Signo(Vanterior)!=Signo(realV)){//Asegurando que no sobrepase el margen de parada
//				digestado=PARADO;
//				FlagBloqueo=1;
//			}
//			else if((valorx>0x1E0)&&(valorx<0x220))//Joystick en reposo
//				digestado=VCTEFOR;
//			else if(valorx>0x220)
//				digestado=ACELFOR;
//			break;	
//		case ACELBACK://Aceleración hacia atrás
//			if((valorx>0x1E0)&&(valorx<0x220))
//				digestado=VCTEBACK;
//			else if(valorx>0x1E0)
//				digestado=DECELBACK;
//			break;
//		case VCTEBACK://Velocidad constante hacia atrás
//			if(valorx>0x220)
//				digestado=DECELBACK;
//			else if(valorx<0x1E0)
//				digestado=ACELBACK;
//			break;
//		case DECELBACK:
//			if(Signo(Vanterior)!=Signo(realV)){//Asegurando que no sobrepase el margen de parada
//				digestado=PARADO;
//				FlagBloqueo=1;
//			}
//			else if((valorx>0x1E0)&&(valorx<0x220))//Joystick en reposo
//				digestado=VCTEBACK;
//			else if(valorx<0x1E0) //Joystick hacia atrás
//				digestado=ACELBACK;
//			break;
//		}
//	Vanterior=realV;    //Memorizamos la velocidad del instante anterior
//	correctV1=correctV;//
//}

/******************************************************************************************************/
/*********************   FIN   *****Control joystick******   FIN   ************************************/
/******************************************************************************************************/

/******************************************************************************************************/
/********************************Control soplido*******************************************************/
/******************************************************************************************************/

/******************************************************************************************/
// Conversion_Soplido(): Traduce el valor del soplido según el estado para los motores
/******************************************************************************************/
//void Conversion_Soplido(void){
//	int val;				
///////////////////////////////////////////////////////////////////////////////////////////	
////           Máquina de estados para interpretar el dato del ADC y determinar            //
////	  A(Soplo normal),B(Aspiración normal),MA(Soplo fuerte),MB(Aspiración fuerte))       //
///////////////////////////////////////////////////////////////////////////////////////////
//	switch (estado){
//		case CERO:
//			Dato=NULO;
//			tiempo=0;
//			val=valorsop;
//			
//			if ((val>0x2AA)&&(val<0x3E0)){
//				entrada=A;
//				estado=PREGUNTA;
//			}else if(val>=0x3E0){
//				entrada=MA;
//				estado=PREGUNTA;
//			}else if ((val<0x180)&&(val>0x50)){
//				entrada=B;
//				estado=PREGUNTA;
//			}else if (val<=0x50){
//				entrada=MB;
//				estado=PREGUNTA;
//			}else{
//				entrada=NULO;
//				estado=CERO;	 
//			}	
//			break;
//		case PREGUNTA:
//			val=valorsop;
//			tiempo++;
//			if (tiempo>=2)
//				estado=UNICO_E;
//			switch (entrada){
//				case MA:
//					if(val<0x3E0){
//						estado=CERO;
//						tiempo=0;
//						entrada=NULO;
//					}
//					break;
//				case MB:
//					if(val>0x50){
//						estado=CERO;
//						tiempo=0;
//						entrada=NULO;
//					}
//					break;
//				case A:
//					if(val>=0x3E0){
//						entrada=MA;
//					}else if(val<0x2AA){
//						estado=CERO;
//						tiempo=0;
//						entrada=NULO;
//					}
//					break;
//				case B:
//					if(val<=0x50){
//						entrada=MB;
//					}else if(val>0x180){
//						estado=CERO;
//						tiempo=0;
//						entrada=NULO;
//					}
//					break;
//				case NULO:
//					break;
//			}
//			break;
//		case UNICO_E:
//			val=valorsop;
//			switch (entrada){
//				case MA:
//					if(val<0x3E0){
//						Dato=NULO;
//						estado=CERO;
//						tiempo=0;
//						entrada=NULO;
//					}else{
//						Dato=MA;
//						estado=UNICO_E;
//					}
//					break;
//				case MB:
//					if(val>0x50){
//						estado=CERO;
//						tiempo=0;
//						entrada=NULO;
//						Dato=NULO;
//					}else{
//						Dato=MB;
//						estado=UNICO_E;
//					}
//					break;
//				case A:
//					if(val<0x2AA){
//						estado=CERO;
//						tiempo=0;
//						entrada=NULO;
//						Dato=NULO;
//					}else{
//						Dato=A;
//						estado=UNICO_E;
//					}
//					break;
//				case B:
//					if(val>0x180){
//						estado=CERO;
//						tiempo=0;
//						entrada=NULO;
//						Dato=NULO;
//					}else{
//						Dato=B;
//						estado=UNICO_E;
//					}
//					break;
//				case NULO:
//					break;
//			}
//		break;
//	}
//	
//////////////////////////////////////////////////////////////////////////////////////
////    Maquina de estados para decidir la velocidad a través del dato determinado    //
//////////////////////////////////////////////////////////////////////////////////////
//	switch (situacion){											  
//		case STOP:
//			if(Dato!=NULO)		
//				situacion=ANDANDO;
//			else
//				situacion=STOP;
//			break;
//		case ANDANDO:
//			switch (Dato){
//					case MA://Soplido fuerte					
//						inputR=0;
//						if(inputV>=0){
//							inputV+=INCREMENTO_SOPLIDO*TS_MSEGUNDOS/100;
//						}else if(realV<0){
//							inputV+=DECREMENTO_SOPLIDO*TS_MSEGUNDOS/100;
//							if(inputV>=0){
//								inputV=0;
//								situacion=PARANDO;
//							}
//						}
//						break;
//					case MB://Aspiración fuerte						
//						inputR=0;
//						if(inputV<=0){//Si velocidad negativa: Mayor marcha atrás
//							inputV-=INCREMENTO_SOPLIDO*TS_MSEGUNDOS/100;
//							
//						}else if(inputV>0){//Si velocidad positiva: Frenando
//							inputV-=DECREMENTO_SOPLIDO*TS_MSEGUNDOS/100;
//							if(inputV<=0){
//								inputV=0;
//								situacion=PARANDO;
//							}
//						}
//						break;
//					case A://Soplido débil						
//						inputR=GIRO_SOPLIDO;
//						break;
//					case B:	//Aspiración débil					
//						inputR=-GIRO_SOPLIDO;
//						break;
//					case NULO:
//						inputR=0;
//						break;
//			}
//			break;
//		case PARANDO:
//			if(Dato==NULO)
//				situacion=STOP;
//			break;
//	}
//	Sensores();
//	if (correctV>correctV1)
//		correctV+=(inputV-correctV)*TS_MSEGUNDOS/8000; //Si la nueva corregida es mayor que la anterior hacemos un suave incremento
//                                                  //Iguala a la memorizada en 8 segundos
//	realV=correctV;
//	realV=Saturacion(realV, -(valorpot+256)*V_MAX/1279, (valorpot+256)*V_MAX/1279); //Saturación de la velocidad final
//		
//	wd=(1000*realV-(correctR*DISEJES/2))/RADIO;
//	wi=(1000*realV+(correctR*DISEJES/2))/RADIO;//Velocidades angulares para cada rueda
//	
//	realD=Velocidad_datomotor(wd);
//	realI=Velocidad_datomotor(wi);

//correctV1=correctV;	
//}

/******************************************************************************************************/
/***************** FIN ************** Control soplido ********************* FIN ***********************/
/******************************************************************************************************/
