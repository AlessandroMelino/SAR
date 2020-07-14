///******************************************************************************/
///*  This file is part of the uVision/ARM development tools                    */
///*  Copyright KEIL ELEKTRONIK GmbH 2002-2004                                  */
///******************************************************************************/
///*                                                                            */
///*  SERIAL.C:  Low Level Serial Routines                                      */
///*                                                                            */
///******************************************************************************/

//#include <LPC21xx.H>                     /* LPC21xx definitions               */

//#define CR   0x0D

//char mensaje[256],echo[256],gotchars[256];
//unsigned char mensajei, echoi,gotcharsi;
///**********************************************/
////	init_serial(): Inicialización del serial
///**********************************************/
//void init_serial (void)  {               /* Initialize Serial Interface       */
//  
//  PINSEL0 = 0x00000005;                  /* Enable RxD0 and TxD0              */
//  U0LCR = 0x8A;                          /* 8 bits, no Parity, 1 Stop bit     */
//  U0DLL = 0x84;
//  U0DLM = 0x1;                            /* 9600 Baud Rate @ 15MHz VPB Clock  */
//  U0LCR = 0x1A;                          /* DLAB = 0                          */
//}

///**************************************************/
////	int miputchar(int): Inicialización del serial
///**************************************************/
//void miputchar (int ch)  {                  /* Write character to Serial Port    */
// 
//  if (ch == '\n')  {
//    while (!(U0LSR & 0x20));
//		U0THR = CR;                          /* output CR */
//		while (!(U0LSR & 0x01));
//		echo[echoi]=U0RBR;
//		echoi++;
//		
//  }
//	else{
//		while (!(U0LSR & 0x20));
//		U0THR = ch;               //Cada caracter enviado a la AX3500 se devuelve para confirmar
//		while (!(U0LSR & 0x01));
//		echo[echoi]=U0RBR;
//		echoi++;
//		
//	}
//}


//char migetchar (void)  {                    /* Read character from Serial Port   */

//  while (!(U0LSR & 0x01));
//	gotchars[gotcharsi]=U0RBR;
//	gotcharsi++;
//  return (gotchars[gotcharsi-1]);
//  
//}

//void puthex (int hex){							/*mando por el serial un digito hexadecimal*/
//	
//	if (hex > 9) miputchar('A' + (hex - 10));
//	else		 miputchar ('0' + hex);
//}
///**********************************************************************/		 
///**     Funciones de comandos del motor,utilizadas en funciones.c    **/		 
///**********************************************************************/
///**      Comandos elementales     **/
///***********************************/
///**    !C\n Desactiva el freno	  **/
///**      !c\n Activa el freno	    **/
///**      !Axx\n MotorD forward	  **/
///**      !axx\n MotorD backward   **/
///**      !c\n Activa el freno	    **/
///**      !Bxx\n MotorI forward	  **/
///**	  !bxx\n MotorI backward      **/
///**      xx == Valor de 2 byte	  **/
///***********************************/
//char Confirmacion_comando(void){
//	char confirmacion;
//		confirmacion=migetchar();
//		migetchar();//Después de la confirmación toma el retorno de carro
//	if (confirmacion=='+')
//		return 1;
//	else
//		return 0;
//}
//char init_powercard(void){
//	char exito=0;

//		miputchar('%');   //Comando de reset
//		miputchar('r');
//		miputchar('r');
//		miputchar('r');
//		miputchar('r');
//		miputchar('r');
//		miputchar('r');
//		miputchar('\n');
//		do{//El mensaje de bienvenida es la confirmacion de comando
//			mensaje[mensajei]=migetchar();
//			mensajei++;
//		}	while (mensaje[mensajei-1] != 'K');//Último caracter legible enviado en mensaje de bienvenida
//		mensaje[mensajei]=migetchar();//Termina con un 0x0D, lo esperamos
//		while(!exito){
//			//Base de tiempo = 2, esencial para un correcto funcionamiento en lazo cerrado
//			miputchar('*');
//			miputchar('A');
//			miputchar('2');
//			miputchar(' ');
//			miputchar('0');
//			miputchar('2');    // *A2 02 y *A3 02 configuran las bases de tiempo de encoder
//			miputchar('\n');
//			exito=Confirmacion_comando();
//		}
//		exito=0;
//		while(!exito){
//			miputchar('*');
//			miputchar('A');	
//			miputchar('3');
//			miputchar(' ');
//			miputchar('0');
//			miputchar('2');    // *A2 03 y *A3 03 configuran las bases de tiempo de encoder
//			miputchar('\n');
//			exito=Confirmacion_comando();
//		}
//		exito=0;//Tras cada comprobacion de exito, lo ponemos a 0.
//		while(!exito){
//			miputchar('*');
//			miputchar('A');	
//			miputchar('3');
//			miputchar(' ');
//			miputchar('0');
//			miputchar('2');    // *A2 03 y *A3 03 configuran las bases de tiempo de encoder
//			miputchar('\n');
//			exito=Confirmacion_comando();
//		}
//		exito=0;//Tras cada comprobacion de exito, lo ponemos a 0.
//		while(!exito){
//			miputchar('^');
//			miputchar('8');	
//			miputchar('6');
//			miputchar(' ');
//			miputchar('0');
//			miputchar('0');    // ^86 00 y ^87 00 ponen a cero la ganancia diferencial -> Comportamiento más suave en lazo cerrado
//			miputchar('\n');
//			exito=Confirmacion_comando();
//		}
//		exito=0;//Tras cada comprobacion de exito, lo ponemos a 0.
//		while(!exito){
//			miputchar('^');
//			miputchar('8');	
//			miputchar('7');
//			miputchar(' ');
//			miputchar('0');
//			miputchar('0');    // ^86 00 y ^87 00 ponen a cero la ganancia diferencial -> Comportamiento más suave en lazo cerrado
//			miputchar('\n');
//			exito=Confirmacion_comando();
//		}
//		return 1;//Todo ha sido ejecutado
//		exito=0;
//}

//void Avanza_D (int dato){
//	char exito=0;
//	while(!exito){
//		miputchar('!');			                                          		 
//		miputchar('A');															 
//		puthex((dato>>4)&0x0F);													 
//		puthex((dato)&0x0F);
//		miputchar('\n');
//		exito=Confirmacion_comando();
//	}
//	exito=0;
//}

//void Avanza_I (int dato){
//	char exito=0;
//	while(!exito){
//		miputchar('!');
//		miputchar('B');
//		puthex((dato>>4)&0x0F);
//		puthex((dato)&0x0F);
//		miputchar('\n');
//		exito=Confirmacion_comando();
//	}
//	exito=0;
//}

//void Retrocede_D (int dato){
//	char exito=0;
//	dato=-1*dato;
//	while(!exito){
//		miputchar('!');
//		miputchar('a');
//		puthex((dato>>4)&0x0F);
//		puthex((dato)&0x0F);
//		miputchar('\n');
//		exito=Confirmacion_comando();
//	}
//	exito=0;
//}

//void Retrocede_I (int dato){
//	char exito=0;
//	dato=-1*dato;
//	while(!exito){
//		miputchar('!');
//		miputchar('b');
//		puthex((dato>>4)&0x0F);
//		puthex((dato)&0x0F);
//		miputchar('\n');
//		exito=Confirmacion_comando();
//	}
//	exito=0;
//}
//void Frena (void){
//	char exito=0;
//	while(!exito){
//     miputchar('!');
//		 miputchar('c');	  
//		 miputchar('\n');
//		exito=Confirmacion_comando();
//	}
//	exito=0;
//}
//void Quitafreno (void){
//	char exito=0;
//	while(!exito){
//		miputchar('!');
//		miputchar('C');	  
//		miputchar('\n');
//		exito=Confirmacion_comando();
//	}
//	exito=0;
//}
//void Configura_Lazoabierto(void){
//	char exito=0;
//	//Lazo abierto motor 1
//	while(!exito){
//		miputchar('^');
//		miputchar('8');
//		miputchar('0');
//		miputchar(' ');
//		miputchar('0');
//		miputchar('0');    //^80 00 y ^81 00 configuran lazos cerrados con realimentación de encoder
//		miputchar('\n');
//		exito=Confirmacion_comando();
//	}
//	exito=0;
//	//Lazo abierto motor 2
//	while(!exito){
//		miputchar('^');
//		miputchar('8');
//		miputchar('1');
//		miputchar(' ');
//		miputchar('0');
//		miputchar('0');    //^80 00 y ^81 00 configuran lazos cerrados con realimentación de encoder
//		miputchar('\n');
//		exito=Confirmacion_comando();
//	}
//	exito=0;
//}
//void Configura_Lazocerrado(void){
//	char exito=0;
//	//Lazo cerrado motor 1
//	while(!exito){
//		miputchar('^');
//		miputchar('8');
//		miputchar('0');
//		miputchar(' ');
//		miputchar('0');
//		miputchar('5');    //^80 00 y ^81 00 configuran lazos cerrados con realimentación de encoder
//		miputchar('\n');
//		exito=Confirmacion_comando();
//	}
//	exito=0;
//	//Lazo cerrado motor 2
//	while(!exito){
//		miputchar('^');
//		miputchar('8');
//		miputchar('1');
//		miputchar(' ');
//		miputchar('0');
//		miputchar('5');    //^80 00 y ^81 00 configuran lazos cerrados con realimentación de encoder
//		miputchar('\n');
//		exito=Confirmacion_comando();
//	}
//	exito=0;
//}
