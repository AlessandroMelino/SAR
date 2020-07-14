#include <LPC21xx.H>
#include "funciones.h"
#include "timer.h"
#include "LPC_FullCAN_SW.h"
#include "math.h"
//#include "serial.h"

signed char velI,velD;
//int SDI,SDD,STI,STD,SLDD,SLDT,SLID,SLIT,SJO;//Sensores	de posición
//int alarma=0;		//alarma de colisión
//short EJEX,EJEY,EJEZ;//Ejes del acelerómetro (Picos)

//char Cambiomodo=0;//Flag que se enciende en cambios de modo para notificar envío a PC por CAN
	
int modo=0,modoantes=0; //Indican el modo actual, y el anterior en los instantes que se cambia el modo por el teclado.
int valorpot, valorx, valory,valorsop;//Valores tomados de los ADC
int ejey,ejex; //Ejes joystick ajustados con zona muerta, dan un valor positivo o negativo, en código. 		
int inputV=0, correctV, correctV1, realV=0, inputR, correctR;           //V lineal y angular
int wd, wi;                        //V motores en mrad/s          
int realD,realI;                   //V motores en código a interpretar por la tarjeta
char Parada;
//int nivel_bateria;
//char FlagBloqueo=0;//Flag del modo JDigital monitorizado en Display

                                                      // Estados de
enum tipo_de_estado_PC estado_PC= SINCRONIZANDO;     //     PC    (tipo de enum en funciones.h)
char Flag_sincronia_PC;

//Encoders absolutos, tiempos absolutos, con sus respectivos anteriores
int encoderDabs=0,tDabs=1000,prevencoderDabs=0,prevtDabs=0, encoderIabs=0,tIabs=1000,prevencoderIabs=0,prevtIabs=0;
//Valores incrementales
int encoderD=0, encoderI=0, tD=1000, tI=1000;
//Número de encoders leído
char EncoderUpdate=0; //1 para un encoder, 2 para los 2, cuando llega a 2 se emparejan sus valores y se actualiza el grid map.
	
int PCmensajeA=0,PCmensajeB=0;//Mensaje CAN que llega desde el PC, para depuración

//UNIDADES DE LAS V: mm/s
//UNIDADES DE inputR, wd y wi: mrad/s

// unsigned char Vmodo[7][20]=   {"Pulse 0 cambiar modo",//Cada fila lleva su modo (1-Joystick,2-Soplido,...)
//                                "Modo Joystick       ",
//															  "Modo Soplido        ",
//                                "Modo PC             ",
//															  "Modo Joyst. Digital ",
//	                              "#.-Activar sens     ",
//                                "*.-Desactivar sens  "};
// 
// unsigned char Vdefault[4][20]={"  -SARA2-  Bat:....V",//En modo menú, Vdisplay en su inicialización
//	                              "Seleccione modo:    ",
//								                " 1.-Joystick  3.-PC ",
//						  		              " 2.-Soplido 4.-J.Dig"};
// 
//  unsigned char Vpc[4][20]=    {" Sincronizando...   ", //SINCRONIZANDO
//	                              " No se detecta App  ", //NO SINCRONISMO
//								                " Sincronizado       "};//SINCRONISMO		
unsigned char sensoresON=0;		
//double medida[]=        {    0,    0,    0,    0,    0,      0,    0,   0,   0};//Aquí se guardan las últimas medidas de los sensores
//double posx[]=          {xSLDD,xSLIT,xSLDT,xSLID, xSTD,   xSDI, xSTI,xSDD,xSJO};//Posición en x de cada sensor en mm(Origen centro eje)
//double posy[]=          {ySLDD,ySLIT,ySLDT,ySLID, ySTD,   ySDI, ySTI,ySDD,ySJO};//Posición en y de cada sensor (Origen centro eje)
//double orientacion[]=   {    0,   PI,    0, 	PI,-PI/2,3*PI/40,-PI/2,PI/2,PI/4};//Orientación de cada sensor   (Origen derecha)
//double obsx[]=          {    0,    0,    0,    0,    0,      0,    0,   0,   0};
//double obsy[]=          {    0,    0,    0,    0,    0,      0,    0,   0,   0};
//double xvectorforce[]=  {    0,    0,    0,    0,    0,      0,    0,   0,   0};
//double yvectorforce[]=  {    0,    0,    0,    0,    0,      0,    0,   0,   0};
//double modvectormult[]= {    0,    0,    0,    0,    0,      0,    0,   0,   0};
//double modvectorforce[]={    0,    0,    0,    0,    0,      0,    0,   0,   0};
//double xrepulsion=0;
//double yrepulsion=0;
//double speed=0, alpha=ALPHA, beta=BETA, omega=OMEGA;
//double wallangle;
//int gridmap[ROWS][COLUMNS]={0};
//double rounderrorsx[ROWS*COLUMNS],rounderrorsy[ROWS*COLUMNS];
/*******************************/
// int abs(int): Valor absoluto 
/*******************************/
int abs(int numero){
	if (numero>=0)
		return numero;
	else
		return -numero;
}

/*******************************************************/
// char Signo(int): Si == 0:' ' Si > 0:'+'  Si < 0:'-' 
/*******************************************************/
char Signo(int numero){
	if (numero>0)
		return '+';
	else if (numero==0)
		return ' ';
	else
		return '-';
}

///************************************************************************************************************/
//// double Regresion(char x, char y): Devuelve el término de grado 1 de la regresión lineal de los puntos x, y 
///************************************************************************************************************/
//double Regresion(int px[], int py[],int npoints){
//	int xsum=0,squaresum=0,ysum=0,multsum=0;
//	double m,xmean=0,ymean=0;
//	double squaremean=0,multmean=0,covarianza,varianzax;
//	int n;
//	for (n=0;n<npoints;n++){
//			xsum+=px[n];
//			squaresum+=(px[n]*px[n]);
//			ysum+=py[n];
//			multsum+=px[n]*py[n];
//	}
//	squaremean=(double)squaresum/(double)npoints;//Depurando, en estas lineas también se me modifica xmean ¡No lo entiendo!
//	ymean=(double)ysum/(double)npoints;
//	multmean=(double)multsum/(double)npoints;
//	xmean=(double)xsum/(double)npoints;//Solucion provisional: Lo calculo lo último
//	covarianza=multmean-(xmean*ymean);//Como es muy cachondo y me modifica xmean aquí también
//	xmean=(double)xsum/(double)npoints;//Lo tendré que recalcular
//	varianzax=squaremean-(xmean*xmean);
//	m=covarianza/varianzax;
//	return m;
//}

/*******************************************************/
// int Saturacion(int num,int liminf,int limsup) 
/*******************************************************/
int Saturacion(int num,int liminf,int limsup){
	if (num>limsup)
		return limsup;
	else if (num<liminf)
		return liminf;
	else return num;
}

/***********************************************************************************************/
// double maxpos(double[],int): Retorna el máximo positivo, si no hay máximo positivo retorna 0. 
/***********************************************************************************************/
double maxpos(double arr[], int size){
	double maxval;
	int index;
		maxval=arr[0];
		for(index=1;index<size;index++)
		{
			if (arr[index]>maxval)
				maxval=arr[index];
		}
		if(maxval>0)
			return maxval;
		else
			return 0;
}

/***********************************************************************************************/
// double minneg(double[],int): Retorna el mínimo negativo, si no hay mínimo negativo retorna 0. 
/***********************************************************************************************/
double minneg(double arr[], int size){
	double minval;
	int index;
		minval=arr[0];
		for(index=1;index<size;index++)
		{
			if (arr[index]<minval)
				minval=arr[index];
		}
		if(minval<0)
			return minval;
		else
			return 0;
}

/*********************************************************************/
// config(): Inicialización de variables, configuración de pines... 
/*********************************************************************/
void config(void){
	//estado=CERO;   //	Variables inicializacion soplido
	//situacion=STOP;	   //	Variables inicializacion soplido
	/*configuracion e inicializacion de los distintos dispositivos*/
	IODIR0|=(1<<10)|(1<<15)|(1<<17)|(1<<21); //Pines como salida
	IOSET0|=(1<<15);	//Señal de ON por P0.15
	// No divider: peripheral clock = processor clock
  	VPBDIV = 1; 
    // Init Vector Interrupt Controller
  	VICIntEnClr = 0xFFFFFFFFL; // Disable all Ints
  	VICIntSelect = 0x00000000L;
	
	if((IOPIN0>>16)&1)
		modo=MODO_JOYSTICK;
	else if((IOPIN0>>20)&1)
		modo=MODO_PC;
}
/*********************************************************************/
// init_can(): Inicialización del BUS CAN
/*********************************************************************/
void init_can (void){
	int i;

	// Initialisation of CAN interfaces
  	// CAN interface 1, use IRQVec0, at 1 Mbit
  	FullCAN_Init(1,0,CANBitrate001m_12MHz); 
  	//Set CAN Err ISR to IRQVec2
  	FullCAN_SetErrIRQ(2);
	
//		FullCAN_SetFilter(1,0x101);//Motores tDabs
//		FullCAN_SetFilter(1,0x102);//Motores tIabs
		FullCAN_SetFilter(1,0x120);//PC
//  	FullCAN_SetFilter(1,0x210);//Bateria
//		FullCAN_SetFilter(1,0x110);//Joystick
//		FullCAN_SetFilter(1,0x111);//modo(Joystick)
//		FullCAN_SetFilter(1,0x201);//SensoresA
//		FullCAN_SetFilter(1,0x202);//SensoresB
//		FullCAN_SetFilter(1,0x203);	//SensoresC
	
//		 DEBUG HELP:
//  Wait for 10 milliseconds, to allow debug hardware to catch up
  	for(i=0;i<200000;i++);
}

/*********************************************************************/
// Can_envio(): Envío de datos por el bus CAN
/*********************************************************************/
void Can_Envio(void){	
	short poty,potx,modo_envio;
	FULLCAN_MSG MsgBuf;

	velI=(char)realI;
	velD=(char)realD;
	poty=(char)valory;
	potx=(short)valorx;
	modo_envio=(short)modo;

	MsgBuf.Dat1 = 0x00080110L;
  MsgBuf.DatA = (((potx<<16)&0xFFFF0000)|((velI<<8)&0x0000FF00)); //Valorx del joystick y dato de velocidad I en los MSB del msjA
	MsgBuf.DatA = (MsgBuf.DatA|(velD&0xFF)); // Dato de velocidad D en los LSB del msjA
	MsgBuf.DatB = (((poty<<16)&0xFFFF0000)|(modo_envio&0xFFFF)); // Valory del joystick en los MSB del msjB
  // Transmit initial message on CAN 1  
 	FullCAN_PushMessage(1,&MsgBuf);
}

/*********************************************************************/
// Can_Modo_aPC(): Envío del modo por el BUS CAN sólo al PC
/*********************************************************************/
void Can_Modo_aPC(void){
	FULLCAN_MSG MsgBuf;
	MsgBuf.Dat1 = 0x00080111L;
  MsgBuf.DatB= 0; // Vacío el mensaje B
	MsgBuf.DatA = modo; //  envio modo//	0 Menu/1Joystick/2Soplido/3Pc/4JDigital/>4depuracion
	//Si el modo es 3 (Modo PC), el PC debe devolver un mensaje para confirmarlo.
  // Transmit initial message on CAN 1 
 	FullCAN_PushMessage(1,&MsgBuf);
}

/*********************************************************************/
// Can_Lectura(): Toma de datos del BUS CAN
/*********************************************************************/
void Can_Lectura(void){
	FULLCAN_MSG MsgBuf; // Buffers one CAN message

 	// Check if message received on CAN 1
    if (FullCAN_PullMessage(1,&MsgBuf)){ // Message was received  	
			switch ((MsgBuf.Dat1 & 0x07FF)){
//				case 0x201:
//					SLDD=((MsgBuf.DatB>>16)&0xFFFF);	  //guardamos valores
//					SLIT=(MsgBuf.DatB&0xFFFF);
//					SDI=((MsgBuf.DatA>>16)&0xFFFF);
//					STD=(MsgBuf.DatA&0xFFFF);
//					break;
//				case 0x202:
//					SLDT=((MsgBuf.DatB>>16)&0xFFFF);      //guardamos valores
//					STI=(MsgBuf.DatB&0xFFFF);
//					SLID=((MsgBuf.DatA>>16)&0xFFFF);
//					SDD=(MsgBuf.DatA&0xFFFF);
//					break;
//				case 0x203:
//					EJEX=((MsgBuf.DatB>>16)&0xFFFF);	  //guardamos valores
//					EJEY=(MsgBuf.DatB&0xFFFF);
//					EJEZ=(((int)MsgBuf.DatA>>16)&0xFFFF);
//					SJO=(MsgBuf.DatA&0xFFFF);
//					break;
//				case 0x101:
//					encoderDabs=(MsgBuf.DatA);
//					tDabs=(MsgBuf.DatB);
//					encoderD=encoderDabs-prevencoderDabs;
//					tD=tDabs-prevtDabs;
//					prevencoderDabs=encoderDabs;
//					prevtDabs=tDabs;
//					if(EncoderUpdate==1)//Si se ha leido el otro encoder
//						EncoderUpdate=2;//Marcamos los dos leidos
//					else if (EncoderUpdate==2)//Si ya se habían leido los dos y no se ha atendido externamente
//						EncoderUpdate=1;//Marcamos solo este leido,  para emparejar con el siguiente
//					else if (EncoderUpdate==0)//Si no se ha leido ningún encoder todavía 
//						EncoderUpdate=1;//Marcamos leído uno de ellos
//					break;
//				case 0x102:
//					encoderIabs=(MsgBuf.DatA);
//					tIabs=(MsgBuf.DatB);
//					encoderI=encoderIabs-prevencoderIabs;
//					tI=tIabs-prevtIabs;
//					prevencoderIabs=encoderIabs;
//					prevtIabs=tIabs;
//					break;
//				case 0x210:
//					nivel_bateria = (MsgBuf.DatA);
//					break;

				case 0x120:
					PCmensajeA=((MsgBuf.DatA)&0xFFFFFFFF);
					PCmensajeB=((MsgBuf.DatB)&0xFFFFFFFF);
					Flag_sincronia_PC=1;
				break;
				
				default:break;
			}
		}// Message on CAN 1 received   
}
/*********************************************************************/
// Borrar_Vdisplay(): Vacía el display virtual
/*********************************************************************/
//void Borrar_Vdisplay(void)
//{
//	int i,j;
//	for(i=0;i<4;i++){
//		for(j=0;j<20;j++)
//			Vdisplay[i][j]=0x20;
//	}
//}
/*********************************************************************/
// Vdisplay_bateria(int): Actualiza el nivel de la batería
/*********************************************************************/
//void Vdisplay_bateria (int nivel_bateria){
//	int i;
//	for (i=0;i<15;i++)
//	Vdisplay[0][15]=0x30+(nivel_bateria/100);	// decenas
//	Vdisplay[0][16]=0x30+((nivel_bateria/10)%10);  // unidad
//	Vdisplay[0][17]=0x2E; // caracter '.'
//	Vdisplay[0][18]=0x30+(nivel_bateria%10);	// decimal
//	Vdisplay[0][19]=0x56; // caracter 'V'
//}
/******************************************************************************************/
// Procesa_Teclado(void): Realiza las transiciones de modo con el valor tomado del teclado
/******************************************************************************************/
//char Procesa_Teclado(void)//Cambios de modo
//{	
//	modoantes=modo;
//	switch (valor_teclado){
//		/////////////////////
//		case(1<<10)://Tecla 0
//			modo=0;//Menú
//			break;
//		/////////////////////
//		case (1<<0)://Tecla 1
//			if (modo==0)
//				modo=1;//Joystick
//			else if (modo==9)
//				alpha+=0.05;
//			break;
//		/////////////////////
//		case(1<<1)://Tecla 2
//			if (modo==0)
//				modo=2;//Soplido
//			else if (modo==9)
//				beta+=0.05;
//			break;
//		/////////////////////
//		case(1<<2)://Tecla 3
//			if (modo==0)
//				modo=3;//PC
//			else if (modo==9)
//				omega+=0.05;
//			break;
//		/////////////////////
//		case(1<<3)://Tecla 4
//			if (modo==0)
//				modo=4;//Joystick digital
//			else if (modo==9)
//				alpha-=0.05;
//			break;
//		/////////////////////
//		case(1<<4)://5
//			if (modo==9)
//				beta-=0.05;
//			break;
//	  /////////////////////
//		case(1<<5)://6
//			if (modo==9)
//				omega-=0.05;
//			break;
//		/////////////////////
//		case(1<<6)://7
//			if (modo>4)
//				modo=7;
//			break;
//		/////////////////////
//		case(1<<7)://8
//			if (modo>4)
//				modo=8;
//			break;
//		/////////////////////
//    case(1<<8)://9
//			if (modo>4)
//				modo=9;
//			break;
//		/////////////////////	
//		case(0xA00):// # + *
//			if (modo==0)
//				modo=9;//Modo 9, pantalla de depuración de tunning del control por sensores, modo 5 y superiores, modos de depuración
//			break;
//		/////////////////////
//		case(1<<9)://*
//			if(modo==1||modo==4||modo==2)
//				if (sensoresON==1)
//				sensoresON=0;
//			break;
//		/////////////////////
//		case(1<<11)://#
//			if(modo==1||modo==4||modo==2)
//				if (sensoresON==0)
//				sensoresON=1;
//			break;
//		/////////////////////
//		default: break;
//		/////////////////////
//	}
//	if (modoantes!=modo){
//		Borrar_Vdisplay();
//		return 1;
//	}
//	else return 0;
//}
/******************************************************************************************************************************/
// Variable_Depuracion(char,int,int,int,short): Visualiza una variable entera en pantalla (en realidad en Vdisplay)
// 	Se ha de especificar(cual es la variable,con que letra representarla,en que fila,que columna, cuantas cifras,¿tiene signo?)
// 	Distingue variables de 3, 4 y 9 cifras.
/******************************************************************************************************************************/
//void Variable_Depuracion(int variable,char letra,int fila,int columna,int cifras,short signo){
//	if((cifras>9)||(cifras<1)||((columna+cifras+signo+1)>20)||(signo>1)||(signo<0)||(fila>3))
//		Vdisplay[fila][columna]='#';//Si representa esto, se hecho algo mal 
//	else{//Ejecución normal
//		Vdisplay[fila][columna]=letra; //Primero la letra que hemos escogido
//		Vdisplay[fila][columna+1]='=';//Después un igual
//		if (signo==1) //Si tiene signo, lo representamos
//				Vdisplay[fila][columna+2]=Signo(variable);//No confundir variable signo con funcion Signo
//		if (cifras==1)
//			Vdisplay[fila][columna+2+signo]=abs(variable)+0x30;//Miles
//		else if(cifras==3){//Caso de 3 cifras
//			Vdisplay[fila][columna+2+signo]=((abs(variable)/100)%10)+0x30; //Centenas
//			Vdisplay[fila][columna+3+signo]=((abs(variable)/10)%100%10)+0x30; //Decenas
//			Vdisplay[fila][columna+4+signo]=(abs(variable)%1000%100%10)+0x30; //Unidades
//		}
//		else if(cifras==4){//Caso de 4 cifras
//      Vdisplay[fila][columna+2+signo]=(abs(variable)/1000)+0x30;//Miles
//			Vdisplay[fila][columna+3+signo]=((abs(variable)/100)%10)+0x30;//Centenas
//			Vdisplay[fila][columna+4+signo]=((abs(variable)/10)%100%10)+0x30;//Decenas
//			Vdisplay[fila][columna+5+signo]=((abs(variable))%1000%100%10)+0x30;//Unidades
//		}
//		else if(cifras==9){//Caso de 9 cifras
//			Vdisplay[fila][columna+2+signo]=( abs(variable)/100000000)+0x30;//Cifra más alta
//			Vdisplay[fila][columna+3+signo]=((abs(variable)/10000000)%10)+0x30;
//			Vdisplay[fila][columna+4+signo]=((abs(variable)/1000000)%100%10)+0x30;
//			Vdisplay[fila][columna+5+signo]=((abs(variable)/100000)%1000%100%10)+0x30;
//			Vdisplay[fila][columna+6+signo]=((abs(variable)/10000)%10000%1000%100%10)+0x30;
//			Vdisplay[fila][columna+7+signo]=((abs(variable)/1000)%100000%10000%1000%100%10)+0x30;
//			Vdisplay[fila][columna+8+signo]=((abs(variable)/100)%1000000%100000%10000%1000%100%10)+0x30;
//		  Vdisplay[fila][columna+9+signo]=((abs(variable)/10)%10000000%1000000%100000%1000%100%10)+0x30;
//		  Vdisplay[fila][columna+10+signo]=((abs(variable)/1)%100000000%10000000%1000000%100000%10000%1000%100%10)+0x30;//Unidades
//		}
//	}
//}

/******************************************************************************************/
// Actualiza_Vdisplay(): Escribe en el Vdisplay todo lo que se debe representar
/******************************************************************************************/
//void Actualiza_Vdisplay(void)
//{
//	int i;
//	
//	if (modo==0){
//			for(i=0;i<20;i++){
//				Vdisplay[0][i]=Vdefault[0][i];
//				Vdisplay[1][i]=Vdefault[1][i];//Vdefault es el menú
//				Vdisplay[2][i]=Vdefault[2][i];
//				Vdisplay[3][i]=Vdefault[3][i];
//			}
//	}
//	else if (modo==7){ //Pantalla de depuración
//	
//			Variable_Depuracion(SDI,'I',0,2,3,0);
//			Variable_Depuracion(SDD,'D',0,8,3,0);
//			Variable_Depuracion(SJO,'J',0,14,3,0);
//			Variable_Depuracion(SLID,'I',1,0,3,0);
//			Variable_Depuracion(SLDD,'D',1,14,3,0);//Esta pantalla representa los sensores distribuidos sobre ella como en la silla
//			Variable_Depuracion(SLIT,'I',2,0,3,0);
//			Variable_Depuracion(SLDT,'D',2,14,3,0);
//			Variable_Depuracion(STI,'I',3,4,3,0);
//			Variable_Depuracion(STD,'D',3,10,3,0);
//	}
//	else if (modo==8){ //Pantalla de depuración
//			Variable_Depuracion(EJEX,'X',1,0,4,1);
//			Variable_Depuracion(EJEY,'Y',1,9,4,1);
//			Variable_Depuracion(EJEZ,'Z',2,0,4,1);
//			Variable_Depuracion(alarma,'!',3,0,1,0);
//	}
//	else if (modo==9){//Pantalla de depuración para sintonizar el control de sensores
//			Variable_Depuracion(alpha*1000,'a',0,1,9,0);
//			Variable_Depuracion(beta*1000,'b',1,1,9,0);
//			Variable_Depuracion(omega*1000,'o',2,1,9,0);
////			Variable_Depuracion(180*wallangle/PI,'a',3,0,9,1);
//	}
//	else if (modo!=3) {//Modos de no depuración, no PC
//			for(i=0;i<20;i++){
//				Vdisplay[1][i]=Vmodo[modo][i];// Nombre
//				Vdisplay[2][i]=Vmodo[0][i];//Pulse 0 para volver al modo menu
//				Vdisplay[3][i]=Vmodo[5+sensoresON][i];//Si sensoresON pregunta desactivarlos, si OFF pregunta activarlos con #
//			}
//	}
//	else if (modo==3){
//			for(i=0;i<20;i++){
//				Vdisplay[1][i]=Vmodo[modo][i];// Nombre
//				Vdisplay[2][i]=Vmodo[0][i];//Pulse 0 para volver al modo menu
//				Vdisplay[3][i]=Vpc[estado_PC][i];
//			}
//	}
//	if (modo<5){//Si no estamos depurando
//		for(i=0;i<20;i++)
//			Vdisplay[0][i]=Vdefault[0][i];
//		Vdisplay_bateria(nivel_bateria);
//	}
//}

/******************************************************************************************************/
/******************************* Control de velocidad *************************************************/
/******************************************************************************************************/

/********************************************************************************************/
//	Sensores(): //Limitación de la velocidad por los sensores de ultrasonidos (NO COMPLETADA)
/********************************************************************************************/
void Sensores(){	  
//	int i;
//	double modvectormultx=0,modvectorforcex=0;
//	xrepulsion=0,yrepulsion=0;
	if (sensoresON){
//		speed=(2*PI*((double)RADIO/1000)*10000/(2*4*(double)PPR*(double)RT))*( ((double)encoderD/(double)tD)+((double)encoderI/(double)tI) );
//		medida[0]=SLDD, medida[1]=SLIT, medida[2]=SLDT, medida[3]=SLID, medida[4]=STD, medida[5]=SDI, medida[6]=STI, medida[7]=SDD, medida[8]=SJO;
//		for(i=0;i<9;i++){
//			if (i!=1 && i!=2 && i!=4 && i!=5)//Estos sensores nos e usaran en el control
//			{
//				if (medida[i]==0){
//					medida[i]=900;//Las medidas 0 no detectan obstaculo, por lo que asignamos una gran distancia
//				}
//				obsx[i]=posx[i]/1000+medida[i]/100*cos(orientacion[i]);//Posición x del objeto hallado por el sensor
//				obsy[i]=posy[i]/1000+medida[i]/100*sin(orientacion[i]);//Posición y del objeto hallado por el sensor
//				//modvectormult[i]=ALPHA*exp(-BETA*((medida[i]-(speed-48)*10)/100));
//				modvectormult[i]=alpha*exp(-beta*(medida[i]/100)+omega*speed);
//				modvectormultx=alpha*exp(-beta*(medida[i]/100));
//				if (modvectormult[i]>1.2)
//					modvectormult[i]=1.2;
//				else if (modvectormult[i]<-1.2)
//					modvectormult[i]=-1.2;
//				if (modvectormultx>1.2)
//					modvectormultx=1.2;
//				else if (modvectormultx<-1.2)
//					modvectormultx=-1.2;
//				modvectorforce[i]=modvectormult[i]*sqrt( pow((double)inputV*100/75,2)+pow((double)inputR,2) );
//				modvectorforcex= modvectormultx*sqrt( pow((double)inputV*100/75,2)+pow((double)inputR,2) );
//				xvectorforce[i]=-modvectorforcex*cos(atan2(obsy[i],obsx[i]));
//				yvectorforce[i]=-modvectorforce[i]*sin(atan2(obsy[i],obsx[i]));
//				//xrepulsion+=xvectorforce[i];
//				//yrepulsion+=yvectorforce[i];
//			}	
//		}   //Al salir del bucle ya tenemos los vectores de repulsión, que se han de sumar al comando de entrada   
//		xrepulsion=maxpos(xvectorforce,9)+minneg(xvectorforce,9);
//		yrepulsion=maxpos(yvectorforce,9)+minneg(yvectorforce,9);
//		correctV=(int)(inputV+yrepulsion/2);
//		correctR=(int)(inputR+xrepulsion); 
//		 //realV*=(1+yrepulsion)+ inputR*(yrepulsion);
//		 //inputR=inputR*(1+xrepulsion)+realV*(xrepulsion);
	}
	else{
		correctV=inputV;
		correctR=inputR; 
	}		
}
/******************************************************************************************/
//	Acelerometro(): //Limitación de la velocidad por el acelerómetro
/******************************************************************************************/
//void Acelerometro(void){	 
//	if ( sqrt(EJEX*EJEX+EJEY*EJEY+EJEZ*EJEZ) > 1000 ){//para choques diagonales
//		alarma=1;
//	}
//	if (alarma==1){			   //si alarma activada
//							   //detenemos el sistema
//			realD=0;
//			realI=0;
//		if ((inputV==0)&&(inputR==0))
//			alarma=0; 
//	}
//}

///*****************************************************************************************************/
////	RefreshGrid(): //Actualización del mapa de celdas generado por la lectura puntual de los sensores
///*****************************************************************************************************/
//void MoveGrid(){	  //EN DESARROLLO, POR AHORA SOLO SENSORES LATERALES PARA MANTENER RECTITUD EN PASILLOS
//	double recorridoD,recorridoI,desplazamiento,angulo;
//	double trueoldx,trueoldy;
//	int oldx,oldy;
//	double translationy;
//	int newx,newy;
//	double cosangulo,sinangulo;
//	double efectotraslacioneseny;
//	recorridoD=2*PI*(RADIO)*encoderD/(4000*RT*PPR);
//	recorridoI=2*PI*(RADIO)*encoderI/(4000*RT*PPR);
//	angulo=(recorridoD-recorridoI)*1000/((double)DISEJES);
//	desplazamiento=(recorridoI+recorridoD)/2; //Desplazamiento en m durante el tiempo de muestreo
//	speed=(2*PI*((double)RADIO/1000)*10000/(2*4*(double)PPR*(double)RT))*( ((double)encoderD/(double)tD)+((double)encoderI/(double)tI) );
//	translationy=desplazamiento;
//	cosangulo=cos(angulo);                //Precálculos con el fin de aligerar el código
//	sinangulo=sin(angulo);               //
//	efectotraslacioneseny=(translationy*cosangulo);
//	medida[0]=SLDD, medida[1]=SLIT, medida[2]=SLDT, medida[3]=SLID;
//	
//	
////////////////////////////////////////////////////////////
////     Transformación del grid (Traslación y Giro)      //
////////////////////////////////////////////////////////////
//    //newrounderrorx=zeros(1,dimrow*dimcol);
//    //newrounderrory=newrounderrorx;//Inicialización del tamaño
//    //oldmap=map;

//	for (newx=0;newx<COLUMNS;newx++){
//			for (newy=0;newy<ROWS;newy++){//For all the index combinations
//				trueoldx=((cosangulo*((double)newx-(double)CELLCENTERX)-sinangulo*((double)newy-(double)CELLCENTERY))+(double)CELLCENTERX)-rounderrorsx[newx*ROWS+newy];
//				trueoldy=((sinangulo*((double)newx-(double)CELLCENTERX)+cosangulo*((double)newy-(double)CELLCENTERY)+efectotraslacioneseny)+(double)CELLCENTERY)-rounderrorsy[newx*ROWS+newy];
//				oldx=trueoldx;
//				oldy=trueoldy;
////            //Round error handling
//				rounderrorsx[newx*ROWS+newy]=(double)oldx-trueoldx;
//				rounderrorsy[newx*ROWS+newy]=(double)oldy-trueoldy;
////            oldtruex=O(1);-oldrounderrorx(n); %Origin x correspondant point
////            oldx=round(oldtruex); %Rounded Origin x index
////            oldtruey=O(2);-oldrounderrory(n);%Origin y correspondant point
////            oldy=round(oldtruey); %Rounded Origin y index
////            newrounderrorx(n)=oldx-oldtruex; %New round error
////            newrounderrory(n)=oldy-oldtruey; %New round error
//            
//					if (oldx>=0 && oldx<COLUMNS && oldy>=0 && oldy<ROWS){
//							gridmap[newy][newx]=gridmap[oldy][oldx];
//							
//							if (gridmap[newy][newx]>0)
//									 gridmap[newy][newx]-=CONVERGESPEED;//Esta debería ser dependiente del avance de la silla para no olvidar estando parados
//							/////////////////////////
//					}
//					else 
//						gridmap[newy][newx]=0;//Refilling new map with no obstacles where there is no correspondant old
//				}
//		}
//	}		
//double RegresionGrid(){
//	int i,newx,newy,row,col;
//	int npoints; //Cantidad de puntos detectados y memorizados a la derecha de la silla
//	int x[ROWS],y[ROWS]; //Puntos detectados y memorizados a la derecha de la silla, como mucho habrá uno por fila
//	double p; //Pendiente de la recta hallada por la regresión.
//	for(i=0;i<4;i++){    
//		if (medida[i]==0){
//			medida[i]=900;//Las medidas 0 no detectan obstaculo, por lo que asignamos una gran distancia
//		}
//	}
////////////////////////////////////////////////////////////
////       Inclusión de nuevos puntos en el grid          //
////////////////////////////////////////////////////////////		
//	for(i=0;i<4;i++){
//		newx=(posx[i]+(medida[i]*10)*cos(orientacion[i]))*M2CELL/1000+CELLCENTERX;
//		newy=(posy[i]+(medida[i]*10)*sin(orientacion[i]))*M2CELL/1000+CELLCENTERY;
//		if (newx>=0 && newx<COLUMNS && newy>=0 && newy<ROWS)
//			gridmap[newy][newx]=100;//La incertidumbre de obstáculo es como un porcentaje
//	}
////////////////////////////////////////////////////////////
////         Identificación de ángulo de pared            //
////////////////////////////////////////////////////////////
//	npoints=0;
//	for(row=0;row<ROWS;row++){
//		for(col=(int)(xSLDT*M2CELL/1000+CELLCENTERX);col<COLUMNS;col++){
//			 if (gridmap[row][col]>0){ //Mayor que 0 se detectó algo ahí
//					npoints++; //Nuevo punto detectado
//					x[npoints-1]=col;
//					y[npoints-1]=row; //Puntos añadidos a la regresión
//				 break; //Salimos del for interno, no interesa un obstáculo por detrás del actual
//			 }			 
//		}					
//	}
// p=Regresion(y,x,npoints);//Ponemos al revés x e y para que nos de la recta con pendiente igual al ángulo que la silla forma con la pared
//return p;
//}
/******************************************************************************************************/
/**************** FIN ********** Control de velocidad ****************************** FIN **************/
/******************************************************************************************************/

/******************************************************************************************/
//	Sincronizacion_PC(): //Limitación de la velocidad por el acelerómetro
/******************************************************************************************/
//void Sincronizacion_PC(void){
//	
//}
//void Led_On(void)
//{
//	IOCLR0=(1<<10);
//}
//void Led_Off(void)
//{
//	IOSET0=(1<<10);
//}
//void Led_Toogle(void)
//{
//	if ( (IOPIN0&(1<<10))==(1<<10) )
//		IOCLR0=(1<<10);
//	else
//		IOSET0=(1<<10);
//}
