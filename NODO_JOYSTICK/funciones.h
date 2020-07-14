
//Todos los define, excepto los relativos a los timer, se declaran aquí
#define MODO_MENU        0
#define MODO_JOYSTICK    1
//#define MODO_SOPLIDO     2
#define MODO_PC          3
//#define MODO_DIGJOY      4
//#define MODO_DEPURACION5 5
//#define MODO_DEPURACION7 7
//#define MODO_DEPURACION8 8
//#define MODO_DEPURACION9 9

#define RADIO	90
#define DISEJES	530
#define PI 3.141592654
#define RT 32   //Relación de la reductora de los motores
#define PPR 500 //Pulsos de encoder por revolución
#define V_MAX 2000 //VMAX = 2000 mm/s , aproximadamente 7,2km/h
#define R_MAX 500*PI //Velocidad de giro máx de 90º/s (500pi mrad/s)

#define INCREMENTO_SOPLIDO 25  //Incrementos de velocidad absoluta en cada iteración de soplido
#define DECREMENTO_SOPLIDO 75  //Decrementos de velocidad absoluta en cada iteración de soplido
#define GIRO_SOPLIDO 200*PI

#define ALPHA 0.8   //#define ALPHA 1000     //Pondera la influencia de las fuerzas repulsivas sobre la de comando
#define BETA  2     //Pondera las distancias sensor-obstaculo  
#define OMEGA 0.2   //Pondera la velocidad lineal de la silla  
extern double alpha;
extern double beta;  //Variables para sintonización del control
extern double omega;
//***********Posición de los sensores***********//
#define DISDT 410
#define DISD 830    //¡En milimetros!
#define DISJ 630
		//SENSOR DELANTERO IZQUIERDO(Al lado del lateral):
    #define xSDI (DISEJES/2) //DISEJES ESTA EN MM
    #define ySDI (DISDT+100)
    //SENSOR DELANTERO DERECHO(En el pie, casi al centro)
    #define xSDD 0
    #define ySDD DISD
    //SENSOR DEL JOYSTICK:
    #define xSJO (DISEJES/2)//DISEJES ESTA EN MM
    #define ySJO DISJ
    //SENSOR TRASERO IZQUIERDO:
    #define xSTI -(DISEJES/2)//DISEJES ESTA EN MM
    #define ySTI 0
    //SENSOR TRASERO DERECHO:
    #define xSTD (DISEJES/2)//DISEJES ESTA EN MM
    #define ySTD 0
    //SENSOR LATERAL IZQUIERDO TRASERO:
    #define xSLIT -(DISEJES/2)//DISEJES ESTA EN MM
    #define ySLIT 0
    //SENSOR LATERAL DERECHO TRASERO:
    #define xSLDT (DISEJES/2) //DISEJES ESTA EN MM
    #define ySLDT 0
    //SENSOR LATERAL IZQUIERDO DELANTERO:
    #define xSLID -(DISEJES/2)//DISEJES ESTA EN MM
    #define ySLID DISDT
    //SENSOR LATERAL DERECHO DELANTERO:
    #define xSLDD (double)(DISEJES/2)//DISEJES ESTA EN MM
    #define ySLDD DISDT				
//RELATIVOS AL GRID MAP
//		#define M2CELL        5 //1 metro -> M2CELL celdas  (establece resolución de grid map, a mayor M2CELL, mayor coste de cómputo)
//		#define ROWS          6*M2CELL //Dimensiones en filas (y) del grid map.
//		#define COLUMNS       4*M2CELL //Dimensiones en columnas (x) del grid map.
//		#define CELLCENTERX   2*M2CELL  //2 metros a la derecha, 2 a la izquierda, centro del eje de la silla en x
//		#define CELLCENTERY	  2*M2CELL  //4 metros por delante, 2 metros por detrás, centro del eje de la silla en y
//		#define CONVERGESPEED 5 //Porcentaje de probabilidad perdido en cada muestreo
//		extern int gridmap[ROWS][COLUMNS];
//		extern char EncoderUpdate; //1 para un encoder, 2 para los 2, cuando llega a 2 se emparejan sus valores y se actualiza el grid map.
//		extern double wallangle;
extern signed char velI,velD;
extern int valorpot, valorx, valory,valorsop;          //Valores tomados de los ADC
extern int inputV,correctV,correctV1,realV,inputR,correctR,wd,wi,realD,realI;
extern int modo;
extern int nivel_bateria;
extern int ejey,ejex;//Ejes joystick 		


extern char Parada;
extern int nivel_bateria;
extern char Flag_sincronia_PC;
extern char Cambiomodo;

enum tipo_de_estado_PC {SINCRONIZANDO, NO_SINCRONISMO, SINCRONISMO};    // Estados de
extern enum tipo_de_estado_PC estado_PC;                               //     PC

enum tipo_de_digestado {PARADO, ACELFOR, ACELBACK, VCTEFOR,VCTEBACK,DECELFOR,DECELBACK}; //Estados JDigital
static enum tipo_de_digestado digestado=PARADO;                                         //
extern char FlagBloqueo;                                                               //

enum tipo_veloc {STOP, ANDANDO, PARANDO};           // 
enum tipo_de_estado {CERO, PREGUNTA, UNICO_E,};    // Estados
static enum tipo_veloc situacion;                 //  soplido
static enum tipo_de_estado estado;               //
enum tipo_adquirido {NULO,MA,MB,A,B};      // Variable interpretada 
static enum tipo_adquirido entrada,Dato;  //     del soplido

void config(void);
extern int abs(int /*number*/);
extern int Saturacion(int/*number*/,int/*limite inferior*/,int/*limite superior*/);
extern char Signo(int/*number*/);
extern void Led_On(void);
extern void Led_Off(void);
extern void Led_Toogle(void);

extern char Procesa_Teclado(void);
extern void Borrar_Vdisplay(void),Actualiza_Vdisplay(void);

extern void Adquiere_Analogicos(void),Conversion_Joystick(void),Conversion_JDigital(void),
	Conversion_Soplido(void),Soplido(void),Aceleracion(void);
extern void init_can(void),Can_Envio(void),Can_Lectura(void),Can_Modo_aPC(void);

extern void Sensores(void);
extern void MoveGrid(void);
extern double RegresionGrid(void);
extern void Acelerometro(void);
