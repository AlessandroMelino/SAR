#define ID_MSG_PC			       0x0120
#define	ID_MSG_JOYA			     0x0110	 //Id. del msg CAN: Nodo Joystick, contiene Datos de motores (A)
#define	ID_MSG_JOYB			     0x0111	 //Id. del msg CAN: Nodo Joystick, contiene Modo de funcionamiento (A)
#define	ID_MSG_SENSORESA	   0x0201	 //Id. del msg CAN: Nodo Sensores, contiene Datos sensores SDI,STD (A) y SLDD, SLIT (B)
#define	ID_MSG_SENSORESB	   0x0202	 //Id. del msg CAN: Nodo Sensores, contiene Datos sensores SLID,SDD (A) y SLDT, STI (B)
#define	ID_MSG_ACELEROMETRO	 0x0203	 //Id. del msg CAN: Nodo Sensores, contiene Datos acelerómetro

#define PI 3.14159
#define A ((180*2*PI)/60)										// Constante de velocidad del motor (rad/s/voltios)
#define Vs 24																// Va_max, excitación del driver
#define Wu_max (Vs*A)												// Valor maximo de velocidad (rad/s) cuando Va = Vs = 15v, 256=15*A	
#define N 7																	// Pulsos por revolución del encoder
#define EDGES 4															// Flancos a considerar en la señal del encoder
#define REDUCTORA 50.9											// Valor del radio de reduccion
#define Ts 10										  					// Periodo de muestreo del encoder (en ms); Ts<TAO/10, 1ms (pej, para que sea redondo)
#define KREAL ((500*PI)/(Ts*N))        			// Kreal = 1/(Kenc·Kfc) = 2·PI/(4·Ts·N) = PI

#define HIGH 1
#define LOW 0
#define TRUE 1
#define FALSE 0

/* PARAMS DEL CONTROLADOR */
#define G 0.007*A
#define c 0.5789
#define Ka 10

extern short modo;
extern int auxD,auxI,datoI,datoD;
extern int mensajeA,mensajeB,dir;
extern int encoderIabs,encoderDabs;  //Valores absolutos de los encoder
extern int EncD, EncI,turno;
extern int cuentaI,cuentaD;

enum tipo_de_estado_motores {INICIALIZANDO,PARADO,CONFIG_LC,CONFIG_LA,LC,LA};
extern enum tipo_de_estado_motores estado_motores;
extern enum tipo_de_estado_motores lazoPC;

/****************************************************************************/
extern short prev_A_izq, prev_B_izq;
extern short flanco_subida_A_izq, flanco_bajada_B_izq, flanco_bajada_A_izq, flanco_subida_B_izq;
extern short prev_A_der, prev_B_der;
extern short flanco_subida_A_der, flanco_bajada_B_der, flanco_bajada_A_der, flanco_subida_B_der;
extern int Pulsos_Der;
extern int Pulsos_Izq;
extern float Velocidad_Izq;										 
extern float error_Izq , error_nm1_Izq , u_Izq , u_nm1_Izq;
extern float Error_Sat_Izq , Consigna_Sat_Izq;
extern float Velocidad_Der ;	
extern float error_Der , error_nm1_Der , u_Der , u_nm1_Der;
extern float Error_Sat_Der , Consigna_Sat_Der;

void Cuenta_Pulsos(void);
void Set_Velocidad_Izq(float Velocidad);	
void Set_Velocidad_Der(float Velocidad);
void Controla_Motores(int datosIzq, int datosDer);
void lee_can(void);
void envio_can(int datoA, int datoB,int etiqueta);
void envio_canD(void);
void envio_canI(void);
void lee_encoderD(void);
void lee_encoderI(void);

void Quitafreno (void);
void Frena (void);


//void envio_datos(void);
//void envio_datospc(void);
