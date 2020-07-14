#define T1_MSEGUNDOS Ts				//Se realiza control cada Ts ms
#define TFRENO_MSEGUNDOS 750

#define T_PWM 0.025													// Periodo de la PWM (en ms)
#define PCLK 60000000												// Frecuencia (en Hz) del PCLK
#define MR0_VALUE (T_PWM*PCLK*0.001)		    // Setup priodo de la PWM, se pasa a segundos

extern unsigned int volatile gTimerTick;
extern unsigned int volatile encoderITimerTick,encoderDTimerTick;
extern unsigned int  CanRX;
extern unsigned int  Timer1, T1, Active_Timer1, T1ticks;
extern unsigned int  Timer_freno, Tfreno, Active_Timerfreno, Tfrenoticks;

void config(void);
//void activoINTS(void);
//void desactivoINTS(void);
short IsTimeExpired (unsigned int);
void SlowDown (unsigned int);
void Set_T1(void);
