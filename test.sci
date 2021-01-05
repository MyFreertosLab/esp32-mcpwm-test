A=[1,1,1,1,1,1;-1,1,0.5,-0.5,-0.5,0.5;0,0,sqrt(3)/2,-sqrt(3)/2,sqrt(3)/2,-sqrt(3)/2;1,-1,1,-1,-1,1];
B=[1/6,-1/3,0,1/6;1/6,1/3,0,-1/6;1/6,1/6,1/(2*sqrt(3)),1/6;1/6,-1/6,-1/(2*sqrt(3)),-1/6;1/6,-1/6,1/(2*sqrt(3)),-1/6;1/6,1/6,-1/(2*sqrt(3)),1/6];

A1=[1,1,1,1,1,1;-1,1,0.5,-0.5,-0.5,0.5;0,0,sqrt(3)/2,-sqrt(3)/2,sqrt(3)/2,-sqrt(3)/2;1,-1,1,-1,-1,1;0,0,3,3,0,0;0,0,0,0,3,3];
B1=inv(A1);

//
// Accel (Z) è espressa in m/sec^2 e rispetto al sistema inerziale (terra)
// Gyro (R,P,Y) è espressa in rad/sec e rispetto al Body Frame del Drone
//

// ************
// **** M1 ****
// ************
// Description..:  Front End
//
// Parameters..:   dt (reattività espressa in secondi come intervallo di tempo [0,dt])
//
// Input.......:   RC (Accel (Z), inclinazioni 3D in rad)
//                 IMU_ATTITUDE (Attitude rilevati espressi in rad)
//                 IMU_SPEED (Accel (Z) espressa in m/sec^2 e Velocità angolari rilevate espresse in rad)
//
// Output......:   M1_OUT = Variazioni Richieste (Variazione Accel (Z) in m/sec^2, accelerazioni angolari 3D in rad/sec^2)
//
RC=[30;%pi/6;0;0];
IMU_ATTITUDE=[0;1;0;0]; // IMU_ATTITUDE(1) deve essere sempre 0, serve solo a mantenere la dimesione del vettore
IMU_SPEED=[25;0;0;0];   
dt=1;
M1_TRPY=(RC-IMU_ATTITUDE)/dt; // velocità angolari richieste (rimetto a posto il Accel (Z) subito sotto)
M1_TRPY(1)=RC(1); // Accel (Z) RC(1) è già espressa come accelerazione in m/sec^2
M1_OUT=(M1_TRPY - IMU_SPEED)/dt; // accelerazioni angolari richieste
M1_OUT(1)=M1_TRPY(1) - IMU_SPEED(1); // RC(1) è già espressa come accelerazione in m/sec^2

// ************
// **** M2 ****
// ************
//   Trasformazione: Variazione Moto Richiesto -> Variazioni Forze Motori Richieste
//
M2_IN=M1_OUT;
M2_OUT=B*M2_IN;

// ************
// **** M3 ****
// ************
//   Controller PID: Forze Motori Richieste -> Forze Motori corrette
//
M3_IN=M2_OUT;
// * La stima delle forze motori deve essere attivata se e solo se esiste un sensore adhot (eg. hall sensor) *
// * Per cui la stima eseguita sotto è commentata (è già inclusa in M1 *
// IMU_EST_TRPY=[25;1;0;0]; // Accel (Z) e Velocità angolari in rad/sec
// IMU_EST_TRPY_PREV=[0;0;0;0]; 
// IMU_EST_ACCEL=(IMU_EST_TRPY - IMU_EST_TRPY_PREV)/dt;
// IMU_EST_ACCEL(1)=IMU_EST_TRPY(1); // Accel (Z) è già una accelerazione
// THRUST_STATUS_MOTORS=[5;5;5;5;5;5]; // T.B.D: Questo è messo per prova. Si assume di avere un Hall Sensor
// M3_P_ERR=(M3_IN+THRUST_STATUS_MOTORS)-(B*IMU_EST_ACCEL);
M3_P_ERR=M3_IN;
M3_I_ERR=[0;0;0;0;0;0];
M3_D_ERR=[0;0;0;0;0;0];
kp = 2;
ki = 1;
kd = 1;
M3_OUT = kp*M3_P_ERR+ki*M3_I_ERR+kd*M3_D_ERR;

// ************
// **** M4 ****
// ************
//   Trasformazione: Forze Motori -> PWM
//
M4_IN=M3_OUT;
kdt=3;
M4_STATUS_DUTY_CYCLE=[0;0;0;0;0;0];
M4_DUTY_CYCLE=kdt*sqrt(M4_IN);
M4_OUT=M4_STATUS_DUTY_CYCLE+M4_DUTY_CYCLE

///* Definire i range di scala per ogni categoria di valori
// RC: (Accel (Z), R,P,Y, CHAN1, CHAN2)
// IMU_ATTITUDE: (T, wR,wP,wY)
// IMU_SPEED: Accel (Z) e Gyro (R,P,Y)
// Forze Motori (1..6)
// PID
//

