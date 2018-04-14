//Librerías
#include <avr/io.h>         
#include <avr/interrupt.h>

//Definimos ADCs
#define ADC0 0x00
#define ADC1 0x01

//Definimos pin de salida 2
#define PIN9 9

//Variables para timers
int numSamples=0;
int flag_time=0;
int cambio_freq=0;

//Variables del ADC0
float val_ADC0=0;
float val_filtro_ADC0=0;
float val_filtro_ADC0_ant=0;
float distance_ADC0=0;

//Variables ADC1
float val_ADC1=0;


//Variables para timers
long t=0, t0=0;
long t_encoder=0, t0_encoder=0,filtro_t_encoder=0,filtro_t_encoder_ant=0;
float t_RPS=0;
float t_RPM=0;
float Vel=0;
float VelKm=0;
float ReactionDistance=0;
float StopDistance=0;
float TotStopDistance=0;


//Definimos nuestros valores para nuestra constante proporcional, integral y diferencial
#define KP  40
#define KI  0
#define KD  5  
#define T 1


//Definimos nuestras U y variables de error
float integral=0;
float derivative=0; 
float proportional=0;
float errorT1=0;
float last_error=0;
int PWM=0;
int ref=0;

void setup() {
  //Configuracion de entradas y salidas
    DDRD = 0x00;          // Puerto D como entrada

    //Configuración de nuestra interrupcion por ADC
    ADCSRA = 0;             // Limpiamos el registro
    ADCSRB = 0;             // Limpiamos el registro
    ADMUX |= ADC0;    // Configuramos con ADC0
    ADMUX |= (1 << REFS0);  // COnfiguramos voltaje de referencia
    ADMUX |= (1 << ADLAR);  // left align ADC value to 8 bits from ADCH register
  
    ADCSRA |= (1 << ADPS2) | (1 << ADPS0);          // define preescaler
    ADCSRA |= (1 << ADATE); // enable auto trigger
    ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
    ADCSRA |= (1 << ADEN);  // enable ADC
    ADCSRA |= (1 << ADSC);  // start ADC measurements

    sei();        // Enable global interrupts by setting global interrupt enable bit in SREG

    Serial.begin(115200);     //Iniciamos con nuestro serial en 115200 baudiis
    analogWrite(PIN9,PWM);          //Mandamos a otro pin analogico el valor del PWM
}

//Vector de interrupcion de ADC
ISR(ADC_vect)
{
 
  byte x = ADCH;  // read 8 bit value from ADC

   
  switch (ADMUX&0x0F) {
   case ADC0:
        
        val_ADC0=float(x);
        val_filtro_ADC0= float(0.72)*val_filtro_ADC0_ant+float(0.28)*val_ADC0;
        val_filtro_ADC0_ant=val_filtro_ADC0;
        numSamples++;
    
        ADCSRA&=0x7F;     //Ignoramos lo que no nos importa
        ADMUX &= 0xF0;    //Ignoramos la parte baja
        ADMUX |= ADC1;    // Hacemos toogle de ADC
        ADCSRA |= (1 << ADEN);  // Activamos ADC
        ADCSRA |= (1 << ADSC);  // start ADC measurements
        
      break;
    case ADC1:
    
        val_ADC1=float(x);

        ADCSRA&=0x7F;       //Ignoramos lo que no nos importa
        ADMUX &= 0xF0;      //Ignoramos lo que no nos importa
        ADMUX |= ADC0;    // Hacemos toogle de ADC
        ADCSRA |= (1 << ADEN);  // Activamos ADC
        ADCSRA |= (1 << ADSC);  // start ADC measurements
        

        if(val_ADC1<100){
          if(flag_time==0){
            t_encoder = micros()-t0_encoder;  // calculate elapsed time
            filtro_t_encoder= float(0.95)*filtro_t_encoder_ant+float(0.05)*t_encoder;
            filtro_t_encoder_ant=filtro_t_encoder;
            
            flag_time=1;
            cambio_freq=1;
            t0_encoder = micros();
          }
        }
        else{
          flag_time=0;
        }        
      break; 
    default:
       break;
  }   
}

void loop() {
 
   t = micros()-t0;  // calculate elapsed time
  /*if (numSamples>=1000)
  {
    t = micros()-t0;  // calculate elapsed time
    Serial.print("Sampling frequency: ");
    Serial.print((float)1000000/t);
    Serial.println(" KHz");
    
    t0 = micros();
    numSamples=0;
  }*/

    distance_ADC0 =((1698.0 / (val_filtro_ADC0 - 3)) - 4);
    t_RPS=((float)1000000/filtro_t_encoder)/20;
    t_RPM=t_RPS*60;
  
    

    if(t>100000&&cambio_freq==0){
      t_RPS=0;
      t_RPM=0;
    }

    if(cambio_freq==1||t>100000){

    Vel=(0.065*PI*t_RPS);
    VelKm=(0.065*PI*t_RPS)*3.6;
    ReactionDistance=Vel*0.5;
    StopDistance=(Vel*Vel)/(2*9.81*0.6);
    TotStopDistance=(ReactionDistance+StopDistance)*100;

    if(distance_ADC0<=11+TotStopDistance){
      ref=0;
    }
    else {
      ref=200;
    }

     PID();  
     analogWrite(PIN9,PWM);          //Mandamos a otro pin analogico el valor del PWM
     cambio_freq=0;
     t0 = micros();
    }

    
}


void PID(){
  
  float error;

  Serial.print("Distancia: ");
  Serial.println(distance_ADC0);

  Serial.print("RPS: ");
  Serial.print(t_RPS);
  Serial.println(" Hz"); 

  Serial.print("RPM: ");
  Serial.print(t_RPM);
  Serial.println(" Hz"); 
  
  Serial.print("VEL: ");
  Serial.print(VelKm);
  Serial.println(" km/h"); 

  Serial.print("Reaction: ");
  Serial.print(ReactionDistance);
  Serial.println(" m"); 

  Serial.print("Stop: ");
  Serial.print(StopDistance);
  Serial.println(" m"); 

  
  Serial.print("TotStop: ");
  Serial.print(TotStopDistance);
  Serial.println(" m"); 

  Serial.print("Lectura: ");
  Serial.println((t_RPM/100)*255);
  
  error = ref - (t_RPM/100)*255;
  if(error!=0){
      proportional = error * KP;
      integral = errorT1 +error*KI*T;
      derivative = (error - last_error)/T * KD;
      last_error = error;
      errorT1 = integral;
   }

  Serial.print("KP: ");
  Serial.println(proportional);

  Serial.print("KI: ");
  Serial.println(integral);

  Serial.print("KD: ");
  Serial.println(derivative);
  
  
  Serial.print("Error: ");
  Serial.println(error);

  PWM = proportional +integral + derivative;
  Serial.print("PWM BEFORE: ");
  Serial.println(PWM);

  if(ref>100){
      
      //Si el PWM es menor a cierto valor pero no e cero entonces lo dejamos ahi porque sino se apaga
      if(PWM<100){
           PWM=100;
      }
     
      //Si el PWM es mayor a cierto valor entonces lo truncamos a un valor máximo
      if(PWM>255){
           PWM=255;
      }

  }

  else{
          //Si el PWM es menor a cierto valor pero no e cero entonces lo dejamos ahi porque sino se apaga
      if(PWM<0){
           PWM=0;
       }
     
      //Si el PWM es mayor a cierto valor entonces lo truncamos a un valor máximo
      if(PWM>80){
           PWM=80;
      }
  }
  
  Serial.print("PWM AFTER: ");
  Serial.println(PWM);
}


