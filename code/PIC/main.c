// Microcontrolador utilizado
#include   <18f2550.h>

// Configurações para gravação
//#fuses HSPLL, CPUDIV1, PLL4, NOWDT, PUT, NOLVP, USBDIV

// Definições para uso de Rotinas de Delay
//#use delay(clock=48000000)
#fuses XT, NOWDT, NOPROTECT, PUT, NOLVP 
#use delay(crystal=4000000)
#use i2c(slave, sda=PIN_B0, scl=PIN_B1, address=0x58, SLOW)
//#use rs232(baud=9600,xmit=PIN_C6,rcv=PIN_C7)

#priority timer1,timer3

// Modo rápido de inicialização das portas
#use fast_io(a)
#use fast_io(b)
#use fast_io(c)
#use fast_io(e)

// Endereço de memória acossiado com cada porta
#byte porta = 0xf80
#byte portb = 0xf81
#byte portc = 0xf82

// Portas ligadas a ponte H
#bit ain1 = portb.6
#bit ain2 = portb.5
#bit stby = portb.7

//motor constants
const float reduction = 50;
//encoder constants
const float rising = 6;
//Convert period to speed in radians, period is in us
const double to_rad_s = (((2*3.141592)/reduction)/rising)*1000000;
//control

//open loop, loop == 0
//closed loop, loop == 1
int8 loop = 1;

double speed_rad = 0;

// HZ
const double frequency = 60;

// PID PARAMETERS
int16 KP = 5, KI = 200, KD = 0, set_point = 10;

// SECONDS
const double dt = (1 / frequency);

// PID
double integral = 0;
double last_error = 0;


// Função de controle de velocidade
int PID(double setpoint, double input) {
  
  double error = setpoint - input;
  double derivative = (error - last_error) / dt;
  
  integral = integral + (error * dt);  
  last_error = error;

  double output = KP * (error) + KI * (integral) + KD * (derivative);
    
  if (output > 255) output = 255;
  if (output < 0) output = 0;
  
  return output;
}


//ccp
unsigned int8 pwm = 0;
int16 period = 0;

//i2c
unsigned int8 incoming, state;
unsigned int8 index = 0;
const unsigned int8 length = 4;
unsigned int8 vector[length], frame[length];

//FRAME[0] ENDEREÇO I2C
//FRAME[1] ID
//FRAME[2 E 3] DADOS

int16 two_int8_to_int16(int8 byte0, int8 byte1){
   int16 value = byte0;
   value = value << 8;
   value |= byte1;
   return value;
}

void create_frame(unsigned int8 data){
   vector[index] = data;
   index++;
   if(index >= length){
      index = 0;
      for(int8 i = 0; i < length; i++){
         frame[i] = vector[i];
         vector[i] = 0;
      }
      
      //salva os params
      if(frame[1] == 0xE0){
         set_point = two_int8_to_int16(frame[2], frame[3]);
      }
      
      if(frame[1] == 0xE2){
         pwm = frame[3];
         //two_int8_to_int16(frame[2], frame[3]);
      }
      
      if(frame[1] == 0xE3){
         KP = two_int8_to_int16(frame[2], frame[3]);
      }
      
      if(frame[1] == 0xE4){
         KI = two_int8_to_int16(frame[2], frame[3]);
      }
      
      if(frame[1] == 0xE5){
         KD = two_int8_to_int16(frame[2], frame[3]);
      }
      
      if(frame[1] == 0xAA){
         loop = 0;
      }
      
      if(frame[1] == 0xFF){
         loop = 1;
      }
   }
}


#INT_SSP 
void ssp_interrupt() 
{
state = i2c_isr_state();
if(state < 0x80)     // Master is sending data 
  {
    incoming = i2c_read();
    create_frame(incoming);
  }

if(state >= 0x80)   // Master is requesting data from slave 
  {
   
   //read Set Point
   if(frame[1] == 0xF0){
      i2c_write(((set_point >> 8 ) & 0xFF));
      i2c_write((set_point & 0xFF));
   }
   
   //read period
   if(frame[1] == 0xF1){
      i2c_write(((period >> 8 ) & 0xFF));
      i2c_write((period & 0xFF));
   }
   
   //read pwm
   if(frame[1] == 0xF2){
      i2c_write(pwm);
      i2c_write(0);
   }
   
   //read KP
   if(frame[1] == 0xF3){
      i2c_write(((KP >> 8 ) & 0xFF));
      i2c_write((KP & 0xFF));
   }
   
   //read Ki
   if(frame[1] == 0xF4){
      i2c_write(((KI >> 8 ) & 0xFF));
      i2c_write((KI & 0xFF));
   }
   
   //read Kd
   if(frame[1] == 0xF5){
      i2c_write(((KD >> 8 ) & 0xFF));
      i2c_write((KD & 0xFF));
   }
   
   if(frame[1] == 0xFA){
      i2c_write(loop);
      i2c_write(0);
   }
   
  }
}

//Leitura período da onda
#int_ccp1
void interrupt_ccp1() {
   set_timer1(0);
   period = CCP_1;
}


// Tratamento da interrupção
#int_timer3
void timer3(){
   //60Hz
   set_timer3(33333);
   if(loop == 1){
      speed_rad = (to_rad_s/period);
      pwm = PID((double) set_point, speed_rad);
   }
   set_pwm2_duty(pwm);
}

// Função para configurar o pwm
void config_pwm() {
   setup_ccp2(CCP_PWM);
   setup_timer_2(T2_DIV_BY_1, 255, 1);
}

// Função para configurar o modo capture
void config_capture(){
   setup_ccp1(CCP_CAPTURE_RE);
   setup_timer_1(T1_INTERNAL | T1_DIV_BY_1); //1us
   enable_interrupts(INT_CCP1);
}

// Função para configurar e ativar a ponte h para a movimentação do robô
void config_motor() {
   ain1 = 1;
   ain2 = 0;
   stby = 1;   
}

// Função para configurar o timer 3 responsável por chamar o controlador PID
void config_timer3() {
   setup_timer_3 (T3_INTERNAL | T3_DIV_BY_1);
   enable_interrupts(int_timer3);
}

// Programa principal
void main() {
   setup_oscillator( OSC_8MHZ );

   // configura os tris
   set_tris_a(0b11111111);
   set_tris_b(0b00011011);
   set_tris_c(0b11111101);

   // inicializa os ports
   porta=0x00;
   portb=0x00;
   portc=0x00;
   
   config_capture();
   config_pwm();
   config_motor();
   config_timer3();
   
   enable_interrupts(INT_SSP);//I2C interrupt
   enable_interrupts(GLOBAL);
    
   while (true) {

   }
}
