// Codigo para el controlador de temperatura PI del criostato
// Paula Sanchez Almagro
// TFG 2021 
// ETSIT ISOM UPM 


//*****************************************   SECCION 1   *****************************************
// Librerias empleadas
#include <LiquidCrystal.h> 
#include <SPI.h> 

// Constantes para control PI
#define Kp 3.00       // Definicion de constante para el control proporcional   
#define Ki 0.01       // Definicion de constante para el control integral      


// Constantes y variable de control para el manejo de los botones del display
#define CICLOS_PULSACION1 1   // Umbral para evitar rebotes y aumentar/disminuir set-point de 1 en 1
#define CICLOS_PULSACION2 20  // Umbral para aumentar/disminuir set-point de 5 en 5
#define CICLOS_PULSACION3 30  // Umbral para aumentar/disminuir set-point de 10 en 10
#define CICLOS_PULSACION4 40  // Umbral para aumentar/disminuir set-point de 25 en 25
#define CICLOS_PULSACION5 50  // Umbral limite en el aumento/disminucion del set-point de 25 en 25
int n_ciclos =0;              // Contador de ciclos de pulsacion de los botones

// Pines del convertidor A/D (entrada de datos al ARDUINO)
const byte DAT = 12;      // SPI MISO Pin
const byte CLK = 13;      // SPI Clock Pin
const byte CS = 10;       // SPI SS Pin (Chip Select)

// Pines display LCD
LiquidCrystal lcd = LiquidCrystal(3, 2, 4, 5, 6, 7); 

// Tabla tension-temperatura del diodo del criostato para corriente de polarización de 100 uA
// float tabla_diodo_tension[]={1.7589, 1.7539, 1.6876, 1.5852, 1.4576, 1.3125, 1.1885, 1.1120, 1.0875, 1.0743, 1.0635, 1.054, 1.0446, 1.0356, 1.0264, 1.0169, 1.0073, 0.9976, 0.9877, 0.9777, 0.9676, 0.9574, 0.9472, 0.9368, 0.9264, 0.9159, 0.9053, 0.8947, 0.8840, 0.8733, 0.8624, 0.8516, 0.8406, 0.8296, 0.8186, 0.8074, 0.7963, 0.7851, 0.7738, 0.7625, 0.7512, 0.7398, 0.7285, 0.7171, 0.7057, 0.6944, 0.6830, 0.6717, 0.6604, 0.6492, 0.6380, 0.6268, 0.6158, 0.6047, 0.5938, 0.5829, 0.5721, 0.5613, 0.5506, 0.5399};
// float tabla_diodo_temperatura[]={4, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 160, 165, 170, 175, 180, 185, 190, 195, 200, 205, 210, 215, 220, 225, 230, 235, 240, 245, 250, 255, 260, 265, 270, 275, 280, 285, 290, 295};

// Tabla tension-temperatura del diodo del criostato para corriente de polarización de 10 uA
//float tabla_diodo_tension[]={1.800000,1.612400,1.395600,1.271200,1.175900,1.111400,1.101400,1.093300,1.085500,1.077600,1.069600,1.061300,1.053000,1.044600,1.036100,1.027600,1.019000,1.010200,1.001400,0.992400,0.983200,0.974000,0.964600,0.955200,0.945700,0.935900,0.926100,0.916200,0.906200,0.896200,0.886000,0.875800,0.865500,0.855100,0.844600,0.834100,0.823500,0.812900,0.802100,0.791400,0.780600,0.769800,0.758900,0.747900,0.736900,0.725900,0.714800,0.703700,0.692600,0.681400,0.670200,0.659000,0.647700,0.636500,0.625100,0.613800,0.602400,0.591000,0.579600,0.568200,0.556700,0.545200,0.533700,0.522200,0.510700,0.499200,0.487600,0.476000,0.464400,0.452800,0.441100,0.429500,0.417800,0.406100,0.394400,0.382600,0.370900,0.359100,0.347300,0.335500,0.323700,0.311800,0.299900,0.288000,0.276100,0.264200,0.252300,0.240300};
//float tabla_diodo_temperatura[]={0,5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100,105,110,115,120,125,130,135,140,145,150,155,160,165,170,175,180,185,190,195,200,205,210,215,220,225,230,235,240,245,250,255,260,265,270,275,280,285,290,295,300,305,310,315,320,325,330,335,340,345,350,355,360,365,370,375,380,385,390,395,400,405,410,415,420,425,430,435};

// Tabla tensión-temperatura del diodo nuevo para corriente de 100 uA recalibrada
float tabla_diodo_tension[]={1.800000,1.612400,1.395600,1.271200,1.175900,1.111400,1.101400,1.093300,1.085500,1.077600,1.069600,1.061300,1.053000,1.044600,1.036100,1.027600,1.019000,1.010200,1.001400,0.992400,0.983200,0.974000,0.964600,0.955200,0.945700,0.935900,0.926100,0.916200,0.906200,0.896200,0.886000,0.875800,0.865500,0.855100,0.844600,0.834100,0.823500,0.812900,0.802100,0.791400,0.780600,0.769800,0.758900,0.747900,0.736900,0.725900,0.714800,0.703700,0.692600,0.681400,0.670200,0.659000,0.647700,0.636500,0.625100,0.613800,0.602400,0.591000,0.579600,0.568200,0.556700,0.545200,0.533700,0.522200,0.510700,0.499200,0.487600,0.476000,0.464400,0.452800,0.441100,0.429500,0.417800,0.406100,0.394400,0.382600,0.370900,0.359100,0.347300,0.335500,0.323700,0.311800,0.299900,0.288000,0.276100,0.264200,0.252300,0.240300};
float tabla_diodo_temperatura[]={5.04,10.64,16.24,21.84,27.44,33.04,38.64,44.24,49.84,55.44,61.04,66.64,72.24,77.84,83.44,89.04,94.64,100.24,105.84,111.44,117.04,122.64,128.24,133.84,139.44,145.04,150.64,156.24,161.84,167.44,173.04,178.64,184.24,189.84,195.44,201.04,206.64,212.24,217.84,223.44,229.04,234.64,240.24,245.84,251.44,257.04,262.64,268.24,273.84,279.44,285.04,290.64,296.24,301.84,307.44,313.04,318.64,324.24,329.84,335.44,341.04,346.64,352.24,357.84,363.44,369.04,374.64,380.24,385.84,391.44,397.04,402.64,408.24,413.84,419.44,425.04,430.64,436.24,441.84,447.44,453.04,458.64,464.24,469.84,475.44,481.04,486.64,492.24};

//Variables a utilizar en la funcion convertir
int i;                  // indice para recorrer el array de tensiones 
int n;                  // Variable para guardar el indice del array que coincide con el intervalo buscado
float m;                // Pendiente de la recta de ajuste
float v0;               // Valor de tension que limita inferiormente el intervalo de tensiones en la que se encuentra
float t0;               // Valor de temperatura que limita inferiormente el intervalo de temperaturas en la que se encuentra

//Variables a utilizar en la funcion leer_tension
float leer_tension_del_ADC_mediante_SPI=0; // Tension en bornas real tras ajustar el valor de salida del conversor
int valorADC;           // Valor de salida del conversor
int msb;                // Bit mas significativo
int lsb;                // Bit menos significativo

int PWMpin=9;           // Definicion del pin de salida PWM

float valor_acum=0.0;   // Valor acumulado de la tension del diodo
int n_muest=0;          // Numero total de muestras leidas
float temperatura=0;    // Temperatura actual del criostato
float integral=0;       // Valor acumulado del error
float setpoint=40;      // Valor del set-point
int potencia=0;         // Valor de la potencia del calefactor
float a;
float b;

// Declaracion de los botones de usuario 
int AN0=0;              // Boton para disminuir el setpoint
int AN1=1;              // Boton para aumentar el setpoint

//Activacion del control PI
int AN2=2;              // Valor analogico que indica el estado del pulsador que activa el control PI


//*****************************************   SECCION 2   *****************************************

// Funcion en la que se inicializan las librerias y se 
// configuran los pines de entrada/salida y el estado 
// incial de alguno de ellos
void setup() {
  Serial.begin(9600);

  lcd.begin(16, 2); //Inicializacion display lcd 16x2

  SPI.begin();      //Inicializacion libreria SPI

  //Configuracion de pines
  
  pinMode(DAT,INPUT); 
  pinMode(CS,OUTPUT); 

  // Se escribe el pin CS como LOW y depues en HIGH  
  // porque el estado del pin de PWR ON se desconoce
  digitalWrite(CS,LOW);   
  digitalWrite(CS,HIGH);  

  digitalWrite(CLK,LOW);

  pinMode (PWMpin, OUTPUT);

  // Inicializacion de los botones de usuario 
  pinMode (AN0, INPUT); 
  pinMode (AN1, INPUT);
}

//*****************************************   SECCION 3   *****************************************

// Funcion que hace una conversion entre la tension medida en el diodo 
// y la temperatura que le correspone segun la tabla anteriormente definida
float convertir(float tension_diodo){
  // Busqueda con las tablas dato los valores limites de intervalo de tension y temperatura en el que se encuentra
  for(i=1; i<87; i=i+1){
    if((tabla_diodo_tension[i]<tension_diodo) && (tension_diodo<= tabla_diodo_tension[i-1])){
     n=i; 
    }
  }

  // Linealizacion del intervalo de tension y temperatura en el que se encuentra tension_diodo
  m=(tabla_diodo_temperatura[n]-tabla_diodo_temperatura[n-1])/(tabla_diodo_tension[n]-tabla_diodo_tension[n-1]);
  v0=tabla_diodo_tension[n-1];
  t0=tabla_diodo_temperatura[n-1];

  return (m*(tension_diodo-v0))+t0;
}

// Funcion que lee el valor de salida del conversor A/D 
void leer_tension (){  
  leer_tension_del_ADC_mediante_SPI=0;
  
  //Comunicacion SPI con MSB en primer lugar
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); 
  
  //Entre conversiones el pin CS debe estar activo a nivel alto
  digitalWrite(CS,LOW);
  msb = SPI.transfer(0x00);         
  lsb = SPI.transfer(0x00);         
  digitalWrite(CS,HIGH);            
  SPI.endTransaction();
  valorADC=((((msb & 0x1F) << 8) | lsb)>>1);

  // Una vez se captura la infromacion del conversor, se ajusta multiplicando
  // el valor por los 5V de alimentacion del conversor y diviendo entre el
  // numero de niveles que se obtienen con los 12 bits del conversor (2^12=4096)
  // y la ganancia completa del bloque del amplificador diferencial (3.4)
  leer_tension_del_ADC_mediante_SPI= (float)valorADC*5/4096/3.4;   

  //Buffers de acumulacion de los valores leidos y el numero de muestras tomadas
  valor_acum=valor_acum+leer_tension_del_ADC_mediante_SPI;
  n_muest++;
}

// Metodo de control PI
void control_pi ()  {
   float error;
   float valor_PI;

   // Reseteo de parametros si el interruptor de control esta desactivado
   if(analogRead(AN2)<127){
    potencia =0;
    integral=0;
    analogWrite(PWMpin, potencia);    // Se deja de generar el pwm que calienta el criostato
    return;
   }
   
   error=setpoint-temperatura;   
   integral=integral+error;

   // integral solo acumula el error cuando la potencia no excede los limites superior o inferior
   if (potencia==0 && error<0){  
     integral=integral-error;
   }
   if (potencia==255 && error>0){ 
     integral=integral-error;
   }  

   // Formula para control PI  
   valor_PI=Kp*error + Ki*integral;   

   potencia=valor_PI;                 

   // No superar los limites
   if (potencia>255)                   
     potencia=255;
   if (potencia<0)
     potencia=0;
   
   // Actualizacion de la potencia de salida del controlador
   analogWrite(PWMpin, potencia); 
}

// Funcion para actualizar la temperatura que se esta midiendo
void actualizar_temp (){
  
   // Si han transcurrido 10 bucles (1s) actualiza la temperatura, y aplica la correccion de offset 
   if (n_muest==10){
     temperatura=((convertir (valor_acum/n_muest))* (253.4/261.5))-(4355/523);     
     n_muest=0;
     valor_acum=0.0;
     
     // Refresco del display con la temperatura
      lcd.setCursor(0, 0);
      lcd.print("Temp= ");
      lcd.print(temperatura, 1); 
      lcd.print("K  ");
      lcd.setCursor(0, 1);
      lcd.print("SP= ");
      lcd.print(setpoint, 0);
      lcd.print("K    ");
      lcd.setCursor(13, 0);
      lcd.print("Pot");

      //Cambiar que si pasa de 100 a 99 no se quede un numero de mas y actualiza el puntero de impresion de K aqui y en el SP

      // Refresco del valor en porcentaje de la potencia suminsitrada
      int potencia_porc= potencia*100/255;
      int cursor_lcd;
      if(potencia_porc<10){
        cursor_lcd=14;
        lcd.setCursor(12, 1);
        lcd.print("  ");
      }else if((10<=potencia_porc)&&(potencia_porc<100)){
        cursor_lcd=13;
        lcd.setCursor(12, 1);
        lcd.print(" ");
      }else{
        cursor_lcd=12;
      }
      lcd.setCursor(cursor_lcd, 1);
      lcd.print(potencia_porc);
      
      lcd.setCursor(15, 1);
      lcd.print("%");
     }
   }


void imprimir_SP (){
  if(setpoint<1){
        setpoint=1;
  }else if(setpoint>500){
        setpoint=500;
  }
  lcd.setCursor(0, 1);
  lcd.print("SP= ");
  lcd.print(setpoint, 0);
  lcd.print("K    ");
}

// Funcion de lectura de los botones para configurar el valor de set-point. Es 
// sensible a los ciclos que se mantienen pulsados los botones aumentando la 
// velocidad con la que aumenta/disminuye el valor de set-point
void leer_teclado(){
  if((analogRead(AN0)>127)||(analogRead(AN1)>127)){
    n_ciclos++;

    if(n_ciclos <= CICLOS_PULSACION1){
      return; // Evita rebotes
     } else if ((n_ciclos > CICLOS_PULSACION1)&&(n_ciclos <= CICLOS_PULSACION2)) {
      if(analogRead(AN0)>127){
        setpoint --;
        imprimir_SP();
        return;
      } 
    
      if(analogRead(AN1)>127){
        setpoint++;  
        imprimir_SP();
        return;
      } 
    }else if((n_ciclos > CICLOS_PULSACION2)&&(n_ciclos <= CICLOS_PULSACION3)){
      if(analogRead(AN0)>127){
        setpoint = setpoint-5;
        imprimir_SP();
        return;
      } 
    
      if(analogRead(AN1)>127){
        setpoint = setpoint +5;  
        imprimir_SP();
        return;
      }
    }else if((n_ciclos > CICLOS_PULSACION3)&&(n_ciclos <= CICLOS_PULSACION4)){
      if(analogRead(AN0)>127){
        setpoint = setpoint-10;
        imprimir_SP();
        return;
      } 
    
      if(analogRead(AN1)>127){
        setpoint = setpoint +10;  
        imprimir_SP(); 
        return;
      }
    }else if((n_ciclos > CICLOS_PULSACION4)&&(n_ciclos <= CICLOS_PULSACION5)){
      if(analogRead(AN0)>127){
        setpoint = setpoint-25;
        imprimir_SP();
        return;
      } 
    
      if(analogRead(AN1)>127){
        setpoint = setpoint +25;  
        imprimir_SP();
        return;
      }
    }
 }
  n_ciclos=0;  
}

//*****************************************   SECCION 4   *****************************************

void loop() {
  long sync = millis();

   leer_tension (); // Leer y acumular la tension del diodo
   leer_teclado (); // Manejo de botones
   actualizar_temp (); // Convertir tension a temperatura y actualizar el display
   control_pi ();   // Control PI
    
   // Ejecucion continua cada 100ms
   while (millis()-sync<100);
}
