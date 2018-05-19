#include <QTRSensors.h>

#define MOTOR_E1 6 //IN1 ptH
#define MOTOR_E2 5 //IN2 ptH
#define MOTOR_D1 11 //IN3 ptH
#define MOTOR_D2 10 //IN4 ptH

//======DEFINIÇÃO TEMPO DA LUTA=================
unsigned long tempoInicio = 0;
#define TEMPO_FIM 180000 //3 Minutos
//==============================================

#define QTD_SENS_BORDA_F 8 //QTD de sensores da frente

#define QTD_SENS_BORDA_T 2 //QTD de sensores de trás
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2 (serve só para o array e, pelo Proteus ,não está conectado na placa)



//int *valorSensorBordaF;
unsigned int ValoresQtrrc8[QTD_SENS_BORDA_F];
//int *valorSensorBordaT;
unsigned int ValoresQtrrc2[QTD_SENS_BORDA_T];

int velocidades [12] = { -255, -230, -200, 200, 230, 255};
//===============================================================================
QTRSensorsRC qtrrc2((unsigned char[]) {A4, A5}, QTD_SENS_BORDA_T, TIMEOUT, EMITTER_PIN);
QTRSensorsRC qtrrc8((unsigned char[]) {13, 12, 9, 8, 7, 4, 3, 2}, QTD_SENS_BORDA_F, TIMEOUT, EMITTER_PIN); 


void lerSensorBorda(unsigned int * valSensoresBorda, int tamanhoArray)
{
  if (tamanhoArray == 2)
    qtrrc2.read (valSensoresBorda);
  else
    qtrrc8.read (valSensoresBorda);
    
  for (int i = 0; i < tamanhoArray; i++)
  {
    valSensoresBorda[i] = ((valSensoresBorda[i] <= 1000) ? 1 : 0); //<=COR_BORDA definir #define. Vendo nada o retorno é 2500
  }

  
  //==============================================Vai pra debug
  for (unsigned char i = 0; i < tamanhoArray; i++)
  {
    Serial.print(valSensoresBorda[i]);   //Valor máximo do teste: 2500 2500. Diminue com branco
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  }
  Serial.println();
 //==============================================Vai pra debug 
  
}



//void detectarBorda (int * sensoresFrente, int * sensoresTras, int * retorno)
void detectarBorda (unsigned int * sensoresFrente, unsigned int * sensoresTras, int * retorno)
{
  //'retorno' é um array de 2 posições que será preenchido com um codigo para a função de reação.
  //Os sensores da frente são positivos os 2 de trás são negativos
  byte metadeQtdF = QTD_SENS_BORDA_F / 2;
  for (int i = 0; i <= QTD_SENS_BORDA_F; i++)
  {
    if (i <= metadeQtdF)
      retorno[0] += sensoresFrente[i];
    else
      retorno[1] += sensoresFrente[i];

    retorno[0] -= sensoresTras[1];
    retorno[1] -= sensoresTras[0];
  }
  return;
}
boolean detectaOpon() {
  return false;
}

void procurar ()
{
  Serial.println("Procurando");
  unsigned long tempo = 0;
  int valMaxRand = (sizeof(velocidades) / sizeof(velocidades[0]));
  int *codReacao = (int *) malloc (2 * sizeof (int));

  int randPosicao = 0;
  int velociRandE = 0;
  int velociRandD = 0;

  do
  {
    randPosicao = random (0, valMaxRand);     //entre min=0 e max-1
    velociRandE = velocidades[randPosicao];
    randPosicao = random (0, valMaxRand);    //entre min=0 e max-1
    velociRandD = velocidades[randPosicao];

    tempo = millis();
    //inicia a movimentação com as verificações
    do
    {
      movimentacao (velociRandE, velociRandD);
      lerSensorBorda(ValoresQtrrc8,8); //preenche o array de valores
      lerSensorBorda(ValoresQtrrc2,2); //preenche o array de valores
      detectarBorda(ValoresQtrrc8, ValoresQtrrc2, codReacao);
      
      if (codReacao[0] != 0 && codReacao[1] != 0)
      {
        reacaoBorda (codReacao);
      }
      else if (detectaOpon()) {}

    } while (millis() - tempo >= 3000);
    tempo = 0;


  } while ((millis() - tempo) <= TEMPO_FIM);

  free(codReacao);
  pinMode(13, OUTPUT);
  digitalWrite (13, HIGH);
  delay(2000);
  digitalWrite (13, LOW);
  pinMode(13, INPUT);
}

void movimentacao(int potenciaE, int potenciaD) {
  motorEsq(potenciaE);
  motorDir(potenciaD);
}


void reacaoBorda (int * codigoReacao)
{
  unsigned long tempo = 0;
  do {
    if (codigoReacao[0] > 0 && codigoReacao[1] > 0)
    {
      tempo = millis();
      do
      {
        movimentacao (-255, -255);

      } while (((millis() - tempo) <= 1500) || (codigoReacao[0] > 0 && codigoReacao[1] > 0) );
      tempo = 0;
      parar();
    }
    else if (codigoReacao[0] > 0 && codigoReacao[1] < 0)
    {
      tempo = millis();
      do
      {
        movimentacao (255, 200);
      } while (((millis() - tempo) <= 1500) || (codigoReacao[0] > 0 && codigoReacao[1] < 0));
      tempo = 0;
      parar();
    }
    else if (codigoReacao[0] < 0 && codigoReacao[1] > 0)
    {
      tempo = millis();
      do
      {
        movimentacao (200, 255);
      } while (((millis() - tempo) <= 1500) || (codigoReacao[0] < 0 && codigoReacao[1] > 0));
      tempo = 0;
      parar();
    }
    else if (codigoReacao[0] < 0 && codigoReacao[1] < 0)
    {
      tempo = millis();
      do
      {
        movimentacao (255, 255);
      } while (((millis() - tempo) <= 1500) || (codigoReacao[0] < 0 && codigoReacao[1] < 0));
      tempo = 0;
      parar();
    }
    else if (codigoReacao[0] > 0 && codigoReacao[1] == 0)
    {
      tempo = millis();
      do
      {
        do
        {
          movimentacao (255, 0);
        } while ((millis() - tempo) <= 1500);
        movimentacao (240, 110);
      } while (((millis() - tempo) <= 1000) || (codigoReacao[0] > 0 && codigoReacao[1] == 0));
      tempo = 0;
      parar();
    }
    else if (codigoReacao[0] == 0 && codigoReacao[1] > 0)
    {
      tempo = millis();
      do
      {
        do
        {
          movimentacao (0, 255);
        } while ((millis() - tempo) <= 1500);
        movimentacao (110, 240);
      } while (((millis() - tempo) <= 1000) || (codigoReacao[0] == 0 && codigoReacao[1] > 0));
      tempo = 0;
      parar();
    }
    else if ((codigoReacao[0] == 0 && codigoReacao[1] < 0) || (codigoReacao[0] < 0 && codigoReacao[1] == 0))
    {
      tempo = millis();

      do
      {
        movimentacao (255, 255);
      } while ((millis() - tempo) <= 1500);

      tempo = 0;
      parar();

    }
  } while (codigoReacao[0] != 0 && codigoReacao[1] != 0);
}

void parar ()
{
  motorDir(0);
  motorEsq(0);
}

//Com base no robô seguidor Marquinho
void motorDir (int potencia)
{
  if (potencia > 0) {
    digitalWrite(MOTOR_D2, LOW);
    analogWrite(MOTOR_D1, abs(potencia));
  }
  else
  {
    digitalWrite (MOTOR_D1, LOW);
    analogWrite (MOTOR_D2, abs(potencia));
  }
}
void motorEsq(int potencia)
{
  if (potencia > 0)
  {
    digitalWrite(MOTOR_E2, LOW);
    analogWrite(MOTOR_E1, abs(potencia));
  }
  else
  {
    digitalWrite(MOTOR_E1, LOW);
    analogWrite(MOTOR_E2, abs(potencia));
  }
}

void setup()
{

  delay(500);
  //(13, OUTPUT); em uso pelo arraySensor
 // digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc2.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    qtrrc8.calibrate();
  }
  //digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  
}
void loop() {
  
  procurar ();  
 
}
