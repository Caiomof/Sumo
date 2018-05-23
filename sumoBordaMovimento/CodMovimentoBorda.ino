#include <QTRSensors.h>

#define MOTOR_E1 6 //IN1 ptH
#define MOTOR_E2 5 //IN2 ptH
#define MOTOR_D1 11 //IN3 ptH
#define MOTOR_D2 10 //IN4 ptH

#define NUM_VELOCIDADES 6 //Número de tipos de velocidades (para o mov. aleatório)

//======DEFINIÇÃO TEMPO DA LUTA=================
unsigned long tempoInicial = 0;
bool fim = false;
#define TEMPO_FIM 180 //180s - 3 min
//==============================================
int codReacao[2];//Array p/ armazenar Código para Reação de Borda



#define QTD_SENS_BORDA_F   8     // number of sensors used     8-2 sensores (por causa da falha)

#define QTD_SENS_BORDA_T 2 //QTD de sensores de trás
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low (não diminuir para 1000, pois os sensores do array ficarão sempre em 1)
#define EMITTER_PIN   A0     // emitter is controlled by digital pin 2 (serve só para o array e, pelo Proteus, não está conectado na placa. A conexão será feita)


unsigned int ValoresQtrrc8[QTD_SENS_BORDA_F];
unsigned int ValoresQtrrc2[QTD_SENS_BORDA_T];

int velocidades [NUM_VELOCIDADES] = {-255, -230, -200, 200, 230, 255};
//===============================================================================
/*
  Pino D13 foi transportado para o 4º sensor e receberá valor sempre 0, pois está enviando valores sempre baixos (1).
  O penúltimo sensor (na porta 3) está enviando valor sempre alto (0), mas isso poderá ser ignorado, pois ele não 
  está na extremidade e está sempre como 0.    
*/
QTRSensorsRC qtrrc8((unsigned char[]) {8, 12, 9, 13, 7, 4, 3, 2}, QTD_SENS_BORDA_F, TIMEOUT, EMITTER_PIN); 

QTRSensorsRC qtrrc2((unsigned char[]) {A4, A5}, QTD_SENS_BORDA_T, TIMEOUT, EMITTER_PIN);



/*
  Função para leitura dos sensores de borda.
  Ela recebe o array que será preenchido com os valores e o tamanho
  do array. Pelo tamanho do Array será verificado se a solicitação é
  para os sensores da frente ou de trás.
*/
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
  //Contornando falhas nos sensores
  valSensoresBorda[3] = 0; 
    
  //==============================================Vai pra debug

  if (tamanhoArray == 2)
    Serial.println("Sensores de tras (1 e 2)");
  else
    Serial.println("Array de sensores da frente (1..8)");
  for (unsigned char i = 0; i < tamanhoArray; i++)
  {
    Serial.print(valSensoresBorda[i]);   //Valor máximo do teste: 2500 2500. Diminue com branco
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  }
  Serial.println();
  delay(1000);
  //==============================================Vai pra debug
 
}

/*
  Função que verifica os array's de valores para cada sensor de borda e alimenta o array de retorno.
  'retorno' é um array de 2 posições que será preenchido com um codigo para a função de reação.
  Os sensores da frente são positivos. Dos 8 sensores do array da frente, a 1ª metade corresponde
  ao lado esquerdo, enquanto a 2ª metade, ao lado direito.
  As 2 unidades de sensores de trás são representadas por valor negativo.
*/
void detectarBorda (unsigned int * sensoresFrente, unsigned int * sensoresTras, int * retorno)
{
  retorno[0] = 0;
  retorno[1] = 0;

  int metadeQtdF = QTD_SENS_BORDA_F / 2;
  for (int i = 0; i < QTD_SENS_BORDA_F; i++)//(0..7)==(8) posições
  {
    if (i < metadeQtdF) //(0..3)
    {
      retorno[0] += sensoresFrente[i];
      Serial.print("FE ");
      Serial.println(retorno[0]);
    }

    else //(4..7)
    {
      retorno[1] += sensoresFrente[i];
      Serial.print("FD ");
      Serial.println(retorno[1]);
    }
  }

  retorno[1] -= (sensoresTras[0]); //Valor do sensor esq. na posição dir.
  Serial.print("TE ");
  Serial.println(retorno[1]);
  
  retorno[0] -= (sensoresTras[1]); //Valor do sensor dir. na posição esq.
  Serial.print("TD ");
  Serial.println(retorno[0]);
 

  //==============================================Vai pra debug

  Serial.print("Codigo de reacao: ");
  Serial.print(retorno[0]);
  Serial.print('\t');
  Serial.println(retorno[1]);
  //==============================================Vai pra debug

  return;
}
boolean detectaOpon() {
  return false;
}

/*
  Função que recebe a "tupla" com código de reação e age em conformidade
*/
void reacaoBorda (int * codigoReacao)
//A prioridade é não sair da arena, logo nesta função não será verificada a detecção de oponente.
{
  //Serial.println("Reagindo aa borda");
  unsigned long tempo = 0;
  do {
    if (codigoReacao[0] > 0 && codigoReacao[1] > 0) 
    {
      tempo = millis();
      do
      {
        movimentacao (-255, -255);

        lerSensorBorda(ValoresQtrrc8, QTD_SENS_BORDA_F); //preenche o array de valores
        lerSensorBorda(ValoresQtrrc2, QTD_SENS_BORDA_T); //preenche o array de valores
        detectarBorda(ValoresQtrrc8, ValoresQtrrc2, codReacao);

        Serial.println("Indo para trás");
        
      } while (((millis() - tempo) <= 2000) || (codigoReacao[0] > 0 && codigoReacao[1] > 0) );
      tempo = 0;
      //parar();
    }
    else if (codigoReacao[0] > 0 && codigoReacao[1] < 0)
    {
      tempo = millis();
      do
      {
        movimentacao (255, 200);

        lerSensorBorda(ValoresQtrrc8, QTD_SENS_BORDA_F); //preenche o array de valores
        lerSensorBorda(ValoresQtrrc2, QTD_SENS_BORDA_T); //preenche o array de valores
        detectarBorda(ValoresQtrrc8, ValoresQtrrc2, codReacao);

        Serial.println("Curva para Direita");

      } while (((millis() - tempo) <= 2000) || (codigoReacao[0] > 0 && codigoReacao[1] < 0));
      tempo = 0;
      //parar();
    }
    else if (codigoReacao[0] < 0 && codigoReacao[1] > 0)
    {
      tempo = millis();
      do
      {
        movimentacao (200, 255);

        lerSensorBorda(ValoresQtrrc8, QTD_SENS_BORDA_F); //preenche o array de valores
        lerSensorBorda(ValoresQtrrc2, QTD_SENS_BORDA_T); //preenche o array de valores
        detectarBorda(ValoresQtrrc8, ValoresQtrrc2, codReacao);

        Serial.println("Curva para Esquerda");

      } while (((millis() - tempo) <= 2000) || (codigoReacao[0] < 0 && codigoReacao[1] > 0));
      tempo = 0;
      //parar();
    }
    else if (codigoReacao[0] < 0 && codigoReacao[1] < 0)
    {
      tempo = millis();
      do
      {
        movimentacao (255, 255);

        lerSensorBorda(ValoresQtrrc8, QTD_SENS_BORDA_F); //preenche o array de valores
        lerSensorBorda(ValoresQtrrc2, QTD_SENS_BORDA_T); //preenche o array de valores
        detectarBorda(ValoresQtrrc8, ValoresQtrrc2, codReacao);

        Serial.println("Para Frente");

      } while (((millis() - tempo) <= 2000) || (codigoReacao[0] < 0 && codigoReacao[1] < 0));
      tempo = 0;
      //parar();
    }
    else if (codigoReacao[0] > 0 && codigoReacao[1] == 0)
      //Aqui o robô gira no próprio eixo e depois faz curva na mesma direção
    {
      tempo = millis();
      do
      {
        do
        {
          movimentacao (255, 0);
        
          Serial.println("Virando à Direita");
        
        } while ((millis() - tempo) <= 1500);
        movimentacao (240, 200);

        lerSensorBorda(ValoresQtrrc8, QTD_SENS_BORDA_F); //preenche o array de valores
        lerSensorBorda(ValoresQtrrc2, 2); //preenche o array de valores
        detectarBorda(ValoresQtrrc8, ValoresQtrrc2, codReacao);

        Serial.println("Curava à Direita");

      } while (((millis() - tempo) <= 2000) || (codigoReacao[0] > 0 && codigoReacao[1] == 0));
      tempo = 0;
      //parar();
    }
    else if (codigoReacao[0] == 0 && codigoReacao[1] > 0)
    {
      tempo = millis();
      do
      {
        do
        {
          movimentacao (0, 255);

          Serial.println("Virando à Esquerda");
          
        } while ((millis() - tempo) <= 1500);
        movimentacao (200, 240);

        lerSensorBorda(ValoresQtrrc8, QTD_SENS_BORDA_F); //preenche o array de valores
        lerSensorBorda(ValoresQtrrc2, 2); //preenche o array de valores
        detectarBorda(ValoresQtrrc8, ValoresQtrrc2, codReacao);

        Serial.println("Curava à Esquerda");

      } while (((millis() - tempo) <= 2000) || (codigoReacao[0] == 0 && codigoReacao[1] > 0));
      tempo = 0;
      //parar();
    }
    else if ((codigoReacao[0] == 0 && codigoReacao[1] < 0) || (codigoReacao[0] < 0 && codigoReacao[1] == 0))
    {
      tempo = millis();

      do
      {
        movimentacao (255, 255);

                Serial.println("Indo para Frente");

      } while ((millis() - tempo) <= 2000);

      tempo = 0;
      //parar();

    }

    lerSensorBorda(ValoresQtrrc8, QTD_SENS_BORDA_F); //preenche o array de valores
    lerSensorBorda(ValoresQtrrc2, 2); //preenche o array de valores
    detectarBorda(ValoresQtrrc8, ValoresQtrrc2, codReacao);

  } while (codigoReacao[0] != 0 || codigoReacao[1] != 0);
}

/*
  Função inicial que executa os movimento de busca e
  verifica os sensores de borda e oponente para reagir
  caso necessário.
*/
void procurar ()
{
  Serial.println("Nova procura");

  unsigned long tempoMovimento = 0;
  int valMaxRand = (sizeof(velocidades) / sizeof(velocidades[0]));


  int randPosicao = 0;
  int velociRandE = 0;
  int velociRandD = 0;

  randomSeed(analogRead(A6)); //Uma porta analógica desocupada
  randPosicao = random (0, valMaxRand);     //entre min=0 e max-1
  velociRandE = velocidades[randPosicao];

  randomSeed(analogRead(A6));
  randPosicao = random (0, valMaxRand);    //entre min=0 e max-1
  velociRandD = velocidades[randPosicao];
  velociRandD *= (((velociRandE < 0)&&(velociRandD < 0)) ? (-1) : 1); //Evita que o robô dê ré durante a busca.

  tempoMovimento = millis();
  //inicia a movimentação com as verificações
  do
  {
    movimentacao (velociRandE, velociRandD);
    lerSensorBorda(ValoresQtrrc8, QTD_SENS_BORDA_F); //preenche o array de valores
    lerSensorBorda(ValoresQtrrc2, QTD_SENS_BORDA_T); //preenche o array de valores
    detectarBorda(ValoresQtrrc8, ValoresQtrrc2, codReacao);

    if (codReacao[0] != 0 || codReacao[1] != 0)
    {
      reacaoBorda (codReacao);
    }
    else if (detectaOpon()) {}

//===============================Debug
   Serial.print("Aleatorio com: ");
   Serial.print(velociRandE);
   Serial.print('\t');
   Serial.println(velociRandD);
//===============================Debug

  } while ((millis() - tempoMovimento) <= 3000);

  Serial.println("Fim da procura");


}

void movimentacao(int potenciaE, int potenciaD) {
  motorEsq(potenciaE);
  motorDir(potenciaD);
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

  
  Serial.begin(9600);
  delay(500);
  pinMode(13, OUTPUT);// em uso pelo arraySensor
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode

  Serial.println("Calibracao...");

  for (int i = 0; i < 300; i++) //A qtd de iterações pode ser modificado
  {
    qtrrc2.calibrate();
    qtrrc8.calibrate();
  }
  digitalWrite(13, LOW);

}
void loop() {
  tempoInicial = millis();
  //if (!fim)
  //{
    do
    {
      procurar ();
    } while ((millis() - tempoInicial) <= TEMPO_FIM);
 //   fim = true;
 // }else 
  //{
  parar();
 // }
}
