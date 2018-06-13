#include <QTRSensors.h>
#include <SharpIR.h>

//=========MOTORES====================
#define MOTOR_E1 6 //IN1 ptH
#define MOTOR_E2 5 //IN2 ptH
#define MOTOR_D1 11 //IN3 ptH
#define MOTOR_D2 10 //IN4 ptH

//=============DEBUG===================
#define DELAY 500
#define DEBUG_OPON 1
#define DEBUG_BORDA 0
#define DEBUG_MOTOR 0

//=========Velocidades=================
#define VEL_MAX_PADRAO 255
#define VEL_MIN_PADRAO
#define VEL_ESQ_CURVA_DIR 200
#define VEL_DIR_CURVA_DIR 110
#define VEL_ESQ_CURVA_ESQ 110
#define VEL_DIR_CURVA_ESQ 150
#define VEL_GIRO 160
#define QTD_SENS_OPON 3
//========== SHARP IR=================
SharpIR sensor0(GP2YA41SK0F, A3);
SharpIR sensor1(GP2YA41SK0F, A2);
SharpIR sensor2(GP2YA41SK0F, A1);
int distance[QTD_SENS_OPON] = {0};

//==========PI========================
float base = 2.0;

#define VBASE 200
#define KP 70
#define KI 0.001

//==========Tempos de movimentação==============
#define TEMPO_CURVA 900
#define TEMPO_MOV_LINEAR 1000 //PROVISÓRIO

//================TEMPO DA LUTA=================
unsigned long tempoInicial = 0;
bool fim = false;
#define TEMPO_FIM 180000 //180s - 3 min


//===========DEFINIÇÕES IR_BORDA==================================================================
#define QTD_SENS_BORDA_F   8   // number of sensors used     8-2 sensores (por causa da falha)
#define QTD_SENS_BORDA_T 2   //QTD de sensores de trás
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low (não diminuir para 1000, pois os sensores do array ficarão sempre em 1)
#define EMITTER_PIN   A0   // emitter is controlled by digital pin A0 (serve só para o array)

int codReacao[2];//Array p/ armazenar Código para Reação de Borda

unsigned int ValoresQtrrc8[QTD_SENS_BORDA_F];
unsigned int ValoresQtrrc2[QTD_SENS_BORDA_T];


QTRSensorsRC qtrrc8((unsigned char[]) {
  8, 12, 9, 13, 7, 4, 3, 2
}, QTD_SENS_BORDA_F, TIMEOUT, EMITTER_PIN);
QTRSensorsRC qtrrc2((unsigned char[]) {
  A4, A5
}, QTD_SENS_BORDA_T, TIMEOUT, EMITTER_PIN);


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
    valSensoresBorda[i] = ((valSensoresBorda[i] <= 100) ? 1 : 0); //<=COR_BORDA definir #define. Vendo nada o retorno é 2500
  }
  //Contornando "falhas" da conexão pino D13
  valSensoresBorda[3] = 0;

  //================================================================================
  imprimirDebugBorda(valSensoresBorda, tamanhoArray);
  //================================================================================
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
      retorno[0] += sensoresFrente[i];//Valores dos sensores frente-esq. na posição esq.
    }

    else //(4..7)
    {
      retorno[1] += sensoresFrente[i];//Valor do sensor frente-dir. na posição dir.
    }
  }
  retorno[1] -= (sensoresTras[0]); //Valor do sensor trás-esq. na posição dir.
  retorno[0] -= (sensoresTras[1]); //Valor do sensor trás-dir. na posição esq.
  return;
}

/*
  Função para preenchimento do array de valores dos sensores Sharp (ver oponente)
*/
void preenchimento()
{
  distance[0] = sensor0.getDistance();
  distance[1] = sensor1.getDistance();
  distance[2] = sensor2.getDistance();

  for (int i = 0; i < 3; i++)
  {
    if (distance[i] < 12)
    {
      //Serial.print("Distancia "); Serial.print(distance[i]);
      distance[i] = (int)(pow(base, i) + 0.5);
    }
    else
      distance[i] = 0;
  }
}

/*
  Função que lerá os valores dos sensores Sharp e determinará ou não a detecção
*/
int detectaOpon()
{
  int ret = 0;

  preenchimento();

  for (int i = 0; i < 3; i++)
    ret += distance[i];

  return ret;
}

float erro_pi()
{
  int combin = detectaOpon();
  int erro[6] = {45, 0, 23, -45, -23, 0};

  delay(500);

  switch (combin)
  {
    case 1:
      imprimirDebugOpon ("DIREITA", combin, erro[0]);
      return erro[0]; // TA RETORNANDO 45
    case 2:
      imprimirDebugOpon ("MEIO", combin, erro[1]);
      return erro[1]; // TA RETORNANDO 0
    case 3:
      imprimirDebugOpon ("DIREITA MEIO", combin, erro[2]);
      return erro[2]; // TA RETORNANDO 23
    case 4:
      imprimirDebugOpon ("ESQUERDA", combin, erro[3]);
      return erro[3]; // TA RETORNANDO -45
    case 5:
      /*
         SERIA O CASO DE SÓ O DA DIREITA E O DA ESQUERDA ESTAREM ACIONADOS
         E O DO MEIO NAO
         ISSO TEORICAMENTE É IMPOSSIVEL
      */
      break;
    case 6:
      imprimirDebugOpon ("ESQUERDA E MEIO", combin, erro[4]);
      return erro[4]; // TA RETORNANDO -23
    case 7:
      imprimirDebugOpon ("OS TRES", combin, erro[5]);
      return erro[5]; // TA RETORNANDO 0
  }
}

int correcao()
{
  static float erroAnt = 0, somaErro = 0;
  static long int deltaT = 1, betaT = 1;
  float erro;
  int pi;

  if (deltaT != 1)
    deltaT = millis() - deltaT;

  if ((erro = erro_pi()) == 0.0)
  {
    betaT = deltaT;
    somaErro = 0;
  }
  else
  {
    betaT += deltaT;
    somaErro += erro;
  }

  //DEFINIR VALOR DAS CONSTANTES
  pi = (int)(KP * erro) + (KI * somaErro / (betaT));

  deltaT = millis();
  erroAnt = erro;

  //Serial.println(pi); //Print the value to the serial monitor

  return pi;
}

void controle(int pi)
{

  if ( pi < 0)
  {
    pi = map(pi, 0, 3150, 0, (255 - VBASE)); /*255 - Maximo*/
    motorDir(VBASE - pi);
    motorEsq(VBASE + pi);

    //================================================================================
    imprimirDebugMotorPI ((VBASE + pi), (VBASE - pi), "PI < 0", pi);
    //================================================================================
  }
  else if (pi > 0)
  {
    pi = map(pi, 3150, 0, (255 - VBASE), 0 ); /*255 - Maximo*/
    motorDir(VBASE - pi);
    motorEsq(VBASE + pi);
    //================================================================================
    imprimirDebugMotorPI ((VBASE + pi), (VBASE - pi), "PI > 0", pi);
    //================================================================================
  }
  else if (pi == 0)
  {
    movimentacao (255, 255);
    //================================================================================
    imprimirDebugMotorPI (255, 255, "PI == 0", pi);
    //================================================================================
  }
}

/*
  Função que recebe a "tupla" com código de reação e age em conformidade.
  A prioridade é não sair da arena, por isto, nesta função não será verificada
  a detecção de oponente.
*/
void reacaoBorda (int * codigoReacao)
{
  //Serial.println("Reagindo aa borda");
  unsigned long tempo = 0;

  do {
    if (codigoReacao[0] > 0 && codigoReacao[1] > 0)
    {
      tempo = millis();

      do
      {
        movimentacao (-VEL_MAX_PADRAO, -VEL_MAX_PADRAO);

        //================================================================================
        imprimirDebugMotor(-VEL_MAX_PADRAO, -VEL_MAX_PADRAO, "Indo para trás");
        //================================================================================

        lerSensorBorda(ValoresQtrrc8, QTD_SENS_BORDA_F); //preenche o array de valores
        lerSensorBorda(ValoresQtrrc2, QTD_SENS_BORDA_T); //preenche o array de valores
        detectarBorda(ValoresQtrrc8, ValoresQtrrc2, codReacao);

        //&& ao invés de ||: porque caso a detecção mude antes do fim de uma reação, ele já se corrige
      } while (((millis() - tempo) <= TEMPO_MOV_LINEAR) && (codigoReacao[0] > 0 && codigoReacao[1] > 0) );
      tempo = 0;
      //parar();
    }
    else if (codigoReacao[0] > 0 && codigoReacao[1] < 0)
    {
      tempo = millis();
      do
      {
        movimentacao (VEL_ESQ_CURVA_DIR, VEL_DIR_CURVA_DIR);

        //================================================================================
        imprimirDebugMotor(255, 200, "Curva para Direita");
        //================================================================================

        lerSensorBorda(ValoresQtrrc8, QTD_SENS_BORDA_F); //preenche o array de valores
        lerSensorBorda(ValoresQtrrc2, QTD_SENS_BORDA_T); //preenche o array de valores
        detectarBorda(ValoresQtrrc8, ValoresQtrrc2, codReacao);

      } while (((millis() - tempo) <= TEMPO_CURVA) && (codigoReacao[0] > 0 && codigoReacao[1] < 0));

      tempo = 0;
      //parar();
    }
    else if (codigoReacao[0] < 0 && codigoReacao[1] > 0)
    {
      tempo = millis();
      do
      {
        movimentacao (VEL_ESQ_CURVA_ESQ, VEL_DIR_CURVA_ESQ);

        //================================================================================
        imprimirDebugMotor(VEL_ESQ_CURVA_ESQ, VEL_DIR_CURVA_ESQ, "Curva para Esquerda");
        //================================================================================

        lerSensorBorda(ValoresQtrrc8, QTD_SENS_BORDA_F); //preenche o array de valores
        lerSensorBorda(ValoresQtrrc2, QTD_SENS_BORDA_T); //preenche o array de valores
        detectarBorda(ValoresQtrrc8, ValoresQtrrc2, codReacao);

      } while (((millis() - tempo) <= TEMPO_CURVA) && (codigoReacao[0] < 0 && codigoReacao[1] > 0));
      tempo = 0;
      //parar();
    }
    else if (codigoReacao[0] < 0 && codigoReacao[1] < 0)
    {
      tempo = millis();
      do
      {
        movimentacao (VEL_MAX_PADRAO, VEL_MAX_PADRAO);

        //================================================================================
        imprimirDebugMotor(VEL_MAX_PADRAO, VEL_MAX_PADRAO, "Para Frente");
        //================================================================================

        lerSensorBorda(ValoresQtrrc8, QTD_SENS_BORDA_F); //preenche o array de valores
        lerSensorBorda(ValoresQtrrc2, QTD_SENS_BORDA_T); //preenche o array de valores
        detectarBorda(ValoresQtrrc8, ValoresQtrrc2, codReacao);

      } while (((millis() - tempo) <= 2000) && (codigoReacao[0] < 0 && codigoReacao[1] < 0));
      tempo = 0;
      //parar();
    }
    else if (codigoReacao[0] > 0 && codigoReacao[1] == 0)
      //Aqui o robô gira no próprio eixo e depois faz curva na mesma direção
    {
      do {

        tempo = millis(); //Alimenta o tempo

        do
        {
          movimentacao (VEL_GIRO, -VEL_GIRO);

          //================================================================================
          imprimirDebugMotor(VEL_GIRO, -VEL_GIRO, "Virando à Direita");
          //================================================================================

        } while ((millis() - tempo) <= TEMPO_CURVA); //Consome o tempo

        tempo = millis(); // Realimenta o tempo

        do
        {
          movimentacao (VEL_ESQ_CURVA_DIR, VEL_DIR_CURVA_DIR);

          //================================================================================
          imprimirDebugMotor(VEL_ESQ_CURVA_DIR, VEL_DIR_CURVA_DIR, "Curva à Direita");
          //================================================================================

          lerSensorBorda(ValoresQtrrc8, QTD_SENS_BORDA_F); //preenche o array de valores
          lerSensorBorda(ValoresQtrrc2, 2); //preenche o array de valores
          detectarBorda(ValoresQtrrc8, ValoresQtrrc2, codReacao);

          //&& ao invés de ||: porque caso a detecção mude antes do fim de uma reação, ele já se corrige
        } while (((millis() - tempo) <= TEMPO_CURVA) && (codigoReacao[0] > 0 && codigoReacao[1] == 0)); // Consome o tempo

      } while (codigoReacao[0] > 0 && codigoReacao[1] == 0);

      tempo = 0;
      //parar();
    }

    else if (codigoReacao[0] == 0 && codigoReacao[1] > 0)
    {
      do {
        tempo = millis();
        do
        {
          movimentacao (-VEL_GIRO, VEL_GIRO);

          //================================================================================
          imprimirDebugMotor(-VEL_GIRO, VEL_GIRO, "Virando à Esquerda");
          //================================================================================

        } while ((millis() - tempo) <= TEMPO_CURVA);

        tempo = millis();

        do
        {
          movimentacao (VEL_ESQ_CURVA_ESQ, VEL_DIR_CURVA_ESQ);

          //================================================================================
          imprimirDebugMotor(VEL_ESQ_CURVA_ESQ, VEL_DIR_CURVA_ESQ, "Curva à Esquerda");
          //================================================================================

          lerSensorBorda(ValoresQtrrc8, QTD_SENS_BORDA_F); //preenche o array de valores
          lerSensorBorda(ValoresQtrrc2, 2); //preenche o array de valores
          detectarBorda(ValoresQtrrc8, ValoresQtrrc2, codReacao);


        } while (((millis() - tempo) <= TEMPO_CURVA) && codigoReacao[0] == 0 && codigoReacao[1] > 0);

      } while (codigoReacao[0] == 0 && codigoReacao[1] > 0);

      tempo = 0;
      //parar();
    }
    else if ((codigoReacao[0] == 0 && codigoReacao[1] < 0) || (codigoReacao[0] < 0 && codigoReacao[1] == 0))
    {
      tempo = millis();

      do
      {
        movimentacao (VEL_MAX_PADRAO, VEL_MAX_PADRAO);

        //================================================================================
        imprimirDebugMotor(VEL_MAX_PADRAO, VEL_MAX_PADRAO, "Indo para Frente");
        //================================================================================
      } while ((millis() - tempo) <= TEMPO_MOV_LINEAR);

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
  //Serial.println("Nova procura\n");
  //movimentacao (255, 255);

  lerSensorBorda(ValoresQtrrc8, QTD_SENS_BORDA_F); //preenche o array de valores
  lerSensorBorda(ValoresQtrrc2, QTD_SENS_BORDA_T); //preenche o array de valores
  detectarBorda(ValoresQtrrc8, ValoresQtrrc2, codReacao);

  if (codReacao[0] != 0 || codReacao[1] != 0)
  {
    reacaoBorda (codReacao);
  }
  else if (detectaOpon())
  {
    int pi = 0;
    pi = correcao();
    controle(pi);

    //Serial.println("Fim da procura\n");
  }
}

void movimentacao(int potenciaE, int potenciaD) {
  motorEsq(potenciaE);
  motorDir(potenciaD);
}

void parar ()
{
  motorDir(0);
  motorEsq(0);
  //Serial.println("parou");
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

//===============FUNÇÕES DE DEBUD=====================================

/* Como a função de leitura de sensores Borda é chamada uma vez para
   cada array de sensor (de 8 e de 2), basta ter apenas uma entrada
   para valSensorBorda e tamanhoArray na função imprimirDebugBorda.
*/
void imprimirDebugBorda(unsigned int * valSensoresBorda, int tamanhoArray)
{
  if (DEBUG_BORDA)
  {
    if (tamanhoArray == 2)
      Serial.print("Array de sensores da frente(1..8)   "); else
      Serial.print("Sensores de tras(1..2)              ");
    for (unsigned char i = 0; i < tamanhoArray; i++)
    {
      Serial.print(valSensoresBorda[i]);   //Valor máximo do teste: 2500 2500. Diminue com branco
      Serial.print('\t'); // tab para transformar em colunas
    }
    Serial.println();
    delay (DELAY);
  }
}
void imprimirDebugMotor (int motorEsq, int motorDir, const char msg [25]) {
  if (DEBUG_MOTOR)
  {
    Serial.print("Codigo da Reacao:");
    Serial.print("  ");
    int codigoReacao = codReacao[0];
    Serial.print (codigoReacao);
    Serial.print("   ");
    codigoReacao = codReacao[1];
    Serial.print(codigoReacao);
    Serial.print("   |");
    Serial.print (msg);
    Serial.print("| Motores: ");
    Serial.print (motorEsq);
    Serial.print('\t');
    Serial.println (motorDir);
    delay (DELAY);
  }
}
void imprimirDebugMotorPI (int motorEsq, int motorDir, const char msg [25], int pi) {

  if (DEBUG_MOTOR)
  {
    Serial.print("PI:");
    Serial.print("  ");
    Serial.print (pi);
    Serial.print("   |");
    Serial.print (msg);
    Serial.print("| Motores: ");
    Serial.print (motorEsq);
    Serial.print('\t');
    Serial.println (motorDir);
    delay (DELAY);
  }

}

void imprimirDebugOpon (const char posicaoSensor [20], int combin, int erro) {
  if (DEBUG_OPON) {
    Serial.print(posicaoSensor); //Print the value to the serial monitor
    Serial.print("   |");
    Serial.println(combin); //Print the value to the serial monitor
    Serial.println("   |");
    Serial.println(erro);
  }
}
//==================================================================
void setup()
{
  Serial.begin(9600);
  //delay(500);
  pinMode(13, OUTPUT);// em uso pelo arraySensor
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode

  Serial.println("Calibracao...");

  for (int i = 0; i < 100; i++) //A qtd de iterações pode ser modificado
  {
    qtrrc2.calibrate();
    qtrrc8.calibrate();
  }
  digitalWrite(13, LOW);

}
void loop() {
  tempoInicial = millis();
  //if (!fim) //Para o robô parar assim que o tempo de luta for excedido
  //{
  do
  {
    procurar ();
  } while ((millis() - tempoInicial) <= TEMPO_FIM);
  //   fim = true;
  // }else
  //{
  // parar();
  // }
}
