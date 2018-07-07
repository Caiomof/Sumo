#include <SharpIR.h>

//=============DEBUG===================
#define DELAY 0
#define DEBUG_OPON 0
#define DEBUG_BORDA 0
#define DEBUG_MOTOR_BORDA 0
#define DEBUG_MOTOR_CORRECAO 0

//=========MOTORES====================
#define MOTOR_E1 5 //A-B1 ptH
#define MOTOR_E2 6 //A-A1 ptH
#define MOTOR_D1 10 //B-B1 ptH
#define MOTOR_D2 11 //B-A1 ptH

//===Sentidos/Estados dos MOTORES=======
#define FRENTE 1
#define PARADO 0
#define TRAS -1

//============VELOCIDADES================
#define VEL_MAX_PADRAO 90
#define VEL_MIN_PADRAO
#define VEL_ESQ_CURVA_DIR 100
#define VEL_DIR_CURVA_DIR 50
#define VEL_ESQ_CURVA_ESQ 50
#define VEL_DIR_CURVA_ESQ 100
#define VEL_GIRO 120


//==========TEMPO MOVIMENTAÇÃO==============
#define TEMPO_CURVA 300 //antes, 500
#define TEMPO_MOV_LINEAR 150 //PROVISÓRIO

//================TEMPO DA LUTA=================
unsigned long int tempoInicial = 0;
bool fim = false;
#define TEMPO_FIM 180000 //180s - 3 min

//==========DEFINIÇÕES SHARP_IR=================
#define INT 250 //número de interações do filtro
#define QTD_SENS_OPON 3
#define LIMITE 22

//==========PI========================
float base = 2.0;
unsigned long previous = 0;
const long intervalo = 500;

//========== CONSTANTES PI===========
#define KP 70
#define KI 0.001

#define MAX 180
#define VMAX 180
#define VBASE 0

SharpIR sensor0(GP2YA41SK0F, A3); //A1 com A3 estão invertidos por causa da conexão
SharpIR sensor1(GP2YA41SK0F, A2);
SharpIR sensor2(GP2YA41SK0F, A1);
int distance[QTD_SENS_OPON] = {0};


//===========DEFINIÇÕES IR_BORDA========================================================
#define QTD_SENS_BORDA 2
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low (não diminuir para 1000, pois os sensores do array ficarão sempre em 1)
#define EMITTER_PIN   A0   // emitter is controlled by digital pin A0 (serve apenas para o array)

int codReacao[2];//Array p/ armazenar Código para Reação de Borda
#define sensorF1 A5
#define sensorF2 A4
#define sensorD1 A6
#define sensorD2 A7
unsigned int valSensoresF[QTD_SENS_BORDA] = {sensorF1, sensorF2};
unsigned int valSensoresT[QTD_SENS_BORDA] = {sensorD1, sensorD2};
char PortasF[QTD_SENS_BORDA] = {sensorF1, sensorF2};
char PortasT[QTD_SENS_BORDA] = {sensorD1, sensorD2};

/* QTRSensorsRC qtrrc8((unsigned char[]) {
   A0, 12, 9, 8, 7, 4, 3, 2
   }, QTD_SENS_OPON, TIMEOUT, EMITTER_PIN);
   QTRSensorsRC qtrrc2((unsigned char[]) {
   A4, A5
   }, QTD_SENS_OPON, TIMEOUT, EMITTER_PIN); */

/*
   Função para leitura dos sensores de borda.
   Ela recebe o array que será preenchido com os valores e o tamanho
   do array. Pelo tamanho do Array será verificado se a solicitação é
   para os sensores da frente ou de trás.
*/
void lerSensorBorda(int tamanhoArray)
{
  for (int i = 0; i < tamanhoArray ; i++) {
    valSensoresF[i] = analogRead(PortasF[i]);
    valSensoresT[i] = analogRead(PortasT[i]);
  }

  for (int i = 0; i < tamanhoArray; i++)
  {
    valSensoresF[i] = ((valSensoresF[i] <= 80) ? 1 : 0); //<=COR_BORDA definir #define. Vendo nada o retorno é 2500
    valSensoresT[i] = ((valSensoresT[i] <= 80) ? 1 : 0); //<=COR_BORDA definir #define. Vendo nada o retorno é 2500
  }

  //================================================================================
  imprimirDebugBorda();
  //================================================================================
}
/*
   Função que verifica os array's de valores para cada sensor de borda e alimenta o array de retorno.
   'retorno' é um array de 2 posições que será preenchido com um codigo para a função de reação.
   Os sensores da frente são positivos. Dos 8 sensores do array da frente, a 1ª metade corresponde
   ao lado esquerdo, enquanto a 2ª metade, ao lado direito.
   As 2 unidades de sensores de trás são representadas por valor negativo.
*/
void detectarBorda ( int  *codReacao)
{
  codReacao[0] = valSensoresF[0];
  codReacao[1] = valSensoresF[1];

  codReacao[1] -= (valSensoresT[0]); //Valor do sensor trás-esq. na posição dir.
  codReacao[0] -= (valSensoresT[1]); //Valor do sensor trás-dir. na posição esq.
  return;
}

/*
   Função que recebe a "tupla" com código de reação e age em conformidade.
   A prioridade, neste caso, é não sair da arena, por isto, nesta função
   não será verificada a detecção de oponente.
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

        movimentacao (-VEL_GIRO, -VEL_GIRO);

        //================================================================================
        imprimirDebugMotor(-VEL_GIRO, VEL_GIRO, "(NÃO MAIS)Indo para trás");
        //================================================================================


        lerSensorBorda(QTD_SENS_BORDA); //preenche o array de valores
        detectarBorda(codReacao);

        //&& ao invés de ||: porque caso a detecção mude antes do fim de uma reação, ele já se corrige
      } while (((millis() - tempo) <= TEMPO_MOV_LINEAR) && (codigoReacao[0] > 0 && codigoReacao[1] > 0) );
      tempo = 0;
      gira();
    }
    else if (codigoReacao[0] > 0 && codigoReacao[1] < 0)
    {
      tempo = millis();
      do
      {

        movimentacao (-VEL_GIRO, -VEL_GIRO);

        //================================================================================
        imprimirDebugMotor(VEL_GIRO, -VEL_GIRO, "Curva para Direita");
        //================================================================================

        lerSensorBorda(QTD_SENS_BORDA); //preenche o array de valores
        detectarBorda(codReacao);

      } while (((millis() - tempo) <= TEMPO_CURVA) && (codigoReacao[0] > 0 && codigoReacao[1] < 0));

      tempo = 0;
      gira();
    }
    else if (codigoReacao[0] < 0 && codigoReacao[1] > 0)
    {
      tempo = millis();
      do
      {
        movimentacao (-VEL_GIRO, -VEL_GIRO);

        //================================================================================
        imprimirDebugMotor(-VEL_GIRO, VEL_GIRO, "Curva para Esquerda");
        //================================================================================

        lerSensorBorda(QTD_SENS_BORDA); //preenche o array de valores
        detectarBorda(codReacao);

      } while (((millis() - tempo) <= TEMPO_CURVA) && (codigoReacao[0] < 0 && codigoReacao[1] > 0));
      tempo = 0;
      gira();
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

        lerSensorBorda(QTD_SENS_BORDA); //preenche o array de valores
        detectarBorda(codReacao);

      } while (((millis() - tempo) <= 2000) && (codigoReacao[0] < 0 && codigoReacao[1] < 0));
      tempo = 0;
      gira();
    }
    else if (codigoReacao[0] > 0 && codigoReacao[1] == 0)
      //Aqui o robô gira no próprio eixo e depois faz curva na mesma direção
    {
      do {

        tempo = millis(); //Alimenta o tempo

        do
        {
          movimentacao (-VEL_GIRO, -VEL_GIRO);

          //================================================================================
          imprimirDebugMotor(VEL_GIRO, -VEL_GIRO, "Virando à Direita");
          //================================================================================

        } while ((millis() - tempo) <= TEMPO_CURVA); //Consome o tempo

        tempo = millis(); // Realimenta o tempo

        do
        {

          movimentacao (-VEL_GIRO, -VEL_GIRO);
          //================================================================================
          imprimirDebugMotor(-VEL_GIRO, VEL_GIRO, "Curva à Direita");
          //================================================================================

          lerSensorBorda(QTD_SENS_BORDA); //preenche o array de valores
          detectarBorda(codReacao);

          //&& ao invés de ||: porque caso a detecção mude antes do fim de uma reação, ele já se corrige
        } while (((millis() - tempo) <= TEMPO_CURVA) && (codigoReacao[0] > 0 && codigoReacao[1] == 0)); // Consome o tempo

      } while (codigoReacao[0] > 0 && codigoReacao[1] == 0);

      tempo = 0;
      gira();
    }

    else if (codigoReacao[0] == 0 && codigoReacao[1] > 0)
    {
      do {
        tempo = millis();
        do
        {

          movimentacao (-VEL_GIRO, -VEL_GIRO);
          //================================================================================
          imprimirDebugMotor(-VEL_GIRO, VEL_GIRO, "Virando à Esquerda");
          //========================================================================


        } while ((millis() - tempo) <= TEMPO_CURVA);

        tempo = millis();

        do
        {

          movimentacao (-VEL_GIRO, VEL_GIRO);
          //================================================================================
          imprimirDebugMotor(VEL_GIRO, -VEL_GIRO, "Curva à Esquerda");
          //================================================================================

          lerSensorBorda(QTD_SENS_BORDA); //preenche o array de valores
          detectarBorda(codReacao);


        } while (((millis() - tempo) <= TEMPO_CURVA) && codigoReacao[0] == 0 && codigoReacao[1] > 0);

      } while (codigoReacao[0] == 0 && codigoReacao[1] > 0);

      tempo = 0;
      gira();
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
      gira();

    }

    lerSensorBorda(QTD_SENS_BORDA); //preenche o array de valores
    detectarBorda(codReacao);

  } while (codigoReacao[0] != 0 || codigoReacao[1] != 0);
}
/*
   Função para preenchimento do array de valores dos sensores Sharp (ver oponente)
*/
void preenchimento()
{
  for (int i = 0; i < QTD_SENS_OPON; i++)
  {
    switch (i)
    {
      case 0: distance[0] = sensor0.getDistance();
      case 1: distance[1] = sensor1.getDistance();
      case 2: distance[2] = sensor2.getDistance();
    }
    //filtro
    for (int j = 0; j < INT ; j++)
    {
      distance[i] += distance[i] / INT;
    }
    if (distance[i] < LIMITE)
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

int erro_pi()
{
  int combin = detectaOpon();
  int erro[6] = { -45, -23, 0, 23, 45, 0};

  switch (combin)
  {
    case 0:
      return 0;
    case 1:
      imprimirDebugOpon ("DIREITA", combin, erro[0]);
      return erro[0]; // TA RETORNANDO 45
    case 2:
      imprimirDebugOpon ("MEIO", combin, erro[1]);
      return erro[2]; // TA RETORNANDO 0
    case 3:
      imprimirDebugOpon ("DIREITA E MEIO", combin, erro[2]);
      return erro[1]; // TA RETORNANDO 23
    case 4:
      imprimirDebugOpon ("ESQUERDA", combin, erro[2]);
      return erro[4]; // TA RETORNANDO -45
    case 5:
      /*
         SERIA O CASO DE SÓ O DA DIREITA E O DA ESQUERDA ESTAREM ACIONADOS
         E O DO MEIO NAO
         ISSO TEORICAMENTE É IMPOSSIVEL
      */
      break;
    case 6:
      imprimirDebugOpon ("ESQUERDA E MEIO", combin, erro[4]);
      return erro[3]; // TA RETORNANDO -23
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
  float pi;

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
  pi = (float)(KP * erro) + (KI * somaErro / (betaT));

  deltaT = millis();
  erroAnt = erro;
  return pi;
}

void seguir()
{
  //unsigned long int current = millis();

  if (detectaOpon())
  {
    float pi = 0;
    pi = correcao();
    controle(pi);

    //previous = current;
  }
  /* ESTRATEGIA AQUI!!! */
  else
  {
    /*
      // FREIO //
      if (current - previous <= intervalo)
      {
        current = millis();
        movimentacao(-15, -15); // VALOR PARA ELE PARAR
        Serial.println("Teste");
      }
      ///////////
    */
    // AQUI //
    movimentacao(MAX / 2, MAX / 2); // ESTRATEGIA AQUI!!
  }
  //Serial.println(current - previous);
  //////////////////////////////
}

/*
   Função inicial que executa os movimento de busca e
   verifica os sensores de borda e oponente para reagir
   caso necessário.
*/
void procurar ()
{

  unsigned long tempo = 0;
  //movimentacao (VEL_MAX_PADRAO, VEL_MAX_PADRAO); // COMENTAR QUANDO A "seguir()" FOR CORRIGIDA

  do
  {
    lerSensorBorda(QTD_SENS_BORDA); //preenche o array de valores
    detectarBorda(codReacao);
    seguir(); //==========================VERIFICAR PORQUE O PI ESTÁ SAINDO COMO ZERO NA FUNCAO "controle"
  } while (codReacao[0] == 0 && codReacao[1] == 0);
  Serial.println ("IDENTIFICOU A BORDA");
  reacaoBorda (codReacao);
}


void movimentacao(int potenciaE, int potenciaD)
{
  motorEsq(potenciaE);
  motorDir(potenciaD);
}

void controle(int pi)
{

  if ( pi < 0)
  {
    pi = map(pi, 0, 3150, 0, VMAX - VBASE); /*255 - Maximo*/

    motorDir(VBASE + pi);
    motorEsq(VBASE - pi);
    imprimirDebugMotorCorrecao(VBASE - pi, VBASE + pi, "PI Negativo ", pi);

  }
  if (pi > 0)
  {
    pi = map(pi, 3150, 0, VMAX - VBASE, 0 ); /*255 - Maximo*/

    motorDir(VBASE + pi);
    motorEsq(VBASE - pi);
    imprimirDebugMotorCorrecao(VBASE - pi, VBASE + pi, "PI Positivo ", pi);
  }
  if (!pi)
  {
    movimentacao(180, 180);
    imprimirDebugMotorCorrecao(MAX, MAX, "Para Frente ", pi);
  }
  //Serial.println(pi); //Print the value to the serial monitor
}

void gira()
{
    unsigned long tempo = 0;
      tempo = millis();
      do
      {

        movimentacao (-VEL_GIRO, VEL_GIRO);
        //================================================================================
        imprimirDebugMotor(-VEL_GIRO, VEL_GIRO, "Virando à Esquerda");
        //========================================================================
      } while ((millis() - tempo) <= 150);
          tempo = 0;
}


/*
   Com base no robô seguidor Marquinho
*/
void motorDir (int potencia)
{
  //DELAY == 0 É SINAL DE DEBUG DESATIVADO
  if (DELAY == 0)
  {
    if (potencia > 0)
    {
      digitalWrite(MOTOR_D2, LOW);
      analogWrite(MOTOR_D1, abs(potencia));
      //digitalWrite(MOTOR_D1, HIGH);
    }
    else if (potencia < 0)
    {
      digitalWrite (MOTOR_D1, LOW);
      analogWrite (MOTOR_D2, abs(potencia));
      //digitalWrite(MOTOR_D2, HIGH);
    }
    else
    {
      digitalWrite (MOTOR_D1, LOW);
      digitalWrite (MOTOR_D2, LOW);
    }
  }
}
void motorEsq(int potencia)
{
  if (DELAY == 0)
  {
    if (potencia > 0)
    {
      digitalWrite(MOTOR_E2, LOW);
      analogWrite(MOTOR_E1, abs(potencia));
      //digitalWrite(MOTOR_E1, HIGH);
    }
    else if (potencia < 0)
    {
      digitalWrite(MOTOR_E1, LOW);
      analogWrite(MOTOR_E2, abs(potencia));
      //digitalWrite(MOTOR_E2, HIGH);
    }
    else
    {
      digitalWrite(MOTOR_E1, LOW);
      digitalWrite(MOTOR_E2, LOW);
    }
  }
}

//===============FUNÇÕES DE DEBUG=====================================
/* Como a função de leitura de sensores Borda é chamada uma vez para
   cada array de sensor (de 8 e de 2), basta ter apenas uma entrada
   para valSensorBorda e tamanhoArray na função imprimirDebugBorda.
*/
void imprimirDebugBorda()
{
  if (DEBUG_BORDA)
  {

    Serial.print("Sensores da frente   ");
    for (unsigned char i = 0; i < QTD_SENS_BORDA; i++)
    {
      Serial.print(valSensoresF[i]);   //Valor máximo do teste: 2500 2500. Diminue com branco
      Serial.print('\t'); // tab para transformar em colunas
    }
    Serial.println();
    Serial.print("Sensores de tras     ");
    for (unsigned char i = 0; i < QTD_SENS_BORDA; i++)
    {
      Serial.print(valSensoresT[i]);   //Valor máximo do teste: 2500 2500. Diminue com branco
      Serial.print('\t'); // tab para transformar em colunas
    }
    Serial.println();
    delay (DELAY);
  }
}

void imprimirDebugMotor (int motorEsq, int motorDir, const char msg [25])
{
  if (DEBUG_MOTOR_BORDA)
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
void imprimirDebugMotorCorrecao  (int motorEsq, int motorDir, const char msg [25], int pi)
{
  if (DEBUG_MOTOR_CORRECAO)
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
void imprimirDebugOpon (const char posicaoSensor [20], int combin, int erro)
{
  if (DEBUG_OPON) {
    Serial.print(posicaoSensor); //Print the value to the serial monitor
    Serial.print("   |");
    Serial.println(combin); //Print the value to the serial monitor
    Serial.println("   |");
    Serial.println(erro);
    delay (DELAY);
  }
}
//==================================================================
void setup()
{
  Serial.begin(9600);
  pinMode(MOTOR_E1, OUTPUT);
  pinMode(MOTOR_E2, OUTPUT);
  pinMode(MOTOR_D1, OUTPUT);
  pinMode(MOTOR_D2, OUTPUT);

  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);

  pinMode(12, OUTPUT);//led
  digitalWrite(12, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode

  // Serial.println("Calibracao...");

  /* for (int i = 0; i < 100; i++) //A qtd de iterações pode ser modificado
     {
     qtrrc2.calibrate();
     qtrrc8.calibrate();
     }*/
  delay (5000); //DELAY OBRIGATÓRIO (5s)!!!!!!!!!
  digitalWrite(12, LOW);

}
void loop() {
  tempoInicial = millis();
  //if (!fim) //Para o robô parar assim que o tempo de luta for excedido
  //{
  int pi = 0;
  do
  {
    procurar();
  } while ((millis() - tempoInicial) <= TEMPO_FIM);
  //controle(pi);
  //pi = correcao(distance);
  //   fim = true;
  // }else
  //{
  // parar();
  // }
}
