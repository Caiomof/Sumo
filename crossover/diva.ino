//#include <QTRSensors.h>
#include <SharpIR.h>

//=========MOTORES====================
#define MOTOR_D1 6 //IN2 ptH - DIREITO
#define MOTOR_D2 5 //IN1 ptH - DIREITO
#define MOTOR_E1 11 //IN4 ptH - ESQUERDO
#define MOTOR_E2 10 //IN3 ptH - ESQUERDO

//=============DEBUG===================
#define DELAY 500
#define DEBUG_OPON 0
#define DEBUG_BORDA 1
#define DEBUG_MOTOR_BORDA 0
#define DEBUG_MOTOR_CORRECAO 0

//===Sentidos/Estados dos motores=======
#define FRENTE 1
#define PARADO 0
#define TRAS -1

//========== SHARP IR=================
#define INT 250 //número de interaçoes do filtro
#define QTD_SENS_OPON 3
#define LIMITE 12

//========== CONSTANTES PI===========
#define KP 70
#define KI 0.000001

#define VBASE 200

SharpIR sensor0(GP2YA41SK0F, A1);
SharpIR sensor1(GP2YA41SK0F, A2);
SharpIR sensor2(GP2YA41SK0F, A3);
int distance[QTD_SENS_OPON] = {0};

//==========PI========================
float base = 2.0;
//==========Tempos de movimentação==============
#define TEMPO_CURVA 500 //antes, 900
#define TEMPO_MOV_LINEAR 1000 //PROVISÓRIO

//================TEMPO DA LUTA=================
unsigned long tempoInicial = 0;
bool fim = false;
#define TEMPO_FIM 180000 //180s - 3 min

//===========DEFINIÇÕES IR_BORDA==================================================================
#define QTD_SENS_OPON 2
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low (não diminuir para 1000, pois os sensores do array ficarão sempre em 1)
#define EMITTER_PIN   A7   // emitter is controlled by digital pin A0 (serve só para o array)

int codReacao[2];//Array p/ armazenar Código para Reação de Borda
#define sensorF1 A4
#define sensorF2 A5
#define sensorD1 A6
#define sensorD2 A7
unsigned int SensoresF[QTD_SENS_OPON] = {sensorF1, sensorF2};
unsigned int SensoresT[QTD_SENS_OPON] = {sensorD1, sensorD2};
char PortasF[QTD_SENS_OPON] = {sensorF1, sensorF2};
char PortasT[QTD_SENS_OPON] = {sensorD1, sensorD2};



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
        SensoresF[i] = analogRead(PortasF[i]);
        SensoresT[i] = analogRead(PortasT[i]);
    }

    for (int i = 0; i < tamanhoArray; i++)
    {
        SensoresF[i] = ((SensoresF[i] <= 100) ? 1 : 0); //<=COR_BORDA definir #define. Vendo nada o retorno é 2500
        SensoresT[i] = ((SensoresT[i] <= 100) ? 1 : 0); //<=COR_BORDA definir #define. Vendo nada o retorno é 2500
    }
    //Contornando "falhas" da conexão pino D13
    // valSensoresBorda[3] = 0;

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
    // codReacao[0] 0;
    // codReacao[1] = 0;

    //int metadeQtdF = QTD_SENS_OPON / 2;

    codReacao[0] = SensoresF[0];
    codReacao[1] = SensoresF[1];

    codReacao[1] -= (SensoresT[0]); //Valor do sensor trás-esq. na posição dir.
    codReacao[0] -= (SensoresT[1]); //Valor do sensor trás-dir. na posição esq.
    return;
}
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
                movimentacao (TRAS, TRAS);

                //================================================================================
                imprimirDebugMotor(TRAS, TRAS, "Indo para trás");
                //================================================================================

                lerSensorBorda(QTD_SENS_OPON); //preenche o array de valores
                detectarBorda(codReacao);

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
                movimentacao (FRENTE, PARADO);

                //================================================================================
                imprimirDebugMotor(FRENTE, PARADO, "Curva para Direita");
                //================================================================================

                lerSensorBorda(QTD_SENS_OPON); //preenche o array de valores
                detectarBorda(    codReacao);

            } while (((millis() - tempo) <= TEMPO_CURVA) && (codigoReacao[0] > 0 && codigoReacao[1] < 0));

            tempo = 0;
            //parar();
        }
        else if (codigoReacao[0] < 0 && codigoReacao[1] > 0)
        {
            tempo = millis();
            do
            {
                movimentacao (PARADO, FRENTE);

                //================================================================================
                imprimirDebugMotor(PARADO, FRENTE, "Curva para Esquerda");
                //================================================================================

                lerSensorBorda(  QTD_SENS_OPON); //preenche o array de valores
                detectarBorda(    codReacao);

            } while (((millis() - tempo) <= TEMPO_CURVA) && (codigoReacao[0] < 0 && codigoReacao[1] > 0));
            tempo = 0;
            //parar();
        }
        else if (codigoReacao[0] < 0 && codigoReacao[1] < 0)
        {
            tempo = millis();
            do
            {
                movimentacao (FRENTE, FRENTE);

                //================================================================================
                imprimirDebugMotor(FRENTE, FRENTE, "Para Frente");
                //================================================================================

                lerSensorBorda(  QTD_SENS_OPON); //preenche o array de valores
                detectarBorda(    codReacao);

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
                    movimentacao (FRENTE, TRAS);

                    //================================================================================
                    imprimirDebugMotor(FRENTE, TRAS, "Virando à Direita");
                    //================================================================================

                } while ((millis() - tempo) <= TEMPO_CURVA); //Consome o tempo

                tempo = millis(); // Realimenta o tempo

                do
                {
                    movimentacao (FRENTE, PARADO);

                    //================================================================================
                    imprimirDebugMotor(FRENTE, PARADO, "Curva à Direita");
                    //================================================================================

                    lerSensorBorda(  QTD_SENS_OPON); //preenche o array de valores
                    detectarBorda(    codReacao);

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
                    movimentacao (TRAS, FRENTE);

                    //================================================================================
                    imprimirDebugMotor(TRAS, FRENTE, "Virando à Esquerda");
                    //================================================================================

                } while ((millis() - tempo) <= TEMPO_CURVA);

                tempo = millis();

                do
                {
                    movimentacao (PARADO, FRENTE);

                    //================================================================================
                    imprimirDebugMotor(PARADO, FRENTE, "Curva à Esquerda");
                    //================================================================================

                    lerSensorBorda( QTD_SENS_OPON); //preenche o array de valores
                    detectarBorda(    codReacao);


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
                movimentacao (FRENTE, FRENTE);

                //================================================================================
                imprimirDebugMotor(FRENTE, FRENTE, "Indo para Frente");
                //================================================================================
            } while ((millis() - tempo) <= TEMPO_MOV_LINEAR);

            tempo = 0;
            //parar();

        }

        lerSensorBorda(  QTD_SENS_OPON); //preenche o array de valores
        detectarBorda(    codReacao);

    } while (codigoReacao[0] != 0 || codigoReacao[1] != 0);
}
/*
   Função para preenchimento do array de valores dos sensores Sharp (ver oponente)
 */
void preenchimento()
{

    //  distance[0] = sensor0.getDistance();
    //  distance[1] = sensor1.getDistance();
    //  distance[2] = sensor2.getDistance();

    for (int i = 0; i < 3; i++)
    {
        switch (i) {
            case 0: distance[0] = sensor0.getDistance();
            case 1: distance[1] = sensor1.getDistance();
            case 2: distance[2] = sensor2.getDistance();
        }
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
    int erro[6] = {-45, -23, 0, 23, 45, 0};

    switch (combin)
    {
        case 0:
            return 0;
        case 2:
            imprimirDebugOpon ("MEIO", combin, erro[1]);
            return erro[2]; // TA RETORNANDO 0
        case 7:
            imprimirDebugOpon ("OS TRES", combin, erro[5]);
            return erro[5]; // TA RETORNANDO 0
        case 1:
            imprimirDebugOpon ("DIREITA", combin, erro[0]);
            return erro[0]; // TA RETORNANDO 45
        case 3:
            imprimirDebugOpon ("DIREITA E MEIO", combin, erro[2]);
            return erro[1]; // TA RETORNANDO 23
        case 4:
            imprimirDebugOpon ("DIREITA E MEIO", combin, erro[2]);
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
    }
}
/*
void correcao(int pi)
{
    int RIP_PID;

    RIP_PID = erro_pi();

    if (RIP_PID == 1) //DIREITO
    {
        //Serial.println("MOVIMENTO DIREITA -255,255");
        movimentacao(FRENTE, TRAS);
        imprimirDebugMotorCorrecao (FRENTE, TRAS, "Corrigindo aa Direita", RIP_PID);
    }
    else if (RIP_PID == 4 ) //ESQUERDA
    {
        movimentacao(TRAS, FRENTE);
        imprimirDebugMotorCorrecao (TRAS, FRENTE, "Corrigindo aa Esquerda", RIP_PID);
    }
    else if (RIP_PID == 2 || RIP_PID == 7) //MEIO E OS TRES
    {
        //Serial.println("MOVIMENTO FRENTE 255,255");
        movimentacao(FRENTE, FRENTE);
        imprimirDebugMotorCorrecao (FRENTE, FRENTE, "Corrigindo para a Frente", RIP_PID);
    }
}*/
int correcao(int distance[])
{

    static float somaErro = 0;
    static int tempo = 0;
    float erro;
    int pi;

    if ((erro = erro_pi(distance)) == 0.0)
        somaErro = 0;
    else
        somaErro += erro;

    //DEFINIR VALOR DAS CONSTANTES
    pi = (int)(KP * erro) + (KI * somaErro);
    tempo = millis();

    return pi;
}int correcao(int distance[])
{


    static float somaErro = 0;
    static int tempo = 0;
    float erro;
    int pi;

    if ((erro = erro_pi(distance)) == 0.0)
        somaErro = 0;
    else
        somaErro += erro;

    //DEFINIR VALOR DAS CONSTANTES
    pi = (int)(KP * erro) + (KI * somaErro);
    tempo = millis();

    return pi;
}
/*
   Função que recebe a "tupla" com código de reação e age em conformidade.
   A prioridade é não sair da arena, por isto, nesta função não será verificada
   a detecção de oponente.
 */

/*
   Função inicial que executa os movimento de busca e
   verifica os sensores de borda e oponente para reagir
   caso necessário.
 */
/*void procurar ()
  {
//Serial.println("Nova procura\n");
movimentacao (PARADO, PARADO); // com while() tava funcionando

lerSensorBorda(  QTD_SENS_OPON); //preenche o array de valores
lerSensorBorda(  QTD_SENS_OPON); //preenche o array de valores
detectarBorda(    codReacao);

if (codReacao[0] != 0 || codReacao[1] != 0)
{
reacaoBorda (codReacao);
}
else if (detectaOpon()) // while() tá funcionando!!!
{
int pi = 0;
pi = erro_pi();
correcao(pi);
}
}*/
void seguir()
{
    if (detectaOpon())
    {
        int pi = 0;
        pi = erro_pi();
        correcao(pi);
    }
    else // ESTRATEGIA AQUI!!!
    {
        movimentacao(PARADO, PARADO);
        Serial.print("Parado");
    } /////////////////////!!!
}

void procurar ()
{
    //Serial.println("Nova procura\n");
    //movimentacao (FRENTE, FRENTE); // com while() tava funcionando

    lerSensorBorda(  QTD_SENS_OPON); //preenche o array de valores
    lerSensorBorda(  QTD_SENS_OPON); //preenche o array de valores
    detectarBorda(codReacao);
    /*  do{
        Serial.println ("SEGUINDO");
        seguir();
        }while(codReacao[0] == 0 || codReacao[1] == 0);
        Serial.println ("INDENTIFICOU A BORDA");*/
    if (codReacao[0] != 0 || codReacao[1] != 0)
    {
        //      Serial.println ("INDENTIFICOU A BORDA");
        reacaoBorda (codReacao);
    }
    else
    {
        //      Serial.println ("SEGUINDO");
        seguir();
    }
}
/*
Estados:
1 -> FRENTE
0 -> PARADO
-1 -> TRAS
 */
/*void movimentacao(int estadoE, int estadoD)
  {
  motorEsq(estadoE);
  motorDir(estadoD);
  }*/
/*
   Com base no robô seguidor Marquinho
   1 -> FRENTE
   0 -> PARADO
   -1 -> TRAS
 */
void controle(int pi)
{
    motorDir(VBASE + pi);
    motorEsq(VBASE - pi);
}

void motorDir (int potencia)
{
    if (potencia > 0) {
        digitalWrite(MOTOR_D, LOW);
        analogWrite(MOTOR_D, abs(potencia));
    }
    else
    {
        digitalWrite (MOTOR_D, LOW);
        analogWrite (MOTOR_D, abs(potencia));
    }
}
void motorEsq(int potencia)
{
    if (potencia > 0)
    {
        digitalWrite(MOTOR_E, LOW);
        analogWrite(MOTOR_E, abs(potencia));
    }
    else
    {
        digitalWrite(MOTOR_E, LOW);
        analogWrite(MOTOR_E, abs(potencia));
    }
}


/*void motorEsq (int estado)
  {
  switch (estado)
  {
  case 1:
  digitalWrite(MOTOR_E1, HIGH);
  digitalWrite(MOTOR_E2, LOW);
  break;
  case 0:
  digitalWrite(MOTOR_E1, LOW);
  digitalWrite(MOTOR_E2, LOW);
  break;
  case -1:
  digitalWrite (MOTOR_E1, LOW);
  digitalWrite (MOTOR_E2, HIGH);
  break;
  }
  }

  void motorDir(int estado)
  {
  switch (estado) {
  case 1:
  digitalWrite(MOTOR_D1, HIGH);
  digitalWrite(MOTOR_D2, LOW);
  break;
  case 0:
  digitalWrite(MOTOR_D1, LOW);
  digitalWrite(MOTOR_D2, LOW);
  break;
  case -1:
  digitalWrite(MOTOR_D1, LOW);
  digitalWrite(MOTOR_D2, HIGH);
  break;
  }
  }*/
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
        for (unsigned char i = 0; i < QTD_SENS_OPON; i++)
        {
            Serial.print(SensoresF[i]);   //Valor máximo do teste: 2500 2500. Diminue com branco
            Serial.print('\t'); // tab para transformar em colunas
        }
        Serial.println();
        Serial.print("Sensores de tras     ");
        for (unsigned char i = 0; i < QTD_SENS_OPON; i++)
        {
            Serial.print(SensoresT[i]);   //Valor máximo do teste: 2500 2500. Diminue com branco
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
    //delay(500);
    pinMode(13, OUTPUT);// em uso pelo arraySensor
    digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode

    // Serial.println("Calibracao...");

    /* for (int i = 0; i < 100; i++) //A qtd de iterações pode ser modificado
       {
       qtrrc2.calibrate();
       qtrrc8.calibrate();
       }*/
    pinMode(A4, INPUT);
    pinMode(A5, INPUT);
    pinMode(A6, INPUT);
    pinMode(A7, INPUT);

    digitalWrite(13, LOW);

}
void loop() {
    tempoInicial = millis();
    //if (!fim) //Para o robô parar assim que o tempo de luta for excedido
    //{
    int pi = 0
        do
        {
            procurar ();
        } while ((millis() - tempoInicial) <= TEMPO_FIM);
    controle(pi);
    pi= correcao(distance);
    //   fim = true;
    // }else
    //{
    // parar();
    // }
}
