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
#define DEBUG_MOTOR_BORDA 0
#define DEBUG_MOTOR_PI 0

//===Sentidos/Estados dos motores=======
#define FRENTE 1
#define PARADO 0
#define TRAS -1

//========== SHARP IR=================
#define QTD_SENS_OPON 3
#define LIMITE 24
SharpIR sensor0(GP2YA41SK0F, A1);
SharpIR sensor1(GP2YA41SK0F, A2);
SharpIR sensor2(GP2YA41SK0F, A3);
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
    int erro[6] = {1, 2, 3, 4, 6, 7};

    switch (combin)
    {
        case 0:
            return 0;
        case 1:
            imprimirDebugOpon ("DIREITA", combin, erro[0]);
            return erro[0]; // TA RETORNANDO 45
        case 2:]
               imprimirDebugOpon ("MEIO", combin, erro[1]);
               return erro[1]; // TA RETORNANDO 0
        case 3:
               imprimirDebugOpon ("DIREITA MEIO", combin, erro[2]);
               return erro[2]; // TA RETORNANDO 23
        case 4:
               imprimirDebugOpon ("DIREITA MEIO", combin, erro[2]);
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

void correcao(int pi)
{
    int RIP_PID;

    RIP_PID = erro_pi();

    if(RIP_PID == 1 || RIP_PID == 3) //DIREITO, DIREITA E MEIO
    {
        movimentacao(FRENTE, FRENTE);
    }
    else if(RIP_PID == 4 || RIP_PID == 6) //ESQUERDA, ESQUERDA E MEIO
    {
        movimentacao(TRAS, TRAS);
    }
    else if(RIP_PID == 2 || RIP_PID == 7) //MEIO E OS TRES
    {
        movimentacao(TRAS,FRENTE);
    }
}


void procurar ()
{
    //Serial.println("Nova procura\n");

    movimentacao(PARADO,PARADO);

    while(detectaOpon())
    {
        int pi = 0;
        pi = erro_pi();
        correcao(pi);

        //Serial.println("Fim da procura\n");
    }
}

/*
 * Estados:
 *  1 -> FRENTE
 *  0 -> PARADO
 * -1 -> TRAS
 */
void movimentacao(int estadoE, int estadoD)
{
    motorEsq(estadoE);
    motorDir(estadoD);
}

/*
 * Com base no robô seguidor Marquinho
 *  1 -> FRENTE
 *  0 -> PARADO
 * -1 -> TRAS
 */

void motorDir (int estado)
{
    switch (estado)
    {
        case 1:
            digitalWrite(MOTOR_D1, HIGH);
            digitalWrite(MOTOR_D2, LOW);
            break;
        case 0:
            digitalWrite(MOTOR_D1, LOW);
            digitalWrite(MOTOR_D2, LOW);
            break;
        case -1:
            digitalWrite (MOTOR_D1, LOW);
            digitalWrite (MOTOR_D2, HIGH);
            break;
    }
}

void motorEsq(int estado)
{
    switch (estado){
        case 1:
            digitalWrite(MOTOR_E1, HIGH);
            digitalWrite(MOTOR_E2, LOW);
            break;
        case 0:
            digitalWrite(MOTOR_E1, LOW);
            digitalWrite(MOTOR_E2, LOW);
            break;
        case -1:
            digitalWrite(MOTOR_E1, LOW);
            digitalWrite(MOTOR_E2, HIGH);
            break;
    }
}

void setup()
{
    Serial.begin(9600);
    //delay(500);
    pinMode(13, OUTPUT);// em uso pelo arraySensor
    digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode

    Serial.println("Calibracao...");

    digitalWrite(13, LOW);

}
void loop() 
{
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
