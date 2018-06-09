#include <SharpIR.h>

//====ARRAY PARA AS PORTAS DOS SENSORES===
#define QTD_SENS_OPON 3

#define velocidadePadrao 150 //Suposição

#define MOTOR_D 5
#define MOTOR_E 6

#define VBASE 100

#define KP 70
#define KI 0.000001

SharpIR sensor0(GP2YA41SK0F, A3);
SharpIR sensor1(GP2YA41SK0F, A2);
SharpIR sensor2(GP2YA41SK0F, A1);

//Novo tipo para retorno das funções de leitura de sensores
int distance[QTD_SENS_OPON] = {0};
float base = 2.0;

void setup()
{
    Serial.begin(9600); //Enable the serial comunication
}

void prenchimento()
{
    distance[0] = sensor0.getDistance();
    distance[1] = sensor1.getDistance();
    distance[2] = sensor2.getDistance();

    for (int i = 0; i < 3; i++)
    {
        if (distance[i] < 12)
        {
            //Serial.print("Distancia "); Serial.print(i); Serial.print(" : ");
            distance[i] = (int)(pow(base, i) + 0.5);
        }
        else
            distance[i] = 0;
    }
    //Serial.println();
}

//========Início das funções para o PID=======//
float erro_pi(int distance[])
{
    int ret = 0;

    int erro[6] = { -45, 0, -23, 45, 23, 0};
    delay(500);

    for (int i = 0; i < 3; i++)
    {
        ret += distance[i];
    }

    switch (ret)
    {
        case 0:
            Serial.println("Funcao procura"); //Print the value to the serial monitor
            Serial.println(ret); //Print the value to the serial monitor
            Serial.println(); //Print the value to the serial monitor
            break;
        case 1:
            Serial.println("DIREITA"); //Print the value to the serial monitor
            Serial.println(ret); //Print the value to the serial monitor
            Serial.println(); //Print the value to the serial monitor
            return erro[0]; // TA RETORNANDO -45
        case 2:
            Serial.println("MEIO"); //Print the value to the serial monitor
            Serial.println(ret); //Print the value to the serial monitor
            Serial.println(); //Print the value to the serial monitor
            return erro[1]; // TA RETORNANDO 0
        case 3:
            Serial.println("DIREITA MEIO"); //Print the value to the serial monitor
            Serial.println(ret); //Print the value to the serial monitor
            Serial.println(); //Print the value to the serial monitor
            return erro[2]; // TA RETORNANDO -23
        case 4:
            Serial.println("ESQUERDA"); //Print the value to the serial monitor
            Serial.println(ret); //Print the value to the serial monitor
            Serial.println(); //Print the value to the serial monitor
            return erro[3]; // TA RETORNANDO 45
        case 5:
            /*
               SERIA O CASO DE SÓ O DA DIREITA E O DA ESQUERDA ESTAREM ACIONADOS
               E O DO MEIO NAO
               ISSO TEORICAMENTE É IMPOSSIVEL
             */
            break;
        case 6:
            Serial.println("ESQUERDA E MEIO"); //Print the value to the serial monitor
            Serial.println(ret); //Print the value to the serial monitor
            Serial.println(); //Print the value to the serial monitor
            return erro[4]; // TA RETORNANDO 23
        case 7:
            Serial.println("OS TRES"); //Print the value to the serial monitor
            Serial.println(ret); //Print the value to the serial monitor
            Serial.println(); //Print the value to the serial monitor
            return erro[5]; // TA RETORNANDO 0
    }
}

int correcao(int distance[])
{
    /////////////////////////////////////////

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

void loop()
{
    int pi = 0;

    prenchimento();
    pi = correcao(distance);
    controle(pi);
}
