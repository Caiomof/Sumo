/************* PROGRAMAÇÃO DESENVOLVIDA PARA A WINTER CHALLENGE 2018***********
*****************PELA EQUIPE DE ROBÓTICA CARRANCA******************************
*******************************************************************************/

//Supõe-se que o carro vira 90º em 1s 
#include <SharpIR.h>

#define MOTOR_E 5
#define MOTOR_D 6

#define BOTAO 13

//DEFINE PARA SAIDA DOS MOTORES
#define SAIDA_MAX 255
#define SAIDA_MIN -255

//====ARRAY PARA AS PORTAS DOS SENSORES===
const uint8_t PIN_SENSOR_OPONENTE [3] = {A0, A1, A2}; //Sensor Sharp - uint8_t = 'unsigned char'

const uint8_t PIN_SENSOR_BORDA[] = {};//Pode-se colocar nas primeiras posições o 'array' e nas duas últimas os dois pequenos 
//--------------------------------------------------------

#define QTD_SENS_OPON 3
#define QTD_SENS_BORDA 6
#define LIMIAR_VER_OPON 25 //distância que se considera ver o oponente 25cm

#define velocidadePadrao 150 //Suposição

#define DELAY 1000
#define DEBUG 1

//Array que armazena a distância que cada sensor detecta ===Esse array pode ser um retorno da função 'lerSensor'
//int valSensorOponente [3] = {0, 0, 0};
int angulosPossiveis [12] = {30, 45, 60, 90, 180, 360, -30, -45, -60, -90, -180, -360};


//Novo tipo para retorno das funções de leitura de sensores
typedef struct leituraSensores
{
    byte sensoresAtivos[QTD_SENS_OPON]; // um array de 1's e 0's OU (um char com 'EC'=Esquedo e central; 'ECD'= Esquerdo, Direito e Central)
    float distancias[QTD_SENS_OPON];
} LeituraSensor;

//======DEFINIÇÃO TEMPO DE COMBATE=================
unsigned long tempoInicio=0;
#define TEMPO_FIM 180000 //3 Minutos

void setup() 
{
  pinMode (BOTAO, INPUT);
  pinMode (MOTOR_E, OUTPUT);
  pinMode (MOTOR_D, OUTPUT);

  delay(3000);
}

//====FUNÇÃO PROCURAR
void procurar ()
{
  movimentoLinear(velocidadePadrao);
  delay (2000);//Só para exemplo. este pause seria verificações de sensores em um dado tempo
  int angRand = random (0,sizeof(angulosPossiveis)/sizeof(angulosPossiveis[0]));
  movimentoAngular (angRand, false);
  movimentoLinear(velocidadePadrao);
/*
A função movimentoLinear inicia e, depois, a função movimento angular é chamada
recebendo valores contidos no array 'angulosPossiveis'. Esses valores serão escolhidos
aleatoriamente por uma função randômica.
O tempo em qe o robô se manterá em movimento linear também poderá se aleatório e, nas
funções de movimento linear e angular, podem ser chamadas as funções de verificação dos sensores.
*/

i}

//=======FUNÇÃO PARA SENSORES DE BORDA======
LeituraSensor lerSensorBorda(uint8_t pinSensoresBorda[QTD_SENS_BORDA]){}


//====FUNÇÃO PARA LER SENSORES DE OPONENTE=====
/*QUESTÃO
Esta função deve apenas ler, converter e retornar o valor dos sensores um por um? Ou
Obter o valor dos sensores, convertê-los e retornar as distâncias em um array? (VERIFICAR A POSSIBILIDADE DO USO DA FUNÇÃO SharpIR)
*/
LeituraSensor lerSensorOpon (uint8_t pinSensoresOpon[QTD_SENS_OPON], int tamanhoArray)
{
  LeituraSensor leitura;
  //for (int i = 0; i< sizeof(pinSensoresOpon); i++)
  for (int i = 0; i< tamanhoArray; i++)
  {
    SharpIR visaoOpon (GP2YA41SK0F, pinSensoresOpon[i]);
    leitura.distancias[i] = visaoOpon.getDistance();
    (leitura.sensoresAtivos[i] = (leitura.distancias[i] <= LIMIAR_VER_OPON) ? 1 : 0);
  }
     return leitura;
 }

//========Início das funções para o PID=======//
int correcao(bool binSensors[QTD_SENS_OPON],int sensoresAtivos)
{
    static float erroAnt=0, somaErro=0;
    static int tempo=0;
    float erro;
    int pid;

    if((erro = Erro(&erroAnt, binSensors, sensoresAtivos))=0)
        somaErro=0;
    else
        somaErro+= erro;
//DEFINIR VALOR DAS CONSTANTES
    pid=(int)(KP*erro)+(KI*somaErro)+(KD*erro-erroAnt);
    tempo = millis();
    erroAnt = erro;
    return pid;
}
float Erro(float *erroAnt, bool binSensors[QTD_SENS_OPON], int sensoresAtivos) 
    {
        return 0;
    }
//========FIM das funções para o PID=======//

//===FUNÇÃO PARA VIRAR O ROBÔ EM GRAUS===//
/*Uma diferença é que 'positivo' e 'negativo' 
 * tem como referença o ciclo trigonométrico 
 * (abscissa: 180 -- 0)
*/
void movimentoAngular (int grau, boolean girar)
{
  int duracao = abs((grau/90)*1000);
  unsigned long init = millis();
  if (girar==false){
    if (grau<0)
        {
            motorDir(velocidadePadrao*(grau/(grau)));
            motorEsq(velocidadePadrao*(grau/(-grau)));
                do{  
        //restrições: chama funções que verificam sensores e, dependendo do retorno destas, chama a(s) função de PID/ataque ou de desvio de borda.
                    }while ((millis()-init) <= duracao);  
      parar();
    
    /*Neste caso pode ser melhor usar delay(duracao), porque as duas opções
    paralizarão o fluxo do programa.
    Uma opção diferente seria definir uma variável global para ser verificada
    em relação ao tempo de duração. Assim, o código iniciaria a ação dos motores,
    mas continuaria fazendo as outras verificações e, em alguns pontos do código,
    seria feita a checagem do tempo limite (de parada) da tal movimentação. 
    */
    }
    else
    { 
      motorDir(velocidadePadrao*(grau/(grau)));
      motorEsq(velocidadePadrao*(grau/(-grau)));
      do{  
        //restrições
      }while ((millis()-init) <= duracao);  
      parar();
    }
  //Se ângulo < ou > ZERO e girar = true, ele deve girar indefinidamente em algum dos sentidos
  }else  
  {
    motorDir(velocidadePadrao*(grau/(grau)));
    motorEsq(velocidadePadrao*(grau/(-grau)));
  }
}

//===FUNÇÃO PARA MOVIMENTAÇÃO FRENTE-TRÁS===//  
void movimentoLinear(int potencia){
  motorDir(potencia);
  motorEsq(potencia);
}


void parar ()
{
  motorDir(0);
  motorEsq(0);
}


/*Função que fará uma movimentação angular durante 1/2 segundo, 
  para que seja registrado manualmente o ângulo percorrido nesse
  tempo com uma velocidade padrão
*/  
void getAnguloTempo()
{
      movimentoAngular(1,true);
      delay(500);
      parar();
}

//Com base no robô seguidor Marquinho
void motorDir (int potencia)
{
  if (potencia > 0){
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
  if(potencia > 0)
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
void loop() 
{
  boolean botao = digitalRead (BOTAO);
  //verBotao(botao);
  bool binSensors[QTD_SENS_OPON]; 
  movimentoLinear (1);
  movimentoAngular(30, true);
  
  
}



void olhos (){}
void sinalLed(int tipo){}
void estrategia(){}
void borda(){}





/*A seguir, código para uso de botão de partida
e sinais de LED.
*/

//======DEFINIÇÃO SINAIS DE LED=================
#define LED_PRELUTA 1 //sugestão: definir sinal de ações indicadas por led
#define LED_INTERRUPCAO 2
#define LED_FIMLUTA 3

void verBotao(int botao)
{
  if (botao == false)
  {
    sinalLed(LED_PRELUTA);
  }
  else if (botao == true && tempoInicio==0)
  {
    tempoInicio=millis();
    estrategia();
  }
  else if (botao == true && (millis() - tempoInicio)>=1000){
    //para e pisca led. Isto caso seja necessário interromper a luta antes do fim
    parar();
    sinalLed(LED_INTERRUPCAO);
    tempoInicio==0;
  }
  else if ((millis()-tempoInicio)>=TEMPO_FIM)
  {
    //fim da luta
    parar();
    sinalLed(LED_FIMLUTA);
  }
}

void imprimirDebug (int velocPadao) {
  if (DEBUG)
  Serial.print(" Velocidade Padrao : ");
  Serial.print(velocPadao);
  delay(DELAY);
}

