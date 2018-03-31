/************* PROGRAMAÇÃO DESENVOLVIDA PARA A WINTER CHALLENGE 2018***********
*****************PELA EQUIPE DE ROBÓTICA CARRANCA******************************
*******************************************************************************/

//Supõe-se que o carro vira 90º em 1s 

#define MOTOR_E1 5
#define MOTOR_E2 6
#define MOTOR_D1 9
#define MOTOR_D2 10

#define visaoDir A0
#define visaoFre A1
#define visaoEsq A2

#define velocidadePadrao 150 //Suposição

#define BOTAO 13

#define DELAY 1000
#define DEBUG 1

int angulosPossiveis [12] = {30, 45, 60, 90, 180, 360, -30, -45, -60, -90, -180, -360};


//====ARRAY PARA AS PORTAS DOS SENSORES===
const byte PIN_SENSOR_OPONENTE [3] = {A0, A1, A2}; //Sensor Sharp
//--------------------------------------------------------

//======DEFINIÇÃO TEMPO DE COMBATE=================
unsigned long tempoInicio=0;
#define TEMPO_FIM 180000 //3 Minutos

void setup() 
{
  pinMode (BOTAO, INPUT);
  pinMode (MOTOR_E1, OUTPUT);
  pinMode (MOTOR_E2, OUTPUT);
  pinMode (MOTOR_D1, OUTPUT);
  pinMode (MOTOR_D2, OUTPUT);

  delay(3000);
}

//===FUNÇÃO PARA VIRAR O ROBÔ EM GRAUS===//
void movimentoAngular (int grau, boolean girar){
  int duracao = abs((grau/90)*1000);
  unsigned long init = millis();
  if (grau<0)
  {
    analogWrite (MOTOR_E1, velocidadePadrao);
    analogWrite (MOTOR_E2, 0);
    analogWrite (MOTOR_D1, 0);
    analogWrite (MOTOR_D2, velocidadePadrao);
    
    if (girar == false){
      do{  
        //restrições
      }while ((millis()-init) <= duracao);  
      parar();
    }
    
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
    analogWrite (MOTOR_E1, 0);
    analogWrite (MOTOR_E2, velocidadePadrao);
    analogWrite (MOTOR_D1, velocidadePadrao);
    analogWrite (MOTOR_D2, 0);
   if (girar == false){
      do{  
        //restrições
      }while ((millis()-init) <= duracao);  
      parar();
    }
  }
}

//===FUNÇÃO PARA MOVIMENTAÇÃO FRENTE-TRÁS===//  
void movimentoLinear(int potencia){
  if (potencia >0){
    analogWrite (MOTOR_E1, velocidadePadrao);
    analogWrite (MOTOR_E2, 0);
    analogWrite (MOTOR_D1, velocidadePadrao);
    analogWrite (MOTOR_D2, 0);
  }else{
    analogWrite (MOTOR_E1, 0);
    analogWrite (MOTOR_E2, velocidadePadrao);
    analogWrite (MOTOR_D1, 0);
    analogWrite (MOTOR_D2, velocidadePadrao);
  }
}


void parar ()
{
      analogWrite (MOTOR_E1, 0);
      analogWrite (MOTOR_E2, 0);
      analogWrite (MOTOR_D1, 0);
      analogWrite (MOTOR_D2, 0);
}


/*Função que fará uma movimentação angular durante 1/2 segundo, 
  para que seja registrado manualmente o ângulo percorrido nesse
  tempo com uma velocidade padrão
*/  
void getAnguloTempo()
{
      analogWrite (MOTOR_E1, 0);
      analogWrite (MOTOR_E2, velocidadePadrao);
      analogWrite (MOTOR_D1, velocidadePadrao);
      analogWrite (MOTOR_D2, 0);
      delay(500);
      parar();
}

void loop() 
{
  boolean botao = digitalRead (BOTAO);
  //verBotao(botao);
 
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

