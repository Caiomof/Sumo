#define MOTOR_E1 5
#define MOTOR_E2 6
#define MOTOR_D1 9
#define MOTOR_D2 10

//======DEFINIÇÃO TEMPO DA LUTA=================
unsigned long tempoInicio = 0;
#define TEMPO_FIM 180000 //3 Minutos
//==============================================

#define QTD_SENS_OPON 3

#define QTD_SENS_BORDA_F 6
#define QTD_SENS_BORDA_T 2
int *valorSensorBordaF; //ALIMENTADA POR UMA FUNÇÃO DE LER SENSOR BORDA
int *valorSensorBordaT; //ALIMENTADA POR UMA FUNÇÃO DE LER SENSOR BORDA
int *valorSensorOpon;  //ALIMENTADA POR UMA FUNÇÃO DE LER OPON OU PODERIA SER RETIRADA AO USAR A STRUCT

int velocidades [12] = { -255, -230, -110, -50, 50, 110, 230, 255};
//===============================================================================
void detectarBorda (int * sensoresFrente, int * sensoresTras, int * retorno)
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


bool detectarOpon (int * valSensorOpon)
{
  for (int i = 0; i <= QTD_SENS_OPON; i++)
  {
    if (i == 1) {
      return true;
      //em seguida chama a função correção
    } else;
  }
  return false;
  //Continua na procura
}


//========Início das funções para o PID=======//
int correcao(bool binSensors[QTD_SENS_OPON], int sensoresAtivos)
{
  static float erroAnt = 0, somaErro = 0;
  static int tempo = 0;
  float erro;
  int pid;

  if ((erro = Erro(&erroAnt, binSensors, sensoresAtivos)) = 0)
    somaErro = 0;
  else
    somaErro += erro;
  //DEFINIR VALOR DAS CONSTANTES

  //pid=(int)(KP*erro)+(KI*somaErro)+(KD*erro-erroAnt);
  pid = 0;

  tempo = millis();
  erroAnt = erro;
  return pid;
}
float Erro(float *erroAnt, bool binSensors[QTD_SENS_OPON], int sensoresAtivos)
{
  return 0;
}


void aplicandoPID(int VbaseParaPID, int pid) {
  if (pid > 0)
  {
    motorDir(VbaseParaPID);
    motorEsq(VbaseParaPID - pid);
  }
  else if (pid < 0) {
    motorDir(VbaseParaPID + pid);
    motorEsq(VbaseParaPID);
  }
  else {
    motorDir(VbaseParaPID);
    motorEsq(VbaseParaPID);
  }
}


//========FIM das funções para o PID=======//

void procurar ()
{
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
      detectarBorda(valorSensorBordaF, valorSensorBordaT, codReacao);
      if (codReacao[0] != 0 && codReacao[1] != 0)
      {
        reacaoBorda (codReacao);
      }
      else if (detectarOpon(valorSensorOpon))
      {
        parar();
        reacaoOpon();


      }//A IMPLEMENTAR COMPLETAMENTE..........

    } while (millis() - tempo >= 3000);
    tempo = 0;


  } while ((millis() - tempoInicio) >= TEMPO_FIM);
  free(codReacao);
}

void reacaoOpon () //A IMPLEMENTAR COMPLETAMENTE..........
{
  int velBasePID = 0;//CRIADO APENAS PARA PREENCHER A FUNÇÃO. NA VERDADE SERIA UM #define
  int pid = 0;

  do
  {
    // pid = correcao (); //FALTAM OS PARÂMETROS
    aplicandoPID (velBasePID, pid);
  } while (detectarOpon == true);
}


void movimentacao(int potenciaE, int potenciaD) {
  motorEsq(potenciaE);
  motorDir(potenciaD);
}


void reacaoBorda (int * codigoReacao) //OS IF's PODERÃO SER SUBSTITUIDOS POR SWITCH CASE's
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


void setup() {
  // put your setup code here, to run once:

}
void loop() {
  // put your main code here, to run repeatedly:

}
