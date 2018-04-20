#include <QTRSensors.h>
#define NUM_SAMPLES_PER_SENSOR 4
#define QTD_SENS_BORDA 6
#define COR_BORDA 600
#define EMITTER_PIN 2
QTRSensorsAnalog sensoresBorda((unsigned char[]) {
  5, 4, 3, 2, 1, 0
}, QTD_SENS_BORDA, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);

void setup() {
  sensoresBorda.calibrate(QTR_EMITTERS_ON);
}

struct leituraSensores
{
  byte* sensoresAtivos;     // um array de 1's e 0's OU (um char com 'EC'=Esquedo e central; 'ECD'= Esquerdo, Direito e Central)
  unsigned int* distancias;
};
typedef leituraSensores LeituraSensor;

struct leituraSensores lerSensorBorda(uint8_t pinSensoresBorda[QTD_SENS_BORDA], int tamanhoArray)
{
  struct leituraSensores leitura;
  leitura.sensoresAtivos = (byte *) malloc (tamanhoArray);
  leitura.distancias = (unsigned int *) malloc (tamanhoArray * sizeof (unsigned int)); // Seria necessário usar um free ("leitura") antes de usar esta função novamente? Ou, numa nova chamada, ela usa o mesmo espaço alocado na primeira chamada?
  sensoresBorda.readLine(leitura.distancias);

  for (int i = 0; i < tamanhoArray; i++)
  {
    leitura.sensoresAtivos[i] = ((leitura.distancias[i] >= COR_BORDA) ? 1 : 0);
  }
  return leitura;
}

void reagirBorda () {}

void loop() {

}
