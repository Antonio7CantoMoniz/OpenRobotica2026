#include "BluetoothSerial.h"
BluetoothSerial SerialBT;


//Pinos Sensores
const int pino_S1 = 34;
const int pino_S3 = 35;
const int pino_S4 = 36;
const int pino_S5 = 39;

//Pinos LED
const int pino_LED = 2;

//Pinos LDR
const int pino_LDR = 27;
int limite_LDR = 800; //1000 ou 800 // Ajustar: Valor alto = luz forte (branco do LED)

//Motor Esquerdo (A)
const int DIR_A = 19;
const int PWM_A = 25;
const int TRAVAR_A = 13;
//Motor Direito (B)
const int DIR_B = 18;
const int PWM_B = 23;
const int TRAVAR_B = 12;

// --- Configurações de Velocidade e Rampa ---
float v_atual = 40;// Velocidade que vai subindo gradualmente
const float rampa_arranque = 2.3;// Quanto sobe por ciclo (arranque suave)
int velocidade_maxima = 95;
int corte_esquerda = 6; //Corte de tensao na roda da esquerda
unsigned long tempo_arranque = 0;

// --- VARIÁVEIS DE CALIBRAÇÃO ---
int s1_min = 4095, s1_max = 0, s1_media = 0;
int s3_min = 4095, s3_max = 0, s3_media = 0;
int s4_min = 4095, s4_max = 0, s4_media = 0;
int s5_min = 4095, s5_max = 0, s5_media = 0;

bool relat_impresso = false; // Garante que só imprime uma vez ao parar
unsigned volatile int cont = 0;
bool a_tocar_na_linha = false;
unsigned long tempo_ultima_linha = 0;

unsigned long tempo_ultimo_cruzamento = 0;
int limite1 = 0;//
int limite3 = 0;//
int limite4 = 0;//
int limite5 = 0;// 

float Kp = 6.0; //6.0 ou 3.2 ou 2.8
float Ki = 0.0; // 0.05
float Kd = 7.0; //6.5 ou 2.8 ou 2.0

float P = 0, I = 0, D = 0;
int PID_valor = 0;
float erro = 0;
float ultimo_erro = 0;

// Sistema fail-safe
unsigned long tempo_perdido = 0;
bool modo_seg = false;

void setup() {
  SerialBT.begin("ESP-etada");

  pinMode(DIR_A, OUTPUT); pinMode(PWM_A, OUTPUT); pinMode(TRAVAR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT); pinMode(PWM_B, OUTPUT); pinMode(TRAVAR_B, OUTPUT);

  pinMode(pino_LDR, INPUT);

  // Desativar os travoes das rodas
  digitalWrite(TRAVAR_A, LOW);
  digitalWrite(TRAVAR_B, LOW);

  //BLOCO DE ARRANQUE PELO LDR
  while (analogRead(pino_LDR) > limite_LDR) {

    PararMotores();

    // Lê os sensores sem parar enquanto espera
    int l1 = analogRead(pino_S1);
    int l3 = analogRead(pino_S3);
    int l4 = analogRead(pino_S4);
    int l5 = analogRead(pino_S5);

    // Regista o valor mais escuro e o mais claro de cada um
    if (l1 < s1_min) s1_min = l1; if (l1 > s1_max) s1_max = l1;
    if (l3 < s3_min) s3_min = l3; if (l3 > s3_max) s3_max = l3;
    if (l4 < s4_min) s4_min = l4; if (l4 > s4_max) s4_max = l4;
    if (l5 < s5_min) s5_min = l5; if (l5 > s5_max) s5_max = l5;
    //SerialBT.print("LDR: "); SerialBT.println(analogRead(pino_LDR));
    relat_impresso = false;
  }

  // Calcula o limite perfeito (exatamente no meio do branco e do preto lidos)
  limite1 = (s1_min + s1_max) / 2;
  limite3 = (s3_min + s3_max) / 2;
  limite4 = (s4_min + s4_max) / 2;
  limite5 = (s5_min + s5_max) / 2;
  SerialBT.printf("LIMITES AUTO: L1:%d | L3:%d | L4:%d | L5:%d\n", limite1, limite3, limite4, limite5);
  SerialBT.println("PARTIDAAAAA!");

  I = 0; 
  erro = 0; 
  ultimo_erro = 0;
  
  tempo_arranque = millis();

}

void loop() {

  // Calcular o erro
  erro = CalcularErro();

  // Passou a 2a linha
  if (cont == 1){
    PararMotores2();
    SerialBT.println("FIM DA CORRIDA!!");
    while(true){
      delay(100);
    }
  }

  P = erro;
  I = I + erro;

  //A Trela: Limita a memória para o carro não se despistar
  /*if (I > 50) I = 50;
  if (I < -50) I = -50;
  // O Reset: Se estiver perfeitamente na linha, apaga a memória
  if (erro == 0) I = 0;*/

  D = erro - ultimo_erro;

  PID_valor = ( Kp * P ) + ( Ki * I ) + ( Kd * D );
  ultimo_erro = erro;

  // SISTEMA FAIL-SAFE
  if (erro >= 4.5 || erro <= -4.5){
    if (tempo_perdido == 0){
      tempo_perdido = millis();
    }
    else if (millis() - tempo_perdido > 1000){
      modo_seg = true;
    }
  } else {
    tempo_perdido = 0;
    modo_seg = false;
  }

  // Aplicar velocidade aos motores
  if (modo_seg){
    PararMotores();
  } else {
    AplicarMotores();
  }  
}

// ==========================================
// Funcoes para processamento dos dados
// ==========================================
float CalcularErro(){

  int s1 = analogRead(pino_S1);
  int s3 = analogRead(pino_S3);
  int s4 = analogRead(pino_S4);
  int s5 = analogRead(pino_S5);

  if (s1 < limite1 && s5 < limite5) {
    if (a_tocar_na_linha == false && (millis() - tempo_ultima_linha > 1000) && (millis() - tempo_arranque > 4000)) {
      cont = cont + 1;
      a_tocar_na_linha = true;
      tempo_ultima_linha = millis();
      SerialBT.print("LINHA DETETADA! Contagem: ");
      SerialBT.println(cont);
    }
  } else {
    a_tocar_na_linha = false;
  }

  if (s1 >= limite1 && s3 >= limite3 && s4 >= limite4 && s5 >= limite5) {
    if (ultimo_erro > 0) return 5.0;  // Ativa o Fail-Safe e vira tudo à direita
    if (ultimo_erro < 0) return -5.0; // Ativa o Fail-Safe e vira tudo à esquerda
    return 0.0;
  }
  /*if (soma_forcas == 0) {
    if (ultimo_erro > 0) return 5.0;  
    if (ultimo_erro < 0) return -5.0;
    return 0.0;
  }*/
  float novo_erro = 0.0;
  if (s3 < limite3) novo_erro -= 2.0;
  if (s5 < limite5) novo_erro += 2.0;
  if (s4 < limite4) {
      if (novo_erro < 0) novo_erro = -1.0;
      else if (novo_erro > 0) novo_erro = 1.0;
      else novo_erro = 0.0;
  }
  return novo_erro;
}

void AplicarMotores(){

  relat_impresso = false;

  // Aceleração rápida
  if (v_atual < velocidade_maxima){
     v_atual += rampa_arranque;
  }

  int v_base = (int)v_atual;
  int v_esq = (v_base - corte_esquerda) + PID_valor;
  int v_dir = v_base - PID_valor;

  v_esq = constrain(v_esq, 0, 100);
  v_dir = constrain(v_dir, 0, 100);

  int pwm_esq_real = map(v_esq, 0, 100, 0, 255);
  int pwm_dir_real = map(v_dir, 0, 100, 0, 255);

  digitalWrite(DIR_A, HIGH);
  analogWrite(PWM_A, pwm_esq_real);

  digitalWrite(DIR_B, HIGH);
  analogWrite(PWM_B, pwm_dir_real);
}

void PararMotores(){
  v_atual = 40;
  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);
}

void PararMotores2(){
  PararMotores();
  s3_media = (s3_min + s3_max)/2;
  s4_media = (s4_min + s4_max)/2;
  s5_media = (s5_min + s5_max)/2;

  // --- PRINT DO RELATÓRIO DE CALIBRAÇÃO ---
  if (!relat_impresso) {
    SerialBT.printf("\n==== RELATÓRIO DE SENSORES ===\n");
    SerialBT.printf("S3: %d | S4: %d | S5: %d\n", s3_media, s4_media, s5_media);
    SerialBT.print("LDR: "); SerialBT.println(analogRead(pino_LDR));
    SerialBT.println("=============================\n");
    relat_impresso = true;
  }
}