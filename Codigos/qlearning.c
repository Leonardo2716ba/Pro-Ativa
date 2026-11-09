#include <webots/robot.h>
#include <webots/motor.h> 
#include <webots/distance_sensor.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <stdio.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28

#define NUM_ESTADOS 8  
#define NUM_ACOES   3   

#define ALPHA 0.3        
#define GAMMA 0.9         
#define EPSILON_INICIAL 0.3
#define EPSILON_MIN 0.05
#define EPSILON_DECAY 0.999

#define LIMIAR_OBSTACULO 80.0
#define LIMIAR_COLISAO   150.0  

double Q[NUM_ESTADOS][NUM_ACOES];

int calcular_estado(double ps_values[8], bool *colidiu) {
  bool frente = ps_values[0] >= LIMIAR_OBSTACULO || ps_values[7] >= LIMIAR_OBSTACULO;
  bool direita = ps_values[1] >= LIMIAR_OBSTACULO || ps_values[2] >= LIMIAR_OBSTACULO;
  bool esquerda = ps_values[5] >= LIMIAR_OBSTACULO || ps_values[6] >= LIMIAR_OBSTACULO;

  *colidiu = false;
  for (int i = 0; i < 8; i++) {
    if (ps_values[i] > LIMIAR_COLISAO) {
      *colidiu = true;
      break;
    }
  }

  int estado = (frente ? 1 : 0) + (direita ? 2 : 0) + (esquerda ? 4 : 0);
  return estado;
}

int escolher_acao(int estado, double epsilon) {
  double r = (double) rand() / RAND_MAX;
  if (r < epsilon) {
    return rand() % NUM_ACOES;
  } else {
    int melhor_acao = 0;
    for (int a = 1; a < NUM_ACOES; a++) {
      if (Q[estado][a] > Q[estado][melhor_acao]) {
        melhor_acao = a;
      }
    }
    return melhor_acao;
  }
}

double max_Q(int estado) {
  double max_val = Q[estado][0];
  for (int a = 1; a < NUM_ACOES; a++) {
    if (Q[estado][a] > max_val) max_val = Q[estado][a];
  }
  return max_val;
}

void aplicar_acao(int acao, double *left_speed, double *right_speed) {
  switch (acao) {
    case 0: // frente
      *left_speed = 0.5 * MAX_SPEED;
      *right_speed = 0.5 * MAX_SPEED;
      break;
    case 1: // virar esquerda
      *left_speed = -0.3 * MAX_SPEED;
      *right_speed = 0.5 * MAX_SPEED;
      break;
    case 2: // virar direita
      *left_speed = 0.5 * MAX_SPEED;
      *right_speed = -0.3 * MAX_SPEED;
      break;
  }
}

double calcular_recompensa(bool colidiu, int acao) {
  if (colidiu) return -10.0;
  if (acao == 0) return 1.0;    // Recompensa por avançar
  return -0.1;     // Pequeno custo por girar
}

int main(int argc, char **argv) {
 wb_robot_init();
 srand(time(NULL));

 int i;
 WbDeviceTag ps[8];
 
 char ps_names[8][4] = { "ps0", "ps1", "ps2", "ps3", 
                         "ps4", "ps5", "ps6", "ps7" };
 
 for (i = 0; i < 8; i++){
   ps[i] = wb_robot_get_device(ps_names[i]);
   wb_distance_sensor_enable(ps[i], TIME_STEP);
 } 
  
 WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
 WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
 wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
 
 //Zerar a tabela
 for(int i =0; i < NUM_ESTADOS; i++)
   for(int j = 0; j < NUM_ACOES; j++ )
     Q[i][j] = 0.0;
     
  double epsilon = EPSILON_INICIAL;

  // Estado inicial (lê sensores uma vez antes do loop)
  double ps_values[8];
  for (i = 0; i < 8; i++)
    ps_values[i] = wb_distance_sensor_get_value(ps[i]);
  bool colidiu;
  int estado_atual = calcular_estado(ps_values, &colidiu);

  int passo = 0;
  double left_speed = 0.0;
  double right_speed = 0.0;
 
 //LOOP PRINCIPAL
  while (wb_robot_step(TIME_STEP) != -1) {
    // 1. Escolhe e aplica a acao
    int acao = escolher_acao(estado_atual, epsilon);
    aplicar_acao(acao, &left_speed, &right_speed);
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
    
    // 2. Le sensores e calcula o novo estado
    for (i = 0; i < 8; i++) ps_values[i] = wb_distance_sensor_get_value(ps[i]);
    int novo_estado = calcular_estado(ps_values, &colidiu);
    
    // 3. Calcula a recompensa
    double recompensa = calcular_recompensa(colidiu, acao);
    
    // 4. Atualiza a Q-table
    Q[estado_atual][acao] += ALPHA * (recompensa + GAMMA * max_Q(novo_estado)- Q[estado_atual][acao]);
    
    // 5. Avanca o estado e decai epsilon
     estado_atual = novo_estado;
    if (epsilon > EPSILON_MIN) epsilon *= EPSILON_DECAY;
    
    passo++;
    if (passo % 100 == 0) {
      printf("Passo %d | epsilon=%.3f | estado=%d | acao=%d | recompensa=%.2f\n",
             passo, epsilon, estado_atual, acao, recompensa);
    }  
  }

 wb_robot_cleanup();

 return 0;
}