#include <webots/robot.h>
#include <webots/motor.h> 
#include <webots/distance_sensor.h>

#define TIME_STEP 64

#define MAX_SPEED 6.28

int main(int argc, char **argv) {
 wb_robot_init();
  
 int i;
 WbDeviceTag ps[8];
 
 char ps_names[8][4] = { "ps0", "ps1", "ps2", "ps3", 
                         "ps4", "ps5", "ps6", "ps7" };
 
 for (i = 0; i < 8; i++){
   ps[i] = wb_robot_get_device(ps_names[i]);
   wb_distance_sensor_enable(ps[i], TIME_STEP);
 } 
  
 //Motor da roda esquerda
 WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
 //Motor da roda direita
 WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
 
 wb_motor_set_position(left_motor, INFINITY);
 wb_motor_set_position(right_motor, INFINITY);
 
 wb_motor_set_velocity(left_motor, 0);
 wb_motor_set_velocity(right_motor, 0);
 
 //LOOP PRINCIPAL
 while (wb_robot_step(TIME_STEP) != -1){
   double left_speed = 0.5 * MAX_SPEED;
   double right_speed = 0.5 * MAX_SPEED;
   
   // ------- LENDO OS SENSORES
   double ps_values[8];
   for(i = 0; i < 8 ; i++)
     ps_values[i] = wb_distance_sensor_get_value(ps[i]);
     
   // ------- PROCESSAR COMPORTAMENTO
   bool right_obstacle =  ps_values[0] > 80 ||
                          ps_values[1] > 80 ||
                          ps_values[2] > 80;
   
   bool left_obstacle =  ps_values[5] > 80 ||
                         ps_values[6] > 80 ||
                         ps_values[7] > 80;     
   
   if (right_obstacle){              //Virar para esquerda
     right_speed = 0.5 * MAX_SPEED;
     left_speed  = - 0.5 * MAX_SPEED;
   } else if (left_obstacle){         //Virar para a direira
     right_speed = - 0.5 * MAX_SPEED;
     left_speed  =  0.5 * MAX_SPEED;
   }
   
   // ---------- ENVIAR PARA OS ATUADORES
   wb_motor_set_velocity(right_motor, right_speed);
   wb_motor_set_velocity(left_motor, left_speed);
   
 };

 wb_robot_cleanup();

 return 0;
}