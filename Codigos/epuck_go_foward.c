#include <webots/robot.h>
#include <webots/motor.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28

int main(int argc, char **argv) {
 wb_robot_init();

 //Motor da roda esquerda
 WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
 //Motor da roda direita
 WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
 
 // Define a posição alvo como infinita
 wb_motor_set_position(left_motor, INFINITY);
 wb_motor_set_position(right_motor, INFINITY);
 // Define a velocidade com 80% de MAX_SPEED
 wb_motor_set_velocity(left_motor, 0.8 * MAX_SPEED);
 wb_motor_set_velocity(right_motor, 0.8 * MAX_SPEED);

 while (wb_robot_step(TIME_STEP) != -1);

 wb_robot_cleanup();

 return 0;
}