#define TPE_RESHAPE_ITERATIONS 5

#define DEBUG_DRAW_DIVIDE 8

#include "helper.h"

#define ROOM_W 5100
#define ROOM_H ((RES_Y * ROOM_W) / RES_X)
#define SQUARE_SIZE 500

TPE_Vec3 environmentDistance(TPE_Vec3 p, TPE_Unit maxD)
{
  return TPE_envAABoxInside(p,TPE_vec3(0,0,0),TPE_vec3(ROOM_W,ROOM_H,ROOM_W));
}

inactiveCount = 0;

int main(void)
{
  helper_init();

helper_debugDrawOn = 1;

  tpe_world.environmentFunction = environmentDistance;

  s3l_scene.camera.transform.translation.z -= ROOM_W / 2;

s3l_scene.camera.focalLength = 0; // set orthographic projection

  for (int i = 0; i < 4; ++i)
  {

if (i != 2)
{
    helper_addCenterRectFull(SQUARE_SIZE,SQUARE_SIZE,100,100);
    TPE_bodyRotateByAxis(&tpe_world.bodies[i],TPE_vec3(TPE_FRACTIONS_PER_UNIT / 4,0,0));
    tpe_world.bodies[i].joints[4].sizeDivided = 300 / TPE_JOINT_SIZE_MULTIPLIER;
}
else
    helper_addBall(600,100);


    tpe_world.bodies[i].friction = 400;
   
 
tpe_world.bodies[i].elasticity = 100;

    TPE_bodyMove(&tpe_world.bodies[i],TPE_vec3(-1000 + i * 800,ROOM_H / 4,0));
  }


    
  while (helper_running)
  {
    helper_frameStart();

    if (sdl_keyboard[SDL_SCANCODE_LEFT])
      TPE_bodyAccelerate(&tpe_world.bodies[0],TPE_vec3(-10,0,0));
    else if (sdl_keyboard[SDL_SCANCODE_RIGHT])
      TPE_bodyAccelerate(&tpe_world.bodies[0],TPE_vec3(10,0,0));
    
    if (sdl_keyboard[SDL_SCANCODE_UP])
      TPE_bodyAccelerate(&tpe_world.bodies[0],TPE_vec3(0,10,0));

    TPE_worldStep(&tpe_world);

    helper_set3dColor(100,100,100);

    helper_set3dColor(200,10,10);

    for (int i = 0; i < tpe_world.bodyCount; ++i)
      TPE_bodyApplyGravity(&tpe_world.bodies[i],5);


TPE_Unit speed, speedMax = 0;

int anyActive = 0;

for (int i = 0; i < tpe_world.bodyCount; ++i)
{
  for (int j = 0; j < tpe_world.bodies[i].jointCount; ++j)
  {
    tpe_world.bodies[i].joints[j].position.z = 0;
    tpe_world.bodies[i].joints[j].velocity[2] = 0;
  }

if (!(tpe_world.bodies[i].flags & TPE_BODY_FLAG_DEACTIVATED))
  anyActive = 1;


speed = TPE_bodyGetAverageSpeed(&tpe_world.bodies[i]);

if (speed > speedMax)
  speedMax = speed;



}


if (anyActive && speedMax < 50)
  inactiveCount++;
else
  inactiveCount = 0;

if (inactiveCount > 100)
{
  TPE_worldDeactivateAll(&tpe_world);
  inactiveCount = 0;
}




  //  if (helper_debugDrawOn)
      helper_debugDraw(0);

    helper_frameEnd();
  }

  helper_end();

  return 0;
}
