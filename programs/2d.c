#define TPE_RESHAPE_ITERATIONS 5

#define DEBUG_DRAW_DIVIDE 16

#include "helper.h"

#define ROOM_SIZE 7000
#define CUBE_SIZE 800

TPE_Vec3 environmentDistance(TPE_Vec3 p, TPE_Unit maxD)
{
  return TPE_envAABoxInside(p,TPE_vec3(0,ROOM_SIZE / 4,0),TPE_vec3(ROOM_SIZE,ROOM_SIZE / 2,ROOM_SIZE));
}

int main(void)
{
  helper_init();

helper_debugDrawOn = 1;

  tpe_world.environmentFunction = environmentDistance;

  s3l_scene.camera.transform.translation.z -= ROOM_SIZE / 2;

s3l_scene.camera.focalLength = 0; // set orthographic projection

for (int i = 0; i < 4; ++i)
{
  helper_addCenterRectFull(500,500,100,100);

  TPE_bodyRotateByAxis(&tpe_world.bodies[i],TPE_vec3(TPE_FRACTIONS_PER_UNIT / 4,0,0));

tpe_world.bodies[i].joints[4].sizeDivided = 300 / TPE_JOINT_SIZE_MULTIPLIER;

tpe_world.bodies[i].friction = 400;

  TPE_bodyMove(&tpe_world.bodies[i],TPE_vec3(-1000 + i * 800,2000,0));
}


    
  while (helper_running)
  {
    helper_frameStart();

    //helper_cameraFreeMovement();

      
    if (sdl_keyboard[SDL_SCANCODE_LEFT])
      TPE_bodyAccelerate(&tpe_world.bodies[0],TPE_vec3(-10,0,0));
    else if (sdl_keyboard[SDL_SCANCODE_RIGHT])
      TPE_bodyAccelerate(&tpe_world.bodies[0],TPE_vec3(10,0,0));

    TPE_worldStep(&tpe_world);

    helper_set3dColor(100,100,100);

    helper_set3dColor(200,10,10);

    for (int i = 0; i < tpe_world.bodyCount; ++i)
      TPE_bodyApplyGravity(&tpe_world.bodies[i],5);

for (int i = 0; i < tpe_world.bodyCount; ++i)
{
  for (int j = 0; j < tpe_world.bodies[i].jointCount; ++j)
  {
    tpe_world.bodies[i].joints[j].position.z = 0;
    tpe_world.bodies[i].joints[j].velocity[2] = 0;
  }
}

    if (helper_debugDrawOn)
      helper_debugDraw();

    helper_frameEnd();
  }

  helper_end();

  return 0;
}
