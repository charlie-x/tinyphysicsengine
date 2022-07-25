#include "helper.h"

#define ROOM_SIZE 10000
#define CUBE_SIZE 800

TPE_Vec3 environmentDistance(TPE_Vec3 p, TPE_Unit maxD)
{
  TPE_ENV_START( TPE_envAABoxInside(p,TPE_vec3(0,ROOM_SIZE / 4,0),TPE_vec3(ROOM_SIZE,ROOM_SIZE / 2,ROOM_SIZE)),p )
  TPE_ENV_NEXT( TPE_envAABox(p,TPE_vec3(4000,160,4000),TPE_vec3(1000,160,1000)),p )
  TPE_ENV_NEXT( TPE_envAABox(p,TPE_vec3(4000,80,2500),TPE_vec3(1000,80,500)),p )
  TPE_ENV_NEXT( TPE_envAABox(p,TPE_vec3(-1000,270,4500),TPE_vec3(4000,270,250)),p )
  TPE_ENV_END
}

/*
320
320 + 160
*/

int jumpCountdown = 0;
TPE_Unit rotation = 0;
int onGroundCount = 0;

TPE_Vec3 directionVec;

void updateDirection(void)
{
  directionVec.x = TPE_sin(rotation);
  directionVec.z = TPE_cos(rotation);
  directionVec.y = 0;
}

uint8_t collisionCallback(uint16_t b1, uint16_t j1, uint16_t b2, uint16_t j2,
  TPE_Vec3 p)
{
  if (b1 == 0 && b1 == b2 && j1 == 0 && 
    p.y < tpe_world.bodies[0].joints[0].position.y -50)
    onGroundCount = 2;
}

int main(void)
{
  helper_init();

  helper_debugDrawOn = 1;

  updateDirection();

  tpe_world.environmentFunction = environmentDistance;

tpe_world.collisionCallback = collisionCallback;

  s3l_scene.camera.transform.translation.z -= ROOM_SIZE / 2;
  s3l_scene.camera.transform.translation.y += ROOM_SIZE / 3;
  s3l_scene.camera.transform.translation.x -= ROOM_SIZE / 4;
  s3l_scene.camera.transform.rotation.y = -1 * TPE_FRACTIONS_PER_UNIT / 16;

  helper_add2Line(400,300,400);
  
TPE_bodyMove(&tpe_world.bodies[0],TPE_vec3(0,1000,0));
TPE_bodyRotateByAxis(&tpe_world.bodies[0],TPE_vec3(0,0,TPE_FRACTIONS_PER_UNIT / 4));

tpe_world.bodies[0].elasticity = 0;   
tpe_world.bodies[0].friction = 0;   
  tpe_world.bodies[0].flags |= TPE_BODY_FLAG_NONROTATING;

helper_addBall(1000,100);
TPE_bodyMove(&tpe_world.bodies[1],TPE_vec3(-1000,1000,0));

tpe_world.bodies[1].elasticity = 512;
 
  while (helper_running)
  {
    if (onGroundCount > 0)
      onGroundCount--;
      
    if (jumpCountdown > 0)
      jumpCountdown--;

    helper_frameStart();

    s3l_scene.camera.transform.translation.x = tpe_world.bodies[0].joints[0].position.x;

    s3l_scene.camera.transform.translation.y = TPE_keepInRange(
      s3l_scene.camera.transform.translation.y,
      tpe_world.bodies[0].joints[1].position.y,
      tpe_world.bodies[0].joints[1].position.y + 10);

    TPE_bodyMultiplyNetSpeed(&tpe_world.bodies[0],onGroundCount ? 300 : 505);

    s3l_scene.camera.transform.translation.z = tpe_world.bodies[0].joints[0].position.z;
    s3l_scene.camera.transform.rotation.y = -1 * rotation;

    TPE_worldStep(&tpe_world);

    TPE_bodyActivate(&tpe_world.bodies[0]);

    if (onGroundCount)
    {
      if (sdl_keyboard[SDL_SCANCODE_SPACE] && jumpCountdown == 0 && onGroundCount)
      {
        tpe_world.bodies[0].joints[0].velocity[1] = 80;
        jumpCountdown = 8;
        onGroundCount = 0;
      }

#define AAA 16
      if (sdl_keyboard[SDL_SCANCODE_UP] || sdl_keyboard[SDL_SCANCODE_W])
      {
        tpe_world.bodies[0].joints[0].velocity[0] += directionVec.x / AAA;
        tpe_world.bodies[0].joints[0].velocity[2] += directionVec.z / AAA;
      }
      else if (sdl_keyboard[SDL_SCANCODE_DOWN] || sdl_keyboard[SDL_SCANCODE_S])
      {
        tpe_world.bodies[0].joints[0].velocity[0] -= directionVec.x / AAA;
        tpe_world.bodies[0].joints[0].velocity[2] -= directionVec.z / AAA;
      }

      if (sdl_keyboard[SDL_SCANCODE_A])
      {
        tpe_world.bodies[0].joints[0].velocity[2] += directionVec.x / AAA;
        tpe_world.bodies[0].joints[0].velocity[0] -= directionVec.z / AAA;
      }
      else if (sdl_keyboard[SDL_SCANCODE_D])
      {
        tpe_world.bodies[0].joints[0].velocity[2] -= directionVec.x / AAA;
        tpe_world.bodies[0].joints[0].velocity[0] += directionVec.z / AAA;
      }
#undef AAA
    }

    if (sdl_keyboard[SDL_SCANCODE_LEFT])
      rotation -= 8;
    else if (sdl_keyboard[SDL_SCANCODE_RIGHT])
      rotation += 8;

    updateDirection();

    helper_set3dColor(100,100,100);
    helper_draw3dCubeInside(TPE_vec3(0,ROOM_SIZE / 4,0),TPE_vec3(ROOM_SIZE,ROOM_SIZE / 2,ROOM_SIZE),TPE_vec3(0,0,0));
    helper_draw3dCube(TPE_vec3(4000,160,4000),TPE_vec3(2000,320,2000),TPE_vec3(0,0,0));
    helper_draw3dCube(TPE_vec3(4000,80,2500),TPE_vec3(2000,160,1000),TPE_vec3(0,0,0));

    helper_draw3dCube(TPE_vec3(-1000,270,4500),TPE_vec3(8000,540,500),TPE_vec3(0,0,0));
    

helper_draw3dSphere(
tpe_world.bodies[1].joints[0].position
,TPE_vec3(1000,1000,1000),TPE_vec3(0,0,0)  );

    helper_set3dColor(200,10,10);

for (int i = 0; i < tpe_world.bodyCount; ++i)
    TPE_bodyApplyGravity(&tpe_world.bodies[i],5);

    if (helper_debugDrawOn)
      helper_debugDraw();

    helper_frameEnd();
  }

  helper_end();

  return 0;
}
