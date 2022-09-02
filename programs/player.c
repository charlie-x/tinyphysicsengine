//#define SCALE_3D_RENDERING 1

#define S3L_NEAR_CROSS_STRATEGY 2
#define S3L_PERSPECTIVE_CORRECTION 2

#include "helper.h"

#define ROOM_SIZE 10000
#define CUBE_SIZE 800

TPE_Unit elevatorHeight;

TPE_Vec3 environmentDistance(TPE_Vec3 p, TPE_Unit maxD)
{
  TPE_ENV_START( TPE_envAABoxInside(p,TPE_vec3(0,ROOM_SIZE / 4,0),TPE_vec3(ROOM_SIZE,ROOM_SIZE / 2,ROOM_SIZE)),p )
  TPE_ENV_NEXT( TPE_envAABox(p,TPE_vec3(4000,160,4000),TPE_vec3(1000,160,1000)),p )
  TPE_ENV_NEXT( TPE_envAABox(p,TPE_vec3(4000,80,2500),TPE_vec3(1000,80,500)),p )
  TPE_ENV_NEXT( TPE_envAABox(p,TPE_vec3(-1000,270,4500),TPE_vec3(4000,270,250)),p )
  TPE_ENV_NEXT( TPE_envAABox(p,TPE_vec3(-4000,elevatorHeight,0),TPE_vec3(1000,elevatorHeight,1000)),p )
  TPE_ENV_NEXT( TPE_envHalfPlane(p,TPE_vec3(0,0,-2000),TPE_vec3(0,255,255)),p )
  TPE_ENV_NEXT( TPE_envInfiniteCylinder(p,TPE_vec3(2000,0,-1100),TPE_vec3(0,255,0),400),p )
  TPE_ENV_END
}

int jumpCountdown = 0, onGround = 0;
TPE_Unit playerRotation = 0, groundDist;
TPE_Vec3 ballRot, ballPreviousPos, playerDirectionVec;
TPE_Body *playerBody = 0;

void updateDirection(void) // updates player direction vector
{
  playerDirectionVec.x = TPE_sin(playerRotation);
  playerDirectionVec.z = TPE_cos(playerRotation);
  playerDirectionVec.y = 0;
}

int main(void)
{
  helper_init();

  updateDirection();

  ballRot = TPE_vec3(0,0,0);

  tpe_world.environmentFunction = environmentDistance;

  helper_add2Line(400,300,400);

  playerBody = &(tpe_world.bodies[0]);

  TPE_bodyMoveBy(&tpe_world.bodies[0],TPE_vec3(0,1000,0));
  TPE_bodyRotateByAxis(&tpe_world.bodies[0],TPE_vec3(0,0,TPE_FRACTIONS_PER_UNIT / 4));
  playerBody->elasticity = 0;
  playerBody->friction = 0;   
  playerBody->flags |= TPE_BODY_FLAG_NONROTATING;
  groundDist = TPE_JOINT_SIZE(playerBody->joints[0]) + 30;

  helper_addBall(1000,100);
  TPE_bodyMoveBy(&tpe_world.bodies[1],TPE_vec3(-1000,1000,0));
  tpe_world.bodies[1].elasticity = 400;
  tpe_world.bodies[1].friction = 100;

  ballPreviousPos = tpe_world.bodies[1].joints[0].position;

  helper_addCenterRect(600,600,400,50);
  TPE_bodyMoveBy(&tpe_world.bodies[2],TPE_vec3(-3000,1000,2000));
  tpe_world.bodies[2].elasticity = 100;
  tpe_world.bodies[2].friction = 50;
 
  while (helper_running)
  {
    helper_frameStart();

    TPE_worldStep(&tpe_world);

    if (jumpCountdown > 0)
      jumpCountdown--;

    /* Check whether on ground. This can be done in several ways, e.g. by
    checking recent collisions. We'll do it by casting a downwards ray: */

    onGround = (playerBody->flags & TPE_BODY_FLAG_DEACTIVATED) ||
      TPE_DISTANCE( playerBody->joints[0].position,
      TPE_castEnvironmentRay(playerBody->joints[0].position,
      TPE_vec3(0,-1 * TPE_FRACTIONS_PER_UNIT,0),tpe_world.environmentFunction, 
      128,512,512)) <= groundDist;

    elevatorHeight = TPE_sin(helper_frame * 4) + TPE_FRACTIONS_PER_UNIT;

    s3l_scene.camera.transform.translation.x = playerBody->joints[0].position.x;
    s3l_scene.camera.transform.translation.z = playerBody->joints[0].position.z;
    s3l_scene.camera.transform.translation.y = TPE_keepInRange(
      s3l_scene.camera.transform.translation.y,
      playerBody->joints[1].position.y,
      playerBody->joints[1].position.y + 10);

    TPE_bodyMultiplyNetSpeed(&tpe_world.bodies[0],onGround ? 300 : 505);

    s3l_scene.camera.transform.rotation.y = -1 * playerRotation;

    TPE_Vec3 ballRoll = TPE_fakeSphereRotation(ballPreviousPos,
      tpe_world.bodies[1].joints[0].position,1000);

    ballRot = TPE_rotationRotateByAxis(ballRot,ballRoll);

    ballPreviousPos = tpe_world.bodies[1].joints[0].position;

    for (int i = 0; i < tpe_world.bodyCount; ++i)
      TPE_bodyApplyGravity(&tpe_world.bodies[i],5);

    if (onGround)
    {
      TPE_bodyActivate(&tpe_world.bodies[0]);

      if (sdl_keyboard[SDL_SCANCODE_SPACE] && jumpCountdown == 0)
      {
        playerBody->joints[0].velocity[1] = 90;
        jumpCountdown = 8;
      }

#define D 16 // just some vector divisor to make the speed slower
      if (sdl_keyboard[SDL_SCANCODE_UP] || sdl_keyboard[SDL_SCANCODE_W])
      {
        playerBody->joints[0].velocity[0] += playerDirectionVec.x / D;
        playerBody->joints[0].velocity[2] += playerDirectionVec.z / D;
      }
      else if (sdl_keyboard[SDL_SCANCODE_DOWN] || sdl_keyboard[SDL_SCANCODE_S])
      {
        playerBody->joints[0].velocity[0] -= playerDirectionVec.x / D;
        playerBody->joints[0].velocity[2] -= playerDirectionVec.z / D;
      }

      if (sdl_keyboard[SDL_SCANCODE_A])
      {
        playerBody->joints[0].velocity[2] += playerDirectionVec.x / D;
        playerBody->joints[0].velocity[0] -= playerDirectionVec.z / D;
      }
      else if (sdl_keyboard[SDL_SCANCODE_D])
      {
        playerBody->joints[0].velocity[2] -= playerDirectionVec.x / D;
        playerBody->joints[0].velocity[0] += playerDirectionVec.z / D;
      }
#undef D
    }
    
    updateDirection();

    if (sdl_keyboard[SDL_SCANCODE_LEFT])
      playerRotation -= 8;
    else if (sdl_keyboard[SDL_SCANCODE_RIGHT])
      playerRotation += 8;

    if (helper_frame % 64 == 0)
      helper_printCPU();

    // draw the 3D environment

    helper_set3dColor(180,180,180);
    helper_draw3dBoxInside(TPE_vec3(0,ROOM_SIZE / 4,0),TPE_vec3(ROOM_SIZE,ROOM_SIZE / 2,ROOM_SIZE),TPE_vec3(0,0,0));
    helper_set3dColor(100,200,180);
    helper_draw3dBox(TPE_vec3(4000,160,4000),TPE_vec3(2000,320,2000),TPE_vec3(0,0,0));
    helper_draw3dBox(TPE_vec3(4000,80,2500),TPE_vec3(2000,160,1000),TPE_vec3(0,0,0));
    helper_draw3dBox(TPE_vec3(-1000,270,4500),TPE_vec3(8000,540,500),TPE_vec3(0,0,0));
    helper_draw3dBox(TPE_vec3(-4000,elevatorHeight,0),TPE_vec3(2000,2 * elevatorHeight,2000),TPE_vec3(0,0,0));
    helper_draw3dCylinder(TPE_vec3(2000,5000,-1100),TPE_vec3(400,10000,400),TPE_vec3(0,0,0));

    helper_draw3dPlane(TPE_vec3(0,1500,-3500),TPE_vec3(10000,512,4000),TPE_vec3(-64,0,0));
    
    helper_set3dColor(200,50,0);

    helper_draw3dBox(TPE_bodyGetCenterOfMass(&tpe_world.bodies[2]),
      TPE_vec3(1200,800,1200),TPE_bodyGetRotation(&tpe_world.bodies[2],0,2,1));

    helper_draw3dSphere(tpe_world.bodies[1].joints[0].position,
      TPE_vec3(1000,1000,1000),ballRot);

    if (helper_debugDrawOn)
      helper_debugDraw(1);

    helper_frameEnd();
  }

  helper_end();

  return 0;
}
