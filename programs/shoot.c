#include "helper.h"

#define CATAPULT_HEIGHT 1300
#define CATAPULT_WIDTH 700
#define BALL_RADIUS 200

TPE_Unit sides[6] =
{
  0,0,
  1000,1000,
  -1000,2000
};

uint8_t collisionCallback(uint16_t b1, uint16_t j1, uint16_t b2, uint16_t j2,
  TPE_Vec3 p)
{
  // disable collision between the catapult and ball
  return !((b1 == 0 && b2 == 1) || (b2 == 1 && b1 == 0));
}

TPE_Vec3 environmentDistance(TPE_Vec3 p, TPE_Unit maxD)
{
  return TPE_envGround(p,0);
}

TPE_Joint joints[3];
TPE_Connection connections[2];
TPE_Body catapult;

uint8_t released = 0;

int main(void)
{
  helper_init();

  tpe_world.collisionCallback = collisionCallback;

  tpe_world.environmentFunction = environmentDistance;

  s3l_scene.camera.transform.translation.x = -2032;
  s3l_scene.camera.transform.translation.y = 1838;
  s3l_scene.camera.transform.translation.z = -1705;
  s3l_scene.camera.transform.rotation.x = -5;
  s3l_scene.camera.transform.rotation.y = -102;

  joints[0] = TPE_joint(TPE_vec3(0,CATAPULT_HEIGHT,CATAPULT_WIDTH / 2),10);
  joints[1] = TPE_joint(TPE_vec3(0,CATAPULT_HEIGHT,0),10);
  joints[2] = TPE_joint(TPE_vec3(0,CATAPULT_HEIGHT,-1 * CATAPULT_WIDTH / 2),10);
 
  connections[0].joint1 = 0;
  connections[0].joint2 = 1;
  connections[1].joint1 = 1;
  connections[1].joint2 = 2;

  TPE_bodyInit(&tpe_bodies[0],joints,3,connections,2,10);

  joints[1].position.x -= 800;
  joints[1].position.y -= 200;

  tpe_bodies[0].flags |= TPE_BODY_FLAG_SOFT;
  tpe_bodies[0].flags |= TPE_BODY_FLAG_SIMPLE_CONN; // this makes the spring faster

  tpe_world.bodyCount++;

  helper_addBall(BALL_RADIUS,1000);

helper_addCenterBox(500,500,800,200,300);
TPE_bodyMoveBy(&helper_lastBody,TPE_vec3(4000,600,-600));
TPE_bodyDeactivate(&helper_lastBody);

helper_addCenterBox(500,500,800,200,300);
TPE_bodyMoveBy(&helper_lastBody,TPE_vec3(4000,600,600));
TPE_bodyDeactivate(&helper_lastBody);

helper_addCenterRect(500,1800,300,300);
TPE_bodyMoveBy(&helper_lastBody,TPE_vec3(4000,1400,0));
TPE_bodyDeactivate(&helper_lastBody);

helper_addCenterBox(600,600,900,200,300);
TPE_bodyMoveBy(&helper_lastBody,TPE_vec3(4000,2200,0));
TPE_bodyDeactivate(&helper_lastBody);


  while (helper_running)
  {
    helper_frameStart();

    helper_cameraFreeMovement();

    if (released)
    {
      TPE_worldStep(&tpe_world);
      TPE_jointPin(&joints[0],TPE_vec3(0,CATAPULT_HEIGHT,CATAPULT_WIDTH / 2));
      TPE_jointPin(&joints[2],TPE_vec3(0,CATAPULT_HEIGHT,-1 * CATAPULT_WIDTH / 2));

      for (int i = 1; i < tpe_world.bodyCount; ++i)
        TPE_bodyApplyGravity(&tpe_world.bodies[i],6);

      TPE_bodyMultiplyNetSpeed(&tpe_world.bodies[0],500);
    }

    if (!released || tpe_world.bodies[1].joints[0].position.x < 0)
    {
      tpe_world.bodies[1].joints[0].position = joints[1].position;

      for (int i = 0; i < 3; ++i)
        tpe_world.bodies[1].joints[0].velocity[i] = joints[1].velocity[i];
    }

    if (sdl_keyboard[SDL_SCANCODE_SPACE])
      released = 1;

    if (!released)
    {
#define OFFSET 20
      if (sdl_keyboard[SDL_SCANCODE_W])
        joints[1].position.y += OFFSET;
      else if (sdl_keyboard[SDL_SCANCODE_S] && joints[1].position.y > 0)
        joints[1].position.y -= OFFSET;

      if (sdl_keyboard[SDL_SCANCODE_F])
        joints[1].position.x -= OFFSET;
      else if (sdl_keyboard[SDL_SCANCODE_G] && joints[1].position.x < 5)
        joints[1].position.x += OFFSET;

      if (sdl_keyboard[SDL_SCANCODE_D])
        joints[1].position.z -= OFFSET;
      else if (sdl_keyboard[SDL_SCANCODE_A])
        joints[1].position.z += OFFSET;
    }

    if (helper_debugDrawOn)
      helper_debugDraw(1);

    helper_set3DColor(200,200,255);

    helper_draw3DSphere(tpe_world.bodies[1].joints[0].position,TPE_vec3(BALL_RADIUS,BALL_RADIUS,BALL_RADIUS),TPE_vec3(0,0,0));
    
helper_set3DColor(200,255,200);

helper_draw3DBox(TPE_bodyGetCenterOfMass(&tpe_world.bodies[2]),TPE_vec3(700,1000,700),
  TPE_bodyGetRotation(&tpe_world.bodies[2],0,1,2));

helper_draw3DBox(TPE_bodyGetCenterOfMass(&tpe_world.bodies[3]),TPE_vec3(700,1000,700),
  TPE_bodyGetRotation(&tpe_world.bodies[3],0,1,2));

helper_draw3DBox(TPE_bodyGetCenterOfMass(&tpe_world.bodies[4]),TPE_vec3(2000,500,700),
  TPE_bodyGetRotation(&tpe_world.bodies[4],0,1,2));

helper_draw3DBox(TPE_bodyGetCenterOfMass(&tpe_world.bodies[5]),TPE_vec3(800,1100,800),
  TPE_bodyGetRotation(&tpe_world.bodies[5],0,1,2));

    helper_drawLine3D(TPE_vec3(0,0,0),TPE_vec3(0,CATAPULT_HEIGHT / 2,0),255,0,0 );
    helper_drawLine3D(TPE_vec3(0,CATAPULT_HEIGHT / 2,0),TPE_vec3(0,CATAPULT_HEIGHT,-1 * CATAPULT_WIDTH / 2),255,0,0);
    helper_drawLine3D(TPE_vec3(0,CATAPULT_HEIGHT / 2,0),TPE_vec3(0,CATAPULT_HEIGHT,CATAPULT_WIDTH / 2),255,0,0);
    helper_drawLine3D(TPE_vec3(0,CATAPULT_HEIGHT,-1 * CATAPULT_WIDTH / 2),joints[1].position,0,255,0);
    helper_drawLine3D(TPE_vec3(0,CATAPULT_HEIGHT,CATAPULT_WIDTH / 2),joints[1].position,0,255,0);

    helper_frameEnd();
  }

  helper_end();

  return 0;
}
