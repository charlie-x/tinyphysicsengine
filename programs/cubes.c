#include "helper.h"

#define ROOM_SIZE 7000
#define CUBE_SIZE 800

TPE_Unit sides[6] =
{
  0,0,
  1000,1000,
  -1000,2000
};

TPE_Vec3 environmentDistance(TPE_Vec3 p, TPE_Unit maxD)
{
  return TPE_envAABoxInside(p,TPE_vec3(0,ROOM_SIZE / 4,0),TPE_vec3(ROOM_SIZE,ROOM_SIZE / 2,ROOM_SIZE));
}

TPE_Vec3 cubeOrientations[6];
TPE_Vec3 cubePositions[6];

TPE_Joint ballJoints[4];
TPE_Connection ballConnections[3];

void updateOrientPos(int i)
{
  cubeOrientations[i] = TPE_bodyGetRotation(&tpe_world.bodies[i],0,2,1);
  cubePositions[i] = TPE_bodyGetCenterOfMass(&tpe_world.bodies[i]);
}

int main(void)
{
  helper_init();

  tpe_world.environmentFunction = environmentDistance;

  s3l_scene.camera.transform.translation.z -= ROOM_SIZE / 2;
  s3l_scene.camera.transform.translation.y += ROOM_SIZE / 3;
  s3l_scene.camera.transform.translation.x -= ROOM_SIZE / 4;
  s3l_scene.camera.transform.rotation.y = -1 * TPE_FRACTIONS_PER_UNIT / 16;

  for (int i = 0; i < 6; ++i)
{
    helper_addBox(CUBE_SIZE / 2,CUBE_SIZE / 2,CUBE_SIZE / 2,CUBE_SIZE / 4,10);
//helper_addCenterBox(CUBE_SIZE / 2,CUBE_SIZE / 2,CUBE_SIZE / 2,CUBE_SIZE / 4,50);

tpe_world.bodies[tpe_world.bodyCount - 1].elasticity = 200;
tpe_world.bodies[tpe_world.bodyCount - 1].friction = 0;

//if (i % 2)
//tpe_world.bodies[tpe_world.bodyCount - 1].flags |= TPE_BODY_FLAG_NONROTATING;
}

#define move(i,x,y) \
  TPE_bodyMoveBy(&tpe_world.bodies[i],TPE_vec3((CUBE_SIZE / 2 + 10) * x,10 + CUBE_SIZE / 2 + y * (CUBE_SIZE + 10),0));

  move(0,0,0)
  move(1,-2,0)
  move(2,2,0)
  move(3,-1,1)
  move(4,1,1)
  move(5,0,2)

#undef move
  
  for (int i = 0; i < 6; ++i)
  {
    updateOrientPos(i);
    TPE_bodyDeactivate(&tpe_bodies[i]);
  }

  ballJoints[0] = TPE_joint(TPE_vec3(0,ROOM_SIZE / 2,0),0);
  ballJoints[1] = TPE_joint(TPE_vec3(0,ROOM_SIZE / 2 - 100,-650),0);
  ballJoints[2] = TPE_joint(TPE_vec3(0,ROOM_SIZE / 2 - 300,-1300),0);
  ballJoints[3] = TPE_joint(TPE_vec3(0,ROOM_SIZE / 2 - 500,-1950),400);
  ballJoints[3].velocity[0] = 4;
  ballJoints[3].velocity[1] = -200;
  ballJoints[3].velocity[2] = -200;

  ballConnections[0].joint1 = 0; ballConnections[0].joint2 = 1;
  ballConnections[1].joint1 = 1; ballConnections[1].joint2 = 2;
  ballConnections[2].joint1 = 2; ballConnections[2].joint2 = 3;

  TPE_bodyInit(&tpe_world.bodies[6],ballJoints,4,ballConnections,3,1000);

tpe_world.bodies[6].friction = 0;
tpe_world.bodies[6].elasticity = 512;

  tpe_world.bodyCount++;
    
  while (helper_running)
  {
    helper_frameStart();

    helper_cameraFreeMovement();

    if (helper_frame % 16 == 0)
    {
      helper_printCPU();

      if (sdl_keyboard[SDL_SCANCODE_L])
        for (int i = 0; i < 6; ++i)
        {
          TPE_bodyActivate(&tpe_world.bodies[i]);
          TPE_bodyAccelerate(&tpe_world.bodies[i],
          TPE_vec3Plus(TPE_vec3(0,100,0),TPE_vec3Times(cubePositions[i],128)));
        }
    }

    TPE_jointPin(&tpe_world.bodies[6].joints[0],TPE_vec3(0,ROOM_SIZE / 2 - 10,0));

    tpe_world.bodies[6].deactivateCount = 0;

    TPE_worldStep(&tpe_world);

    helper_set3DColor(170,170,170);
    helper_draw3DBoxInside(TPE_vec3(0,ROOM_SIZE / 4,0),TPE_vec3(ROOM_SIZE,ROOM_SIZE / 2,ROOM_SIZE),TPE_vec3(0,0,0));

    helper_set3DColor(200,10,10);

    for (int i = 0; i < 6; ++i)
    {
      TPE_bodyApplyGravity(&tpe_world.bodies[i],7);

      if (!(tpe_world.bodies[i].flags & TPE_BODY_FLAG_DEACTIVATED))
        updateOrientPos(i);

      helper_draw3DBox(cubePositions[i],TPE_vec3(CUBE_SIZE,CUBE_SIZE,CUBE_SIZE),cubeOrientations[i]);
    }

    tpe_world.bodies[6].joints[3].velocity[1] -= 5;

    helper_draw3DSphere(tpe_world.bodies[6].joints[3].position,TPE_vec3(400,400,400),TPE_vec3(0,0,0)  );

    for (int i = 0; i < 3; ++i)
      helper_drawLine3D(
        tpe_world.bodies[6].joints[tpe_world.bodies[6].connections[i].joint1].position,
        tpe_world.bodies[6].joints[tpe_world.bodies[6].connections[i].joint2].position,
        255,0,0);

    if (helper_debugDrawOn)
      helper_debugDraw(1);

    helper_frameEnd();
  }

  helper_end();

  return 0;
}
