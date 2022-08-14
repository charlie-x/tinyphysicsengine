// #define FPS 50

#include "helper.h"

TPE_Vec3 environmentDistance(TPE_Vec3 p, TPE_Unit maxD)
{
  TPE_ENV_START( TPE_envHalfPlane(p,TPE_vec3(0,0,0),TPE_vec3(256,256,0)),p )
  TPE_ENV_NEXT( TPE_envHalfPlane(p,TPE_vec3(0,0,0),TPE_vec3(-256,256,-256)),p )
  TPE_ENV_NEXT( TPE_envHalfPlane(p,TPE_vec3(0,0,0),TPE_vec3(-256,256,256)),p )
  TPE_ENV_END
}

unsigned long timeMeasure = 0;

TPE_Vec3 bodyPositions[16];
TPE_Vec3 bodyOrientations[16];

int main(void)
{
  helper_init();

  tpe_world.environmentFunction = environmentDistance;

  s3l_scene.camera.transform.translation.y = 6000;
  s3l_scene.camera.transform.translation.z = -2000;
  s3l_scene.camera.transform.rotation.x = -70;

  for (int i = 0; i < 16; ++i)
  {
    switch (i % 5)
    {
      case 0: helper_addBox(800,800,800,400,700); break;
      case 1: helper_addTriangle(1100,200,600); break;
      case 2: helper_addBall(500,700); break;
      case 3: helper_addRect(800,800,400,800); break;
      case 4: helper_add2Line(900,200,600); break;
      default: break;
    }

    TPE_bodyMove(&tpe_world.bodies[tpe_world.bodyCount - 1],TPE_vec3((1 - (i % 4)) * 1200,8000,(2 - (i / 4)) * 1200));
  } 

  while (helper_running)
  {
    helper_frameStart();

    helper_cameraFreeMovement();

    if (helper_frame % 16 == 0)
    {
      helper_printCPU();

      if (sdl_keyboard[SDL_SCANCODE_L])
        for (int i = 0; i < tpe_world.bodyCount; ++i)
        {
          TPE_bodyActivate(&tpe_world.bodies[i]);
          TPE_bodyAccelerate(&tpe_world.bodies[i],
            TPE_vec3(0,(500 * 30) / FPS,0));
        }

      printf("world update (us): %lu\n",timeMeasure / 16);

      timeMeasure = 0;
    }

    unsigned long t1 = helper_getMicroSecs();

    TPE_worldStep(&tpe_world);

    timeMeasure += helper_getMicroSecs() - t1;

    for (int i = 0; i < tpe_world.bodyCount; ++i)
    {
      TPE_bodyApplyGravity(&tpe_world.bodies[i],(5 * 30) / FPS);

      TPE_Joint *joints = tpe_world.bodies[i].joints;
      TPE_Vec3 pos = TPE_bodyGetCenterOfMass(&tpe_world.bodies[i]);
      TPE_Vec3 right = TPE_vec3(512,0,0);
      TPE_Vec3 forw = TPE_vec3(0,0,512);

      if (i % 5 != 2 && i % 5 != 1) // don't mind the ugly code
      { 
        if (i % 5 != 4)
        {
          forw = TPE_vec3Minus(joints[2].position,joints[0].position);
          right = TPE_vec3Minus(joints[1].position,joints[0].position);
        }
        else
          forw = TPE_vec3Minus(joints[1].position,joints[0].position);
      }

      TPE_Vec3 orient = TPE_rotationFromVecs(forw,right);
    
      helper_set3dColor(100 + i * 5,16 - i,100 - i * 5); 

bodyPositions[i] =
TPE_vec3KeepWithinBox(
bodyPositions[i],
TPE_bodyGetCenterOfMass(&tpe_world.bodies[i]),
TPE_vec3(20,20,20)
);

bodyOrientations[i].x = 
(TPE_abs(bodyOrientations[i].x - orient.x) < 20) ?
((bodyOrientations[i].x + orient.x) / 2) : orient.x;

bodyOrientations[i].y = 
(TPE_abs(bodyOrientations[i].y - orient.y) < 20) ?
((bodyOrientations[i].y + orient.y) / 2) : orient.y;

bodyOrientations[i].z = 
(TPE_abs(bodyOrientations[i].z - orient.z) < 20) ?
((bodyOrientations[i].z + orient.z) / 2) : orient.z;

      switch (i % 5)
      {
        case 0: helper_draw3dBox(bodyPositions[i],TPE_vec3(1200,1200,1200),bodyOrientations[i]); break;
        case 1: helper_draw3dTriangle(bodyPositions[i],joints[1].position,joints[2].position); break;
        case 2: helper_draw3dSphere(bodyPositions[i],TPE_vec3(500,500,500),bodyOrientations[i]); break;
        case 3: helper_draw3dBox(bodyPositions[i],TPE_vec3(1200,400,1200),bodyOrientations[i]); break; 
        case 4: helper_draw3dBox(bodyPositions[i],TPE_vec3(200,200,1200),bodyOrientations[i]); break;
        default: break;
      }
    }

    helper_set3dColor(100,100,100); 
    helper_draw3dTriangle(TPE_vec3(0,0,0),TPE_vec3(-5000,5000,-10000),TPE_vec3(-5000,5000,10000));
    helper_set3dColor(140,140,140);
    helper_draw3dTriangle(TPE_vec3(0,0,0),TPE_vec3(-5000,5000,10000),TPE_vec3(5000,5000,0));
    helper_set3dColor(80,80,80);
    helper_draw3dTriangle(TPE_vec3(0,0,0),TPE_vec3(-5000,5000,-10000),TPE_vec3(5000,5000,0));

    if (helper_debugDrawOn)
      helper_debugDraw(1);

    helper_frameEnd();
  }

  helper_end();

  return 0;
}
