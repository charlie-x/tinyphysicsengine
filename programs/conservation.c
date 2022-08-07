/**
  Example that fakes the momentum/energy conservation. We simply keep the record
  of total speed in the system (as an approximation of momentum/energy/whatever)
  and if it changes too much, we multiply them accordingly to get them back :)
*/

#include "helper.h"

#define SPHERE_R 3500

TPE_Vec3 environmentDistance(TPE_Vec3 p, TPE_Unit maxD)
{
TPE_ENV_START( TPE_envSphereInside(p,TPE_vec3(0,0,0),SPHERE_R),p )
TPE_ENV_END
}

uint8_t debugDrawOn = 1;

unsigned long timeMeasure = 0;

int main(void)
{
  helper_init();

  tpe_world.environmentFunction = environmentDistance;

  s3l_scene.camera.transform.translation.z = -1 * SPHERE_R - 1000;


  for (int i = 0; i < 4; ++i)
  {
    switch (i)
    {
      case 0: helper_addBox(800,800,800,400,700); break;
      case 1: helper_addBall(500,700); break;
      case 2: helper_addRect(800,800,400,800); break;
      case 3: helper_add2Line(900,200,600); break;
      default: break;
    }

TPE_Body *b = &tpe_world.bodies[tpe_world.bodyCount - 1];

    TPE_bodyMove(b,TPE_vec3((i - 2) * 1200,0,0));


b->friction = 0;
b->elasticity = TPE_FRACTIONS_PER_UNIT;

TPE_bodyAccelerate(b,

TPE_vec3Plus(
TPE_vec3Times(TPE_bodyGetCenterOfMass(b),50),
TPE_vec3(0,10,0))

);
  } 

TPE_Unit sTotal = TPE_worldGetNetSpeed(&tpe_world);

  while (helper_running)
  {
    helper_frameStart();

    helper_cameraFreeMovement();

    if (helper_frame % 16 == 0)
    {
      //helper_printCPU();
      //helper_printCamera();

      if (sdl_keyboard[SDL_SCANCODE_L])
        for (int i = 0; i < tpe_world.bodyCount; ++i)
        {
          TPE_bodyActivate(&tpe_world.bodies[i]);
          TPE_bodyAccelerate(&tpe_world.bodies[i],
            TPE_vec3(0,(500 * 30) / FPS,0));
        }


      timeMeasure = 0;
    }

unsigned long t1 = helper_getMicroSecs();

    TPE_worldStep(&tpe_world);

timeMeasure += helper_getMicroSecs() - t1;

    helper_set3dColor(180,10,10); 

TPE_Unit s = TPE_worldGetNetSpeed(&tpe_world);

TPE_Unit ratio = (sTotal * TPE_FRACTIONS_PER_UNIT) / TPE_nonZero(s);

if (ratio < (4 * TPE_FRACTIONS_PER_UNIT) / 5 ||
  ratio > (6 * TPE_FRACTIONS_PER_UNIT) / 5)
{

printf("net speed is now %d but needs to be %d, correcting!\n",s,sTotal);

for (int i = 0; i < tpe_world.bodyCount; ++i)
  TPE_bodyMultiplyNetSpeed(&tpe_world.bodies[i],ratio);

}

    for (int i = 0; i < tpe_world.bodyCount; ++i)
    {
      TPE_Joint *joints = tpe_world.bodies[i].joints;
      TPE_Vec3 pos = TPE_bodyGetCenterOfMass(&tpe_world.bodies[i]);
      TPE_Vec3 right = TPE_vec3(512,0,0);
      TPE_Vec3 forw = TPE_vec3(0,0,512);

TPE_bodyActivate(&tpe_world.bodies[i]);

      if (i != 1)
      { 
        if (i != 3)
        {
          forw = TPE_vec3Minus(joints[2].position,joints[0].position);
          right = TPE_vec3Minus(joints[1].position,joints[0].position);
        }
        else
          forw = TPE_vec3Minus(joints[1].position,joints[0].position);
      }

      TPE_Vec3 orient = TPE_rotationFromVecs(forw,right);

      switch (i % 5)
      {
        case 0: helper_draw3dBox(pos,TPE_vec3(1200,1200,1200),orient); break;
        case 1: helper_draw3dSphere(pos,TPE_vec3(500,500,500),orient); break;
        case 2: helper_draw3dBox(pos,TPE_vec3(1200,400,1200),orient); break; 
        case 3: helper_draw3dBox(pos,TPE_vec3(200,200,1200),orient); break;
        default: break;
      }
    }

    helper_set3dColor(200,200,200); 

helper_draw3dSphereInside(TPE_vec3(0,0,0),TPE_vec3(SPHERE_R,SPHERE_R,SPHERE_R),
  TPE_vec3(0,0,0) );

    if (helper_debugDrawOn)
      helper_debugDraw(1);

    helper_frameEnd();
  }

  helper_end();

  return 0;
}
