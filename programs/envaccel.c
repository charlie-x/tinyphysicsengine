/** Boring demo, shows how to accelerate environment functions with bounding
  volume checks. */

#include "helper.h"

#define ACCELERATE 1 // if you turn this off, FPS will drastically drop

TPE_Vec3 environmentDistance(TPE_Vec3 p, TPE_Unit maxD)
{
  TPE_ENV_START( TPE_envAABoxInside(p,TPE_vec3(0,0,0),TPE_vec3(30000,30000,30000)), p )

  for (int bigZ = -1; bigZ < 1; ++bigZ)
    for (int bigY = -1; bigY < 1; ++bigY)
      for (int bigX = -1; bigX < 1; ++bigX)
      {
        TPE_Vec3 bigCenter =
        TPE_vec3(bigX * 10000,bigY * 10000,bigZ * 10000);

#if ACCELERATE
        // bouding volume check for the big chunks of environment
        if (TPE_ENV_BCUBE_TEST(p,maxD,bigCenter,10000))
#endif
          for (int smallZ = 0; smallZ < 2; ++smallZ)
            for (int smallY = 0; smallY < 2; ++smallY)
              for (int smallX = 0; smallX < 2; ++smallX)
              {
                TPE_Vec3 smallCenter = TPE_vec3Plus(
                  bigCenter,
                  TPE_vec3(smallX * 4096,smallY * 4096,smallZ * 4096));

#if ACCELERATE
                // bouding volume check for the smaller subchanks of environment
                if (TPE_ENV_BSPHERE_TEST(p,maxD,smallCenter,4096))
#endif
                {
                  TPE_ENV_NEXT( TPE_envBox(p, smallCenter, TPE_vec3(1000,800,900) ,
                    TPE_vec3(100,20,30) ) ,p );

                  TPE_ENV_NEXT( TPE_envCylinder(p, smallCenter, TPE_vec3(2000,1500,300) , 300 ), p );

                  TPE_ENV_NEXT( TPE_envSphere(p, TPE_vec3Minus(smallCenter,TPE_vec3  (-300,400,0)), 1100  )  , p );
                }
              }
      }

  TPE_ENV_END
}

int main(void)
{
  helper_init();

  helper_debugDrawOn = 1;

  tpe_world.environmentFunction = environmentDistance;

  s3l_scene.camera.transform.translation.z -= 2000;

  while (helper_running)
  {
    helper_frameStart();

    helper_cameraFreeMovement();

    TPE_worldStep(&tpe_world);

    if (helper_frame % 32 == 0)
      helper_printCPU();

    if (helper_debugDrawOn)
      helper_debugDraw(1);

    helper_frameEnd();
  }

  helper_end();

  return 0;
}
