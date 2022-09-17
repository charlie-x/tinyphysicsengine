#define CAMERA_STEP 200

#include "helper.h"

#define GRID_SIZE 3000

TPE_Unit height(int32_t x, int32_t y)
{
  //return x * 20;
  //return 2000 + 100 * x + y;

  return _TPE_hash(x / 3 + y) % 8000;
}

TPE_Vec3 environmentDistance(TPE_Vec3 p, TPE_Unit maxD)
{
/*
#define XX 1
#define YY 0
#define XX2 0
#define YY2 1
return TPE_envLineSegment(p,
    TPE_vec3(XX * 4000,height(XX,YY),YY * 4000),
    TPE_vec3(XX2 * 4000,height(XX2,YY2),YY2 * 4000));
*/
  return TPE_envHeightmap(p,TPE_vec3(0,0,0),GRID_SIZE,height,maxD);

//return TPE_envHalfPlane(p,TPE_vec3(0,0,0),TPE_vec3(2,-512,0));
}

int main(void)
{



  helper_init();

  helper_debugDrawOn = 1;

  tpe_world.environmentFunction = environmentDistance;

  s3l_scene.camera.transform.translation.z = -3000;
  s3l_scene.camera.transform.translation.y = 2000;
  s3l_scene.camera.transform.translation.x = 0;
  s3l_scene.camera.transform.rotation.y = TPE_FRACTIONS_PER_UNIT / 16;

  while (helper_running)
  {
    helper_frameStart();

    helper_cameraFreeMovement();

if (helper_frame % 32 == 0)
{
helper_printCamera();

  printf("cam square: %d %d\n",
    s3l_scene.camera.transform.translation.x / GRID_SIZE -
    (s3l_scene.camera.transform.translation.x < 0),
    s3l_scene.camera.transform.translation.z / GRID_SIZE -
    (s3l_scene.camera.transform.translation.z < 0));
}

    if (helper_debugDrawOn)
      helper_debugDraw(1);

TPE_Vec3 ppp =
  TPE_envHeightmap(
TPE_vec3(
s3l_scene.camera.transform.translation.x,
s3l_scene.camera.transform.translation.y,
s3l_scene.camera.transform.translation.z
)
,TPE_vec3(0,0,0),GRID_SIZE,height,50000);
    
helper_drawPoint3D(ppp,0,255,0);



for (int yy = -5; yy <= 5; ++yy)
  for (int xx = -5; xx <= 5; ++xx)
  {
    TPE_Vec3 aaa = TPE_vec3(xx * GRID_SIZE,height(xx,yy),yy * GRID_SIZE);
    helper_drawPoint3D(aaa,255,0,0);

  } 

    helper_frameEnd();
  }

  helper_end();

  return 0;
}
