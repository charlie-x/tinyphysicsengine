#define CAMERA_STEP 200
#define GRID_SIZE 1000

#define HEIGHTMAP_3D_RESOLUTION 32
#define HEIGHTMAP_3D_STEP GRID_SIZE

#include "helper.h"

TPE_Unit height(int32_t x, int32_t y)
{
  x *= 8;
  y *= 8;

  return 
   TPE_sin(x + TPE_cos(y * 2)) * TPE_sin(y * 2 + TPE_cos(x * 4)) /
    (TPE_FRACTIONS_PER_UNIT / 2);
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
  return TPE_envHeightmap(p,
TPE_vec3(0,0,0),GRID_SIZE,height,maxD);

//return TPE_envHalfPlane(p,TPE_vec3(0,0,0),TPE_vec3(2,-512,0));
}

int main(void)
{

#if 0

  TPE_Vec3 aaa = TPE_envHeightmap(TPE_vec3(-4759,5492,1120 ),
TPE_vec3(0,0,0),GRID_SIZE,height,50000);

// WHY Z 1000 ???? SHOULD BE 0

/*
printf("%d\n",
 TPE_testClosestPointFunction(
environmentDistance,
  TPE_vec3(-10000,-4000,-10000),
  TPE_vec3(10000,4000,10000), 
  10,
  30,
  0)
);
*/

TPE_PRINTF_VEC3(aaa);
printf("\n");

return 0;
#endif

  helper_init();

for (int y = 0; y < HEIGHTMAP_3D_RESOLUTION; ++y)
  for (int x = 0; x < HEIGHTMAP_3D_RESOLUTION; ++x)
    helper_setHeightmapPoint(x,y,height(x - HEIGHTMAP_3D_RESOLUTION / 2,y - HEIGHTMAP_3D_RESOLUTION / 2));

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

helper_drawModel(&heightmapModel,
TPE_vec3(-GRID_SIZE / 2,0,-GRID_SIZE / 2),TPE_vec3(512,512,512),TPE_vec3(0,0,0));

    if (helper_debugDrawOn)
      helper_debugDraw(1);

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
