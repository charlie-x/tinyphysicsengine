#define CAMERA_STEP 200

#include "helper.h"

#define GRID_RESOLUTION 8
#define GRID_STEP 1000
#define JOINT_SIZE 300
#define BALL_SIZE 700

#define ROOM_SIZE (GRID_RESOLUTION * GRID_STEP + JOINT_SIZE)

TPE_Vec3 environmentDistance(TPE_Vec3 p, TPE_Unit maxD)
{
  return TPE_envAABoxInside(p,TPE_vec3(0,0,0),TPE_vec3(ROOM_SIZE,ROOM_SIZE,ROOM_SIZE));
}

#define WATER_JOINTS (GRID_RESOLUTION * GRID_RESOLUTION)
#define WATER_CONNECTIONS (2 * ((GRID_RESOLUTION - 1) * GRID_RESOLUTION))

TPE_Joint joints[WATER_JOINTS + 1];
TPE_Connection connections[WATER_CONNECTIONS];

S3L_Unit vertices[WATER_JOINTS * 3];
S3L_Index triangles[((GRID_RESOLUTION - 1) * (GRID_RESOLUTION - 1) * 2) * 3];
S3L_Model3D model;

TPE_Body bodies[2];

TPE_Vec3 jointPlace(int index)
{
  return TPE_vec3((-1 * GRID_RESOLUTION * GRID_STEP) / 2 + (index % GRID_RESOLUTION) * GRID_STEP + GRID_STEP / 2,0,
    (-1 * GRID_RESOLUTION * GRID_STEP) / 2 + (index / GRID_RESOLUTION) * GRID_STEP + GRID_STEP / 2);
}

int main(void)
{
  helper_init();

  puts("WSAD, XC: move the ball");

  helper_debugDrawOn = 1;

  s3l_scene.camera.transform.translation.z = -3000;
  s3l_scene.camera.transform.translation.y = 2000;
  s3l_scene.camera.transform.translation.x = 0;
  s3l_scene.camera.transform.rotation.y = TPE_FRACTIONS_PER_UNIT / 16;

  // build the grid:

  int index = 0;

  for (int j = 0; j < GRID_RESOLUTION; ++j)
    for (int i = 0; i < GRID_RESOLUTION; ++i)
    {
      joints[j * GRID_RESOLUTION + i] = TPE_joint(jointPlace(index),JOINT_SIZE);
      index++;
    }

  index = 0;

  for (int j = 0; j < GRID_RESOLUTION; ++j)
    for (int i = 0; i < GRID_RESOLUTION - 1; ++i)
    {
      connections[index].joint1 = j * GRID_RESOLUTION + i;
      connections[index].joint2 = connections[index].joint1 + 1;

      index++;

      connections[index].joint1 = i * GRID_RESOLUTION + j;
      connections[index].joint2 = connections[index].joint1 + GRID_RESOLUTION;

      index++;
    }

index = 0;

for (int j = 0; j < GRID_RESOLUTION - 1; ++j)
  for (int i = 0; i < GRID_RESOLUTION - 1; ++i)
  {
triangles[index] = j * GRID_RESOLUTION + i;
triangles[index + 1] = triangles[index] + 1;
triangles[index + 2] = triangles[index + 1] + GRID_RESOLUTION;

triangles[index + 3] = triangles[index];
triangles[index + 4] = triangles[index + 1] + GRID_RESOLUTION;
triangles[index + 5] = triangles[index] + GRID_RESOLUTION;

index += 6;
  }

S3L_model3DInit(
  vertices,
  WATER_JOINTS * 3,
  triangles,
  ((GRID_RESOLUTION - 1) * (GRID_RESOLUTION - 1) * 2),
  &model);


  TPE_bodyInit(&bodies[0],joints,WATER_JOINTS,connections,WATER_CONNECTIONS,
    1000);

  bodies[0].flags |= TPE_BODY_FLAG_SOFT;

  joints[WATER_JOINTS] = TPE_joint(TPE_vec3(0,0,ROOM_SIZE / 4),BALL_SIZE);

  TPE_bodyInit(&bodies[1],joints + WATER_JOINTS,1,connections,0,200);

  TPE_worldInit(&tpe_world,bodies,2,environmentDistance);

  while (helper_running)
  {
    helper_frameStart();

    helper_cameraFreeMovement();

TPE_bodyActivate(&bodies[0]);
TPE_bodyActivate(&bodies[1]);

S3L_Unit *v = vertices;

for (int i = 0; i < WATER_JOINTS; ++i)
{
  *v = joints[i].position.x;
  v++;
  *v = joints[i].position.y;
  v++;
  *v = joints[i].position.z;
  v++;
}

for (int index = 0; index < WATER_JOINTS; ++index)
  if (index % GRID_RESOLUTION == 0 || index % GRID_RESOLUTION == GRID_RESOLUTION - 1 ||
      index / GRID_RESOLUTION == 0 || index / GRID_RESOLUTION == GRID_RESOLUTION - 1)
    TPE_jointPin(&joints[index],jointPlace(index));

    TPE_worldStep(&tpe_world);

    TPE_bodyApplyGravity(&tpe_world.bodies[1], bodies[1].joints[0].position.y > 0 ? 5 : -10);

    #define ACC 25
    if (sdl_keyboard[SDL_SCANCODE_W])
      TPE_bodyAccelerate(&bodies[1],TPE_vec3(0,0,ACC));
    else if (sdl_keyboard[SDL_SCANCODE_S])
      TPE_bodyAccelerate(&bodies[1],TPE_vec3(0,0,-1 * ACC));
    else if (sdl_keyboard[SDL_SCANCODE_D])
      TPE_bodyAccelerate(&bodies[1],TPE_vec3(ACC,0,0));
    else if (sdl_keyboard[SDL_SCANCODE_A])
      TPE_bodyAccelerate(&bodies[1],TPE_vec3(-1 * ACC,0,0));
    else if (sdl_keyboard[SDL_SCANCODE_C])
      TPE_bodyAccelerate(&bodies[1],TPE_vec3(0,ACC,0));
    else if (sdl_keyboard[SDL_SCANCODE_X])
      TPE_bodyAccelerate(&bodies[1],TPE_vec3(0,-1 * ACC,0));

helper_set3dColor(255,0,0);

helper_draw3dSphere(bodies[1].joints[0].position,TPE_vec3(BALL_SIZE,BALL_SIZE,BALL_SIZE),TPE_vec3(0,0,0));

helper_set3dColor(0,100,255);
helper_drawModel(&model,TPE_vec3(0,0,0),TPE_vec3(512,512,512),TPE_vec3(0,0,0));

    if (helper_debugDrawOn)
      helper_debugDraw(1);

    helper_frameEnd();
  }

  helper_end();

  return 0;
}
