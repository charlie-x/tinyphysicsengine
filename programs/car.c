#define SCALE_3D_RENDERING 1

#define S3L_NEAR_CROSS_STRATEGY 2
#define S3L_PERSPECTIVE_CORRECTION 1

#include "helper.h"

#define ROOM_SIZE 10000
#define CUBE_SIZE 800

TPE_Vec3 environmentDistance(TPE_Vec3 p, TPE_Unit maxD)
{
//  TPE_ENV_START( TPE_envAABoxInside(p,TPE_vec3(0,ROOM_SIZE / 4,0),TPE_vec3(ROOM_SIZE,ROOM_SIZE / 2,ROOM_SIZE)),p )
TPE_ENV_START( TPE_envHalfPlane(p,TPE_vec3(0,0,0),TPE_vec3(0,512,0)),p )
  TPE_ENV_NEXT( TPE_envHalfPlane(p,TPE_vec3(0,0,-2000),TPE_vec3(0,255,255)),p )
  
TPE_ENV_NEXT( TPE_envSphere(p,TPE_vec3(5000,-1000,0),2000),p )
  TPE_ENV_END
}

TPE_Vec3 ballRot, ballPreviousPos;

TPE_Vec3 directionVec;

TPE_Body *carBody;

TPE_Vec3 carForw, carSide, carUp, carPos;
TPE_Vec3 carVelocity;

int onGroundCount = 0;

uint8_t collisionCallback(uint16_t b1, uint16_t j1, uint16_t b2, uint16_t j2,
  TPE_Vec3 p)
{
  if (b1 == 0 && b1 == b2 && (j1 == 0 || j1 == 1))
    onGroundCount = 10;
}

int main(void)
{
  helper_init();

  ballRot = TPE_vec3(0,0,0);

carPos = TPE_vec3(0,0,0);

helper_debugDrawOn = 1;

  tpe_world.environmentFunction = environmentDistance;
  tpe_world.collisionCallback = collisionCallback;

  s3l_scene.camera.transform.translation.z -= ROOM_SIZE / 2;
  s3l_scene.camera.transform.translation.y += ROOM_SIZE / 3;
  s3l_scene.camera.transform.translation.x -= ROOM_SIZE / 4;
  s3l_scene.camera.transform.rotation.y = -1 * TPE_FRACTIONS_PER_UNIT / 16;

  helper_addCenterRectFull(1000,1800,400,400);

tpe_world.bodies[0].joints[4].position.y += 600;
tpe_world.bodies[0].joints[4].sizeDivided *= 3;
tpe_world.bodies[0].joints[4].sizeDivided /= 2;

carBody = &tpe_world.bodies[0];

TPE_bodyInit(&tpe_world.bodies[0],

tpe_world.bodies[0].joints,
tpe_world.bodies[0].jointCount,
tpe_world.bodies[0].connections,
tpe_world.bodies[0].connectionCount,
300
);
  
TPE_bodyMove(&tpe_world.bodies[0],TPE_vec3(0,1000,0));

tpe_world.bodies[0].elasticity = 250; 
tpe_world.bodies[0].friction = 200; 

  while (helper_running)
  {
    if (onGroundCount > 0)
      onGroundCount--;

    helper_frameStart();

    TPE_worldStep(&tpe_world);


S3L_Vec4 toCar;


s3l_scene.camera.transform.translation.y = carPos.y + 450;

TPE_Vec3 cPos = TPE_vec3KeepWithinDistanceBand(
  TPE_vec3(
    s3l_scene.camera.transform.translation.x,
    s3l_scene.camera.transform.translation.y,
    s3l_scene.camera.transform.translation.z
  ),carBody->joints[4].position,2000,3000);

s3l_scene.camera.transform.translation.x = cPos.x;
s3l_scene.camera.transform.translation.y = cPos.y;
s3l_scene.camera.transform.translation.z = cPos.z;

toCar.x = carPos.x - s3l_scene.camera.transform.translation.x;
toCar.y = carPos.y - s3l_scene.camera.transform.translation.y;
toCar.z = carPos.z - s3l_scene.camera.transform.translation.z;
toCar.w = 0;

TPE_Unit angleDiff =
  s3l_scene.camera.transform.rotation.y -
  (TPE_vec2Angle(toCar.x,toCar.z) - 128);


  if (angleDiff < 100 && angleDiff > -100)
    s3l_scene.camera.transform.rotation.y -= angleDiff / 2;
  else
    s3l_scene.camera.transform.rotation.y -= angleDiff;



//s3l_scene.camera.transform.rotation.y =
//  angle;

s3l_scene.camera.transform.rotation.x = - 35;


carPos = TPE_vec3KeepWithinBox(carPos,
 carBody->joints[4].position,TPE_vec3(20,20,20));



//S3L_lookAt(toCar,&s3l_scene.camera.transform);


carForw = TPE_vec3Normalized( 
  TPE_vec3Minus(carBody->joints[2].position,
  carBody->joints[0].position));

carSide = TPE_vec3Normalized( 
  TPE_vec3Minus(carBody->joints[1].position,
  carBody->joints[0].position));

carUp = TPE_vec3Cross(carForw,carSide);

#define AAA 8
#define BBB 16
#define CCC 60

//helper_cameraFreeMovement();


carVelocity.x = carBody->joints[4].velocity[0];
carVelocity.y = carBody->joints[4].velocity[1];
carVelocity.z = carBody->joints[4].velocity[2];

if (onGroundCount > 0)
{
      if (sdl_keyboard[SDL_SCANCODE_W])
TPE_bodyAccelerate(&tpe_world.bodies[0],TPE_vec3Times(carForw,BBB) );
      else if (sdl_keyboard[SDL_SCANCODE_S])
TPE_bodyAccelerate(&tpe_world.bodies[0],TPE_vec3Times(carForw,-1 * BBB) );

TPE_Unit speed = TPE_LENGTH(carVelocity);

speed -= 40;

if (speed < 0)
  speed = 0;
else if (speed > CCC)
  speed = CCC;


      if (sdl_keyboard[SDL_SCANCODE_D])
TPE_bodySpin(&tpe_world.bodies[0],TPE_vec3Times(carUp,
(-1 * AAA * speed) / CCC ) );
      if (sdl_keyboard[SDL_SCANCODE_A])
TPE_bodySpin(&tpe_world.bodies[0],TPE_vec3Times(carUp,
(AAA * speed) / CCC ) );
}

#undef AAA

    helper_set3dColor(180,180,180);
//    helper_draw3dBoxInside(TPE_vec3(0,ROOM_SIZE / 4,0),TPE_vec3(ROOM_SIZE,ROOM_SIZE / 2,ROOM_SIZE),TPE_vec3(0,0,0));
helper_draw3dPlane(
TPE_vec3(0,0,0),
TPE_vec3(8000,8000,8000),
TPE_vec3(0,0,0));

helper_draw3dPlane(
TPE_vec3(0,1500,-3500),
TPE_vec3(10000,512,4000),
TPE_vec3(-64,0,0));
    
helper_set3dColor(200,50,0);

helper_draw3dBox(
carPos,
TPE_vec3(1200,800,1200),
TPE_bodyGetRotation(&tpe_world.bodies[0],0,2,1)

);

    


    helper_set3dColor(200,10,10);

for (int i = 0; i < tpe_world.bodyCount; ++i)
    TPE_bodyApplyGravity(&tpe_world.bodies[i],5);

    if (helper_debugDrawOn)
      helper_debugDraw(1);

    helper_frameEnd();
  }

  helper_end();

  return 0;
}
