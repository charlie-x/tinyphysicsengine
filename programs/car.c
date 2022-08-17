#define SCALE_3D_RENDERING 1

#define S3L_NEAR_CROSS_STRATEGY 2
#define S3L_PERSPECTIVE_CORRECTION 2

#include "helper.h"
#include "carArenaModel.h"
#include "carModel.h"

#define ACCELERATION 40
#define TURN_RADIUS 3000

TPE_Vec3 environmentDistance(TPE_Vec3 p, TPE_Unit maxD)
{
TPE_ENV_START( TPE_envHalfPlane(p,TPE_vec3(0,0,0),TPE_vec3(0,512,0)),p )
//  TPE_ENV_NEXT( TPE_envHalfPlane(p,TPE_vec3(0,0,-2000),TPE_vec3(0,255,255)),p )

TPE_ENV_NEXT( TPE_envSphereInside(p,TPE_vec3(0,10000,0),20000),p )
 
 TPE_ENV_NEXT( TPE_envSphere(p,TPE_vec3(0,-200,0),1700),p )
  TPE_ENV_END

}

TPE_Unit averageRot(TPE_Unit a1, TPE_Unit a2)
{
  TPE_Unit d = a2 - a1;

  if (d > 100 || d < -100)
    return a2;

  return a1 + d / 2;
}

uint8_t steering = 0;

TPE_Vec3 carRot;

uint8_t jointCollisions;
TPE_Vec3 jointPreviousPositions[4];

uint8_t collisionCallback(uint16_t b1, uint16_t j1, uint16_t b2, uint16_t j2,
  TPE_Vec3 p)
{
  if (b1 == 0 && b1 == b2 && j1 < 4)
  {
    jointCollisions |= 0x01 << j1;
    jointPreviousPositions[j1] = tpe_world.bodies[0].joints[j1].position;
  }
}

TPE_Vec3 ballRot, ballPreviousPos;

TPE_Vec3 directionVec;

TPE_Body *carBody;

TPE_Vec3 carForw, carSide, carUp, carPos;
TPE_Vec3 carVelocity;

int backOnGround, frontOnGround;

TPE_Unit wheelSize;

int main(void)
{
  arenaModelInit();
  carModelInit();
/*
TPE_Vec3 aaa = TPE_vec3(512,0,0);
TPE_Vec3 mmm = TPE_vec3(10,0,0);
TPE_Vec3 ffff = wheelFriction(aaa,mmm,512);

TPE_PRINTF_VEC3(ffff)

putchar('\n');

return 0;
*/
  helper_init();

  ballRot = TPE_vec3(0,0,0);

carPos = TPE_vec3(0,0,0);

helper_debugDrawOn = 1;

  tpe_world.environmentFunction = environmentDistance;

tpe_world.collisionCallback = collisionCallback;


  helper_addCenterRectFull(1000,1800,400,2000);

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

wheelSize = TPE_JOINT_SIZE(tpe_world.bodies[0].joints[0]) + 30;
  
TPE_bodyMove(carBody,TPE_vec3(3000,1000,0));

tpe_world.bodies[0].elasticity = 64; 
tpe_world.bodies[0].friction = 64; 

  while (helper_running)
  {

    helper_frameStart();

jointCollisions = 0;

    TPE_worldStep(&tpe_world);


for (int i = 0; i < 4; ++i)
{
  if (jointCollisions & (0x01 << i))
  {
TPE_Vec3 axis = carSide;

if (i >= 2 && steering)
{
  if (steering == 1)
    axis = TPE_vec3Plus(carSide,carForw);
  else
    axis = TPE_vec3Minus(carSide,carForw);

  axis = TPE_vec3Normalized(axis);
}


  }
}


S3L_Vec4 toCar;

backOnGround =
TPE_DISTANCE( tpe_world.environmentFunction(
carBody->joints[0].position,wheelSize),
carBody->joints[0].position ) <= wheelSize ||

TPE_DISTANCE( tpe_world.environmentFunction(
carBody->joints[1].position,wheelSize),
carBody->joints[1].position ) <= wheelSize;

frontOnGround =
TPE_DISTANCE( tpe_world.environmentFunction(
carBody->joints[2].position,wheelSize),
carBody->joints[2].position ) <= wheelSize ||

TPE_DISTANCE( tpe_world.environmentFunction(
carBody->joints[3].position,wheelSize),
carBody->joints[3].position ) <= wheelSize;




s3l_scene.camera.transform.translation.y = carPos.y + 800;

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


carForw = 

TPE_vec3Plus(
  TPE_vec3Normalized( 
  TPE_vec3Minus(carBody->joints[2].position,
  carBody->joints[0].position)),

  TPE_vec3Normalized( 
  TPE_vec3Minus(carBody->joints[3].position,
  carBody->joints[1].position)) );

carForw.x /= 2;
carForw.y /= 2;
carForw.z /= 2;

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

if (sdl_keyboard[SDL_SCANCODE_D])
  steering = 1;
else if (sdl_keyboard[SDL_SCANCODE_A])
  steering = 2;
else
  steering = 0;

TPE_Unit speed = TPE_vec3Dot(carVelocity,carForw);

if (speed > CCC)
  speed = CCC;
else if (speed < -1 * CCC)
  speed = -1 * CCC;

speed /= 4;

if (frontOnGround && steering)
{

TPE_Vec3 ccc = TPE_vec3Minus(
carBody->joints[4].position,
TPE_vec3Times(carForw,TURN_RADIUS)
);

TPE_bodySpinWithCenter(carBody,TPE_vec3Times(carUp,
(AAA * speed * (steering == 1 ? -1 : 1)  ) / CCC),ccc);
 
}

if (backOnGround)
{

TPE_Unit acc = ACCELERATION + speed;

#define AAAA 16
if (sdl_keyboard[SDL_SCANCODE_W])
{
  TPE_bodyActivate(carBody);
  carBody->joints[0].velocity[0] += (carForw.x * acc) / TPE_FRACTIONS_PER_UNIT;
  carBody->joints[0].velocity[1] += (carForw.y * acc) / TPE_FRACTIONS_PER_UNIT;
  carBody->joints[0].velocity[2] += (carForw.z * acc) / TPE_FRACTIONS_PER_UNIT;
  carBody->joints[1].velocity[0] += (carForw.x * acc) / TPE_FRACTIONS_PER_UNIT;
  carBody->joints[1].velocity[1] += (carForw.y * acc) / TPE_FRACTIONS_PER_UNIT;
  carBody->joints[1].velocity[2] += (carForw.z * acc) / TPE_FRACTIONS_PER_UNIT;
}
else if (sdl_keyboard[SDL_SCANCODE_S])
{
  TPE_bodyActivate(carBody);
  carBody->joints[0].velocity[0] -= (carForw.x * acc) / TPE_FRACTIONS_PER_UNIT;
  carBody->joints[0].velocity[1] -= (carForw.y * acc) / TPE_FRACTIONS_PER_UNIT;
  carBody->joints[0].velocity[2] -= (carForw.z * acc) / TPE_FRACTIONS_PER_UNIT;
  carBody->joints[1].velocity[0] -= (carForw.x * acc) / TPE_FRACTIONS_PER_UNIT;
  carBody->joints[1].velocity[1] -= (carForw.y * acc) / TPE_FRACTIONS_PER_UNIT;
  carBody->joints[1].velocity[2] -= (carForw.z * acc) / TPE_FRACTIONS_PER_UNIT;
}

//TPE_bodyAccelerate(&tpe_world.bodies[0],TPE_vec3Times(carForw,BBB) );
//      else if (sdl_keyboard[SDL_SCANCODE_S])
//TPE_bodyAccelerate(&tpe_world.bodies[0],TPE_vec3Times(carForw,-1 * BBB) );




}

#undef AAA

    helper_set3dColor(180,180,180);
//    helper_draw3dBoxInside(TPE_vec3(0,ROOM_SIZE / 4,0),TPE_vec3(ROOM_SIZE,ROOM_SIZE / 2,ROOM_SIZE),TPE_vec3(0,0,0));

/*
helper_draw3dPlane(
TPE_vec3(0,0,0),
TPE_vec3(30000,30000,30000),
TPE_vec3(0,0,0));
*/
/*
helper_draw3dPlane(
TPE_vec3(0,1500,-3500),
TPE_vec3(10000,512,4000),
TPE_vec3(-64,0,0));
*/

helper_set3dColor(20,150,150);

helper_drawModel(&arenaModel,TPE_vec3(0,0,0),TPE_vec3(512 * 32,512 * 32,512 * 32), 
  TPE_vec3(0,0,0));

/*
helper_draw3dSphereInside(
TPE_vec3(0,20000,0),TPE_vec3(30000,30000,30000),TPE_vec3(0,0,0));
*/
S3L_zBufferClear();
  
helper_set3dColor(200,200,200);

TPE_Vec3 newRot = 
TPE_bodyGetRotation(&tpe_world.bodies[0],0,2,1);

carRot.x = (TPE_abs(carRot.x - newRot.x) < 50) ?
(carRot.x + newRot.x) / 2 : newRot.x;


carRot.x = averageRot(carRot.x,newRot.x);
carRot.y = averageRot(carRot.y,newRot.y);
carRot.z = averageRot(carRot.z,newRot.z);

helper_drawModel(&carModel,
TPE_vec3Minus(carPos,TPE_vec3Times(carUp,400)),
TPE_vec3(600,600,600), 

carRot
);

/*
helper_draw3dBox(
carPos,
TPE_vec3(1200,800,1200),
TPE_bodyGetRotation(&tpe_world.bodies[0],0,2,1)

);
*/
    


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
