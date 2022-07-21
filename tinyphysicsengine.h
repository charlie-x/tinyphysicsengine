#ifndef _TINYPHYSICSENGINE_H
#define _TINYPHYSICSENGINE_H

/**
  WORK IN PROGRESS, UNUSABLE YET

  Simple/suckless header-only hybrid 3D physics engine with no floating point,
  only 32 bit int arithmetic, similar to e.g. small3dlib.
  
  Conventions and formats are the same or similar to those of small3dlib so as
  to make them easily integrate with each other.

  Orientations/rotations are in extrinsic Euler angles in the ZXY order (by Z,
  then by X, then by Y).

  Where it matters (e.g. rotations about axes) we consider a left-handed coord.
  system (x right, y up, z forward).

  --------------------

  by drummyfish, 2022

  This work's goal is to never be encumbered by any exclusive intellectual
  property rights. The work is therefore provided under CC0 1.0 + additional
  WAIVER OF ALL INTELLECTUAL PROPERTY RIGHTS that waives the rest of
  intellectual property rights not already waived by CC0 1.0. The WAIVER OF ALL
  INTELLECTUAL PROPERTY RGHTS is as follows:

  Each contributor to this work agrees that they waive any exclusive rights,
  including but not limited to copyright, patents, trademark, trade dress,
  industrial design, plant varieties and trade secrets, to any and all ideas,
  concepts, processes, discoveries, improvements and inventions conceived,
  discovered, made, designed, researched or developed by the contributor either
  solely or jointly with others, which relate to this work or result from this
  work. Should any waiver of such right be judged legally invalid or
  ineffective under applicable law, the contributor hereby grants to each
  affected person a royalty-free, non transferable, non sublicensable, non
  exclusive, irrevocable and unconditional license to this right.
*/

#include <stdint.h>

typedef int32_t TPE_Unit;
typedef int16_t TPE_UnitReduced;        ///< Like TPE_Unit but saving space

#define TPE_FRACTIONS_PER_UNIT 512

#define TPE_JOINT_SIZE_MULTIPLIER 32

#define TPE_INFINITY 2147483647

#define TPE_JOINT_SIZE(joint) ((joint).sizeDivided * TPE_JOINT_SIZE_MULTIPLIER)

#ifndef TPE_APPROXIMATE_LENGTH
  #define TPE_APPROXIMATE_LENGTH 0      /**< whether or not use length/distance 
                                           approximation rather than exact 
                                           calculation (1 is faster but less
                                           accurate)  */
#endif

#if !TPE_APPROXIMATE_LENGTH
  #define TPE_DISTANCE TPE_dist
  #define TPE_LENGTH TPE_vec3Len
#else
  #define TPE_DISTANCE TPE_distApprox
  #define TPE_LENGTH TPE_vec3LenApprox
#endif

// TODO: faster and more accurate distance approx function based on regions/LUT

#ifndef TPE_LOG
  #define TPE_LOG(s) ;
#endif

#ifndef TPE_LOW_SPEED
/** Speed, in TPE_Units per ticks, that is considered low (used e.g. for auto
disabling bodies). */
  #define TPE_LOW_SPEED 30
#endif

#ifndef TPE_RESHAPE_TENSION_LIMIT
/** Tension limit, in TPE_Units, after which a non-soft body will be reshaped.
Smaller number will keep more stable shapes but will cost more performance. */
  #define TPE_RESHAPE_TENSION_LIMIT 20
#endif

#ifndef TPE_RESHAPE_ITERATIONS
/** How many iterations of reshaping will be performed by the step function if
the body's shape needs to be reshaped. Greater number will keep shapes more
stable but will cost some performance. */
  #define TPE_RESHAPE_ITERATIONS 3
#endif

#ifndef TPE_DEACTIVATE_AFTER
/** After how many ticks of low speed should a body be disabled. This mustn't
be greater than 255. */
  #define TPE_DEACTIVATE_AFTER 128
#endif

#ifndef TPE_LIGHT_DEACTIVATION
/** When a body is activated by a collision, its deactivation counter will be
set to this value, i.e. after a collision the body will be prone to deactivate
sooner than normally. This is to handle situations with many bodies touching
each other that would normally keep activating each other, never coming to
rest. */
  #define TPE_LIGHT_DEACTIVATION \
    (TPE_DEACTIVATE_AFTER - TPE_DEACTIVATE_AFTER / 10)
#endif

#ifndef TPE_TENSION_ACCELERATION_DIVIDER
/** Number by which the base acceleration (TPE_FRACTIONS_PER_UNIT per tick
squared) caused by the connection tension will be divided. This should be power
of 2. */
  #define TPE_TENSION_ACCELERATION_DIVIDER 32
#endif

#ifndef TPE_TENSION_ACCELERATION_THRESHOLD
/** Limit within which acceleration caused by connection tension won't be
  applied. */
  #define TPE_TENSION_ACCELERATION_THRESHOLD 5
#endif

#ifndef TPE_COLLISION_RESOLUTION_ITERATIONS
/** Maximum number of iterations to try to uncollide two colliding bodies. */
  #define TPE_COLLISION_RESOLUTION_ITERATIONS 3
#endif

#ifndef TPE_COLLISION_RESOLUTION_MARGIN
/** Margin, in TPE_Units, by which a body will be shifted back to get out of
  collision. */
  #define TPE_COLLISION_RESOLUTION_MARGIN (TPE_FRACTIONS_PER_UNIT / 64)
#endif

#define TPE_PRINTF_VEC3(v) printf("[%d %d %d]",(v).x,(v).y,(v).z);

typedef struct
{
  TPE_Unit x;
  TPE_Unit y;
  TPE_Unit z;
} TPE_Vec3;

typedef struct
{
  TPE_Vec3 position;
  TPE_UnitReduced velocity[3];
  uint8_t sizeDivided; /**< size (radius, ...), for saving space divided by 
                            TPE_JOINT_SIZE_MULTIPLIER */
} TPE_Joint;

typedef struct
{
  uint8_t joint1;
  uint16_t joint2;
  uint16_t length;     ///< connection's preferred length, uint16_t saves space
} TPE_Connection;

#define TPE_BODY_FLAG_DEACTIVATED 1  /**< Not being updated due to low energy,
                                          "sleeping", will be woken by
                                          collisions etc. */
#define TPE_BODY_FLAG_NONROTATING 2  /**< When set, the body won't rotate, will
                                          only move linearly. */
#define TPE_BODY_FLAG_DISABLED 4     /**< Disabled, not taking part in
                                          simulation. */
#define TPE_BODY_FLAG_SOFT 8         /**< Soft connections, effort won't be made
                                          to keep the body's shape. */

/** Function used for defining static environment, working similarly to an SDF
(signed distance function). The parameters are: 3D point P, max distance D.
The function should behave like this: if P is inside the solid environment
volume, P will be returned; otherwise closest point (by Euclidean distance) to
the solid environment volume from P will be returned, except for a case when
this closest point would be further away than D, in which case any arbitrary
point further away than D may be returned (this allows for potentially 
potentially faster implementation). */
typedef TPE_Vec3 (*TPE_ClosestPointFunction)(TPE_Vec3, TPE_Unit);

/** Function used by the debug drawing functions to draw individual pixels to
  the screen. The parameters are following: pixel x, pixel y, pixel color. */
typedef void (*TPE_DebugDrawFunction)(uint16_t, uint16_t, uint8_t);

typedef struct
{
  TPE_Joint *joints;
  uint8_t jointCount;
  TPE_Connection *connections;
  uint8_t connectionCount;
  TPE_UnitReduced jointMass;
  TPE_UnitReduced friction;
  TPE_UnitReduced elasticity;
  uint8_t flags;
  uint8_t deactivateCount;
} TPE_Body;

typedef struct
{
  TPE_Body *bodies;
  uint16_t bodyCount;
  TPE_ClosestPointFunction environmentFunction;
} TPE_World;

void TPE_bodyInit(TPE_Body *body, 
  TPE_Joint *joints, uint8_t jointCount, 
  TPE_Connection *connections, uint8_t connectionCount,
  TPE_Unit mass);

void TPE_worldInit(TPE_World *world,
  TPE_Body *bodies, uint16_t bodyCount,
  TPE_ClosestPointFunction environmentFunction);

void TPE_vec3Normalize(TPE_Vec3 *v);

TPE_Vec3 TPE_pointRotate(TPE_Vec3 point, TPE_Vec3 rotation);

TPE_Vec3 TPE_rotationInverse(TPE_Vec3 rotation);

void TPE_getVelocitiesAfterCollision(
  TPE_Unit *v1,
  TPE_Unit *v2,
  TPE_Unit m1,
  TPE_Unit m2,
  TPE_Unit elasticity);

TPE_Vec3 TPE_vec3(TPE_Unit x, TPE_Unit y, TPE_Unit z);
TPE_Vec3 TPE_vec3Minus(TPE_Vec3 v1, TPE_Vec3 v2);
TPE_Vec3 TPE_vec3Plus(TPE_Vec3 v1, TPE_Vec3 v2);
TPE_Vec3 TPE_vec3Cross(TPE_Vec3 v1, TPE_Vec3 v2);
TPE_Vec3 TPE_vec3Project(TPE_Vec3 v, TPE_Vec3 base);
TPE_Vec3 TPE_vec3Times(TPE_Vec3 v, TPE_Unit units);
TPE_Vec3 TPE_vec3TimesNonNormalized(TPE_Vec3 v, TPE_Unit q);
TPE_Vec3 TPE_vec3Normalized(TPE_Vec3 v);

/* Computes orientation/rotation (see docs for orientation format) from two
  vectors (which should be at least a close to being perpensicular and do NOT
  need to be normalized). */
TPE_Vec3 TPE_orientationFromVecs(TPE_Vec3 forward, TPE_Vec3 right);

TPE_Unit TPE_vec3Dot(TPE_Vec3 v1, TPE_Vec3 v2);

TPE_Unit TPE_sqrt(TPE_Unit value);

TPE_Unit TPE_vec3Len(TPE_Vec3 v);
TPE_Unit TPE_vec3LenApprox(TPE_Vec3 v);

static inline TPE_Unit TPE_nonZero(TPE_Unit x);

static inline TPE_Unit TPE_dist(TPE_Vec3 p1, TPE_Vec3 p2);
static inline TPE_Unit TPE_distApprox(TPE_Vec3 p1, TPE_Vec3 p2);

TPE_Joint TPE_joint(TPE_Vec3 position, TPE_Unit size);

uint8_t TPE_jointsResolveCollision(TPE_Joint *j1, TPE_Joint *j2,
  TPE_Unit mass1, TPE_Unit mass2, TPE_Unit elasticity, TPE_Unit friction);

/** Tests and potentially resolves a collision between a joint and environment,
  returns 0 if no collision happened, 1 if it happened and was resolved normally
  and 2 if it couldn't be resolved normally. */
uint8_t TPE_jointEnvironmentResolveCollision(TPE_Joint *joint, TPE_Unit
  elasticity, TPE_Unit friction, TPE_ClosestPointFunction env);

uint8_t TPE_bodyEnvironmentCollide(const TPE_Body *body,
  TPE_ClosestPointFunction env);

uint8_t TPE_bodyEnvironmentResolveCollision(TPE_Body *body, 
  TPE_ClosestPointFunction env);

void TPE_bodyGetAABB(const TPE_Body *body, TPE_Vec3 *vMin, TPE_Vec3 *vMax);

uint8_t TPE_checkOverlapAABB(TPE_Vec3 v1Min, TPE_Vec3 v1Max, TPE_Vec3 v2Min,
  TPE_Vec3 v2Max);

uint8_t TPE_bodiesResolveCollision(TPE_Body *b1, TPE_Body *b2);

void TPE_jointPin(TPE_Joint *joint, TPE_Vec3 position);

// -----------------------------------------------------------------------------
// body generation functions:

void TPE_makeBox(TPE_Joint joints[8], TPE_Connection connections[16],
  TPE_Unit width, TPE_Unit depth, TPE_Unit height, TPE_Unit jointSize);

void TPE_makeCenterBox(TPE_Joint joints[9], TPE_Connection connections[18],
  TPE_Unit width, TPE_Unit depth, TPE_Unit height, TPE_Unit jointSize);

void TPE_makeRect(TPE_Joint joints[4], TPE_Connection connections[6],
  TPE_Unit width, TPE_Unit depth, TPE_Unit jointSize);

void TPE_makeTriangle(TPE_Joint joints[3], TPE_Connection connections[3],
  TPE_Unit sideLength, TPE_Unit jointSize);

void TPE_makeCenterRect(TPE_Joint joints[5], TPE_Connection connections[8],
  TPE_Unit width, TPE_Unit depth,  TPE_Unit jointSize);

void TPE_make2Line(TPE_Joint joints[2], TPE_Connection connections[1],
  TPE_Unit length, TPE_Unit jointSize);

//------------------------------------------------------------------------------
// environment functions:

TPE_Vec3 TPE_envAABoxInside(TPE_Vec3 point, TPE_Vec3 center, TPE_Vec3 size);
TPE_Vec3 TPE_envAABox(TPE_Vec3 point, TPE_Vec3 center, TPE_Vec3 maxCornerVec);
TPE_Vec3 TPE_envBox(TPE_Vec3 point, TPE_Vec3 center, TPE_Vec3 maxCornerVec,
  TPE_Vec3 rotation);
TPE_Vec3 TPE_envSphere(TPE_Vec3 point, TPE_Vec3 center, TPE_Unit radius);
TPE_Vec3 TPE_envHalfPlane(TPE_Vec3 point, TPE_Vec3 center, TPE_Vec3 normal);

//---------------------------

void TPE_worldStep(TPE_World *world);

TPE_Unit TPE_bodyNetSpeed(const TPE_Body *body);
TPE_Unit TPE_bodyAverageSpeed(const TPE_Body *body);

void TPE_bodyDeactivate(TPE_Body *body);

void TPE_bodyLimitAverageSpeed(TPE_Body *body, TPE_Unit speedMin,
  TPE_Unit speedMax);

void TPE_bodyMultiplyNetSpeed(TPE_Body *body, TPE_Unit factor);

/** Attempts to shift the joints of a soft body so that the tension of all
strings becomes zero while keeping the joints near their current position. This
function performs one iteration of the equalizing algorithm and doesn't
guarantee a perfect solution, it may help to run multiple iterations (call this
function multiple times). */
void TPE_bodyReshape(TPE_Body *body, TPE_ClosestPointFunction
  environmentFunction);

/** Move a soft body by certain offset. */
void TPE_bodyMove(TPE_Body *body, TPE_Vec3 offset);

/** Zero velocities of all soft body joints. */
void TPE_bodyStop(TPE_Body *body);

void TPE_bodyActivate(TPE_Body *body);

/** Add velocity to a soft body. */
void TPE_bodyAccelerate(TPE_Body *body, TPE_Vec3 velocity);

/** Add angular velocity to a soft body. The rotation vector specifies the axis
of rotation by its direction and angular velocity by its magnitude (magnitude
of TPE_FRACTIONS_PER_UNIT will add linear velocity of TPE_FRACTIONS_PER_UNIT per
tick to a point in the distance of TPE_FRACTIONS_PER_UNIT from the rotation
axis). */
void TPE_bodySpin(TPE_Body *body, TPE_Vec3 rotation);

/** Instantly rotate soft body, the rotation vector specifies the rotation axis
by its direction and the rotation angle by its magnitude (TPE_FRACTIONS_PER_UNIT
stands for full angle, i.e. 2 * PI x). */
void TPE_bodyRotateByAxis(TPE_Body *body, TPE_Vec3 rotation);

/** Compute the center of mass of a soft body. */
TPE_Vec3 TPE_bodyGetCenter(const TPE_Body *body);

/** Compute sine, TPE_FRACTIONS_PER_UNIT as argument corresponds to 2 * PI
  radians. Returns a number from -TPE_FRACTIONS_PER_UNIT to
  TPE_FRACTIONS_PER_UNIT. */
TPE_Unit TPE_sin(TPE_Unit x);

TPE_Unit TPE_cos(TPE_Unit x);

TPE_Unit TPE_atan(TPE_Unit x);

/** Draws a debug view of a 3D physics world using a provided pixel drawing
  function. This can be used to overlay a simple visualization of the physics
  objects to your main render, to spot exact borders of objects etc. The
  function draws simple dotted lines and circles with different "colors" for
  different types of objects (joints, connections, environemnt). camPos, camRot
  and camView should match the camera settings of your main renderer. CamView.x
  is horizontal resolution in pixels, camView.y is the vertical resolution,
  CamView.z says the camera focal length (~FOV) in TPE_Units. envGridRes is the
  resolution of an environment probe grid (the function will probe points in
  space and draw borders of the physics environemnt), envGridSize is the size
  (int TPE_Units) of the grid cell. Note the function may be slow (reducing
  envGridRes can help, workable value can be e.g. 16). */
void TPE_worldDebugDraw(TPE_World *world, TPE_DebugDrawFunction drawFunc,
  TPE_Vec3 camPos, TPE_Vec3 camRot, TPE_Vec3 camView, uint16_t envGridRes,
  TPE_Unit envGridSize);

//------------------------------------------------------------------------------

static inline TPE_Unit TPE_nonZero(TPE_Unit x)
{
  return x != 0 ? x : 1;
}

TPE_Joint TPE_joint(TPE_Vec3 position, TPE_Unit size)
{
  TPE_Joint result;

  result.velocity[0] = 0;
  result.velocity[1] = 0;
  result.velocity[2] = 0;
 
  result.position = position;

  size /= TPE_JOINT_SIZE_MULTIPLIER;

  if (size > 0xff)
    TPE_LOG("WARNING: joint size too big in TPE_joint");

  result.sizeDivided = size;

  return result;
}

TPE_Vec3 TPE_vec3(TPE_Unit x, TPE_Unit y, TPE_Unit z)
{
  TPE_Vec3 r;

  r.x = x;
  r.y = y;
  r.z = z;

  return r;
}

TPE_Unit TPE_sqrt(TPE_Unit value)
{
  int8_t sign = 1;

  if (value < 0)
  {
    sign = -1;
    value *= -1;
  }

  uint32_t result = 0;
  uint32_t a = value;
  uint32_t b = 1u << 30;

  while (b > a)
    b >>= 2;

  while (b != 0)
  {
    if (a >= result + b)
    {
      a -= result + b;
      result = result +  2 * b;
    }

    b >>= 2;
    result >>= 1;
  }

  return result * sign;
}

TPE_Unit TPE_vec3Len(TPE_Vec3 v)
{
  return TPE_sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

TPE_Unit TPE_vec3LenApprox(TPE_Vec3 v)
{
  if (v.x < 0)
    v.x *= -1;

  if (v.y < 0)
    v.y *= -1;

  if (v.z < 0)
    v.z *= -1;

  TPE_Unit sum = v.x + v.y + v.z;

  v.x = (v.x > v.y) ? 
    (v.x > v.z ? v.x : v.z) : 
    (v.y > v.z ? v.y : v.z);

  return (v.x + sum) / 2;
}

TPE_Unit TPE_dist(TPE_Vec3 p1, TPE_Vec3 p2)
{
  p1 = TPE_vec3Minus(p1,p2);
  return TPE_vec3Len(p1); 
}

TPE_Unit TPE_distApprox(TPE_Vec3 p1, TPE_Vec3 p2)
{
  p1 = TPE_vec3Minus(p1,p2);
  return TPE_vec3LenApprox(p1); 
}

void TPE_bodyInit(TPE_Body *body, 
  TPE_Joint *joints, uint8_t jointCount, 
  TPE_Connection *connections, uint8_t connectionCount,
  TPE_Unit mass)
{
  body->joints = joints;
  body->jointCount = jointCount;
  body->connections = connections;
  body->connectionCount = connectionCount;
  body->deactivateCount = 0;
  body->friction = TPE_FRACTIONS_PER_UNIT / 2;
  body->elasticity = TPE_FRACTIONS_PER_UNIT / 2;

  body->flags = 0;

  body->jointMass = mass / jointCount;
 
  if (body->jointMass == 0)
    body->jointMass = 1;

  for (uint32_t i = 0; i < connectionCount; ++i)
  {
    TPE_Unit d = TPE_DISTANCE(
      joints[connections[i].joint1].position,
      joints[connections[i].joint2].position);

    if (d > 0xffff)
      TPE_LOG("WARNING: joint distance too long in TPE_bodyInit");

    connections[i].length = d != 0 ? d : 1; // prevent later division by zero
  }
}

void TPE_worldInit(TPE_World *world,
  TPE_Body *bodies, uint16_t bodyCount,
  TPE_ClosestPointFunction environmentFunction)
{
  world->bodies = bodies;
  world->bodyCount = bodyCount;
  world->environmentFunction = environmentFunction;
}
  
#define C(n,a,b) connections[n].joint1 = a; connections[n].joint2 = b;

void TPE_make2Line(TPE_Joint joints[2], TPE_Connection connections[1], 
  TPE_Unit length, TPE_Unit jointSize)
{
  joints[0] = TPE_joint(TPE_vec3(length / 2,0,0),jointSize);
  joints[1] = TPE_joint(TPE_vec3(length / -2,0,0),jointSize);

  C(0, 0,1)
}

void TPE_makeRect(TPE_Joint joints[4], TPE_Connection connections[6],
  TPE_Unit width, TPE_Unit depth,  TPE_Unit jointSize)
{
  width /= 2;
  depth /= 2;

  for (uint8_t i = 0; i < 4; ++i)
    joints[i] = TPE_joint(
      TPE_vec3((i % 2) ? -1 * width : width,
      0,(i / 2) ? - 1 * depth : depth),
      jointSize);

  C(0, 0,1) C(1, 0,2) C (2, 3,1) C(3, 3,2)
  C(4, 0,3) C(5, 1,2)
}

void TPE_makeCenterRect(TPE_Joint joints[5], TPE_Connection connections[8],
  TPE_Unit width, TPE_Unit depth,  TPE_Unit jointSize)
{
  TPE_makeRect(joints,connections,width,depth,jointSize);

  joints[4] = TPE_joint(TPE_vec3(0,0,0),jointSize);

  C(6, 0,4) C(7, 3,4)
}

void TPE_makeTriangle(TPE_Joint joints[3], TPE_Connection connections[3],
  TPE_Unit sideLength, TPE_Unit jointSize)
{
  joints[0] = TPE_joint(TPE_vec3(sideLength / 2,0,
    TPE_sqrt((sideLength * sideLength) / 2) / 2),
    jointSize);

  joints[1] = joints[0];
  joints[1].position.x *= -1;

  joints[2] = TPE_joint(TPE_vec3(0,0,-1 * joints[0].position.z),jointSize);

  C(0, 0,1) C(1, 1,2) C(2, 2,0)
}

void TPE_makeBox(TPE_Joint joints[8], TPE_Connection connections[16],
  TPE_Unit width, TPE_Unit depth, TPE_Unit height, TPE_Unit jointSize)
{
  width /= 2;
  depth /= 2;
  height /= 2;

  for (uint8_t i = 0; i < 8; ++i)
    joints[i] = TPE_joint( 
      TPE_vec3(  
        (i % 2) ? width : (-1 * width),
        ((i >> 2) % 2) ? height : (-1 * height),
        ((i >> 1) % 2) ? depth : (-1 * depth)),
      jointSize);

  C(0, 0,1) C(1, 1,3) C(2, 3,2) C(3, 2,0)  // top
  C(4, 4,5) C(5, 5,7) C(6, 7,6) C(7, 6,4)  // bottom
  C(8, 0,4) C(9, 1,5) C(10,3,7) C(11,2,6)  // middle
  C(12,0,7) C(13,1,6) C(14,2,5) C(15,3,4)  // diagonal
}

void TPE_makeCenterBox(TPE_Joint joints[9], TPE_Connection connections[18],
  TPE_Unit width, TPE_Unit depth, TPE_Unit height, TPE_Unit jointSize)
{
  TPE_makeBox(joints,connections,width,depth,height,jointSize);

  joints[8] = TPE_joint(TPE_vec3(0,0,0),jointSize);

  C(16, 0,8) C(17, 7,8)
}
  
#undef C

void TPE_bodyDeactivate(TPE_Body *body)
{
  body->flags |= TPE_BODY_FLAG_DEACTIVATED;
}

void TPE_worldStep(TPE_World *world)
{
  for (uint16_t i = 0; i < world->bodyCount; ++i)
  {
    TPE_Body *body = world->bodies + i;   

    if (body->flags & (TPE_BODY_FLAG_DEACTIVATED | TPE_BODY_FLAG_DISABLED))
      continue; 

    TPE_Joint *joint = body->joints, *joint2;

    for (uint16_t j = 0; j < body->jointCount; ++j) // apply velocities
    {
      // non-rotating bodies will copy the 1st joint's velocity

      if (body->flags & TPE_BODY_FLAG_NONROTATING)
        for (uint8_t k = 0; k < 3; ++k)
          joint->velocity[k] = body->joints[0].velocity[k];

      joint->position.x += joint->velocity[0];
      joint->position.y += joint->velocity[1];
      joint->position.z += joint->velocity[2];

      joint++;
    }

    TPE_Connection *connection = body->connections;

    TPE_Vec3 aabbMin, aabbMax;

    TPE_bodyGetAABB(body,&aabbMin,&aabbMax);
 
    for (uint16_t j = 0; j < world->bodyCount; ++j)
    {
      if (j > i ||  (world->bodies[j].flags & TPE_BODY_FLAG_DEACTIVATED))
      {
        // firstly quick-check collision of body AA bounding boxes

        TPE_Vec3 aabbMin2, aabbMax2;
        TPE_bodyGetAABB(&world->bodies[j],&aabbMin2,&aabbMax2);

        if (TPE_checkOverlapAABB(aabbMin,aabbMax,aabbMin2,aabbMax2) &&
          TPE_bodiesResolveCollision(body,world->bodies + j))
        {
          TPE_bodyActivate(body);
          body->deactivateCount = TPE_LIGHT_DEACTIVATION; 

          TPE_bodyActivate(world->bodies + j);
          world->bodies[j].deactivateCount = TPE_LIGHT_DEACTIVATION;
        }
      }
    }
 
    TPE_bodyEnvironmentResolveCollision(body,world->environmentFunction);

    TPE_Unit bodyTension = 0;

    if (!(body->flags & TPE_BODY_FLAG_NONROTATING))
    {
      for (uint16_t j = 0; j < body->connectionCount; ++j) // update velocities
      {
        joint  = &(body->joints[connection->joint1]);
        joint2 = &(body->joints[connection->joint2]);

        TPE_Vec3 dir = TPE_vec3Minus(joint2->position,joint->position);

        TPE_Unit len = TPE_LENGTH(dir);

        len = (len * TPE_FRACTIONS_PER_UNIT) /
          connection->length - TPE_FRACTIONS_PER_UNIT;

        bodyTension += len > 0 ? len : -len;

        if (len > TPE_TENSION_ACCELERATION_THRESHOLD || 
          len < -1 * TPE_TENSION_ACCELERATION_THRESHOLD)
        {
          TPE_vec3Normalize(&dir);

          dir.x /= TPE_TENSION_ACCELERATION_DIVIDER;
          dir.y /= TPE_TENSION_ACCELERATION_DIVIDER;
          dir.z /= TPE_TENSION_ACCELERATION_DIVIDER;

          if (len < 0)
          {
            dir.x *= -1;
            dir.y *= -1;
            dir.z *= -1;
          }

          joint->velocity[0] += dir.x;
          joint->velocity[1] += dir.y;
          joint->velocity[2] += dir.z;

          joint2->velocity[0] -= dir.x;
          joint2->velocity[1] -= dir.y;
          joint2->velocity[2] -= dir.z;
        }

        connection++;
      }

      if (body->connectionCount > 0 && !(body->flags & TPE_BODY_FLAG_SOFT))
      {
        TPE_bodyReshape(body,world->environmentFunction);

        bodyTension /= body->connectionCount;
      
        if (bodyTension > TPE_RESHAPE_TENSION_LIMIT)
          for (uint8_t k = 0; k < TPE_RESHAPE_ITERATIONS; ++k) 
            TPE_bodyReshape(body,world->environmentFunction);
      }
    } // if (rotating)

    if (body->deactivateCount >= TPE_DEACTIVATE_AFTER)
    {
      TPE_bodyStop(body);
      body->deactivateCount = 0;
      body->flags |= TPE_BODY_FLAG_DEACTIVATED;
    }
    else if (TPE_bodyAverageSpeed(body) <= TPE_LOW_SPEED) // TODO: optimize
      body->deactivateCount++;
    else
      body->deactivateCount = 0;
  }
}

void TPE_bodyActivate(TPE_Body *body)
{
  // the if check has to be here, don't remove it

  if (body->flags & TPE_BODY_FLAG_DEACTIVATED)
  {
    TPE_bodyStop(body);
    body->flags &= ~TPE_BODY_FLAG_DEACTIVATED;
    body->deactivateCount = 0;
  }
}

TPE_Unit TPE_bodyNetSpeed(const TPE_Body *body)
{
  TPE_Unit velocity = 0;

  const TPE_Joint *joint = body->joints;

  for (uint16_t i = 0; i < body->jointCount; ++i)
  {
    velocity += TPE_LENGTH(
     TPE_vec3(joint->velocity[0],joint->velocity[1],joint->velocity[2]));

    joint++;
  }

  return velocity;
}

TPE_Unit TPE_bodyAverageSpeed(const TPE_Body *body)
{
  return TPE_bodyNetSpeed(body) / body->jointCount;
}

void TPE_bodyMultiplyNetSpeed(TPE_Body *body, TPE_Unit factor)
{
  TPE_Joint *joint = body->joints;

  for (uint16_t j = 0; j < body->jointCount; ++j)
  {
    for (uint8_t k = 0; k < 3; ++k)
      joint->velocity[k] = 
        (((TPE_Unit) joint->velocity[k]) * factor) /
        TPE_FRACTIONS_PER_UNIT;

    joint++;
  }
}

void TPE_bodyLimitAverageSpeed(TPE_Body *body, TPE_Unit speedMin,
  TPE_Unit speedMax)
{
  for (uint8_t i = 0; i < 16; ++i)
  {
    TPE_Unit speed = TPE_bodyAverageSpeed(body);

    if (speed >= speedMin && speed <= speedMax)
      return;

    TPE_Unit fraction =
      (((speedMax + speedMin) / 2) * TPE_FRACTIONS_PER_UNIT) /
      TPE_nonZero(speed);
    
    TPE_bodyMultiplyNetSpeed(body,fraction);
  }
}

void TPE_bodyReshape(TPE_Body *body, 
  TPE_ClosestPointFunction environmentFunction)
{
  for (uint16_t i = 0; i < body->connectionCount; ++i)
  {
    TPE_Connection *c = &body->connections[i];

    TPE_Joint *j1 = &(body->joints[c->joint1]);
    TPE_Joint *j2 = &(body->joints[c->joint2]);
      
    TPE_Vec3 dir = TPE_vec3Minus(j2->position,j1->position);

    TPE_Vec3 middle = TPE_vec3Plus(j1->position,j2->position);

    middle.x /= 2;
    middle.y /= 2;
    middle.z /= 2;

    TPE_vec3Normalize(&dir);

    dir.x = (dir.x * c->length) / TPE_FRACTIONS_PER_UNIT;
    dir.y = (dir.y * c->length) / TPE_FRACTIONS_PER_UNIT;
    dir.z = (dir.z * c->length) / TPE_FRACTIONS_PER_UNIT;

    TPE_Vec3 positionBackup = j1->position;

    j1->position.x = middle.x - dir.x / 2;
    j1->position.y = middle.y - dir.y / 2;
    j1->position.z = middle.z - dir.z / 2;

    if (environmentFunction != 0 && TPE_LENGTH(TPE_vec3Minus(j1->position,
      environmentFunction(j1->position,TPE_JOINT_SIZE(*j1))))
      < TPE_JOINT_SIZE(*j1))
      j1->position = positionBackup;
  
    positionBackup = j2->position;

    j2->position.x = j1->position.x + dir.x;
    j2->position.y = j1->position.y + dir.y;
    j2->position.z = j1->position.z + dir.z; 

    if (environmentFunction != 0 && TPE_LENGTH(TPE_vec3Minus(j2->position,
      environmentFunction(j2->position,TPE_JOINT_SIZE(*j2))))
      < TPE_JOINT_SIZE(*j2))
      j2->position = positionBackup;
  }
}

TPE_Vec3 TPE_vec3Plus(TPE_Vec3 v1, TPE_Vec3 v2)
{
  v1.x += v2.x;
  v1.y += v2.y;
  v1.z += v2.z;

  return v1;
}

TPE_Vec3 TPE_vec3Minus(TPE_Vec3 v1, TPE_Vec3 v2)
{
  v1.x -= v2.x;
  v1.y -= v2.y;
  v1.z -= v2.z;

  return v1;
}

void TPE_vec3Normalize(TPE_Vec3 *v)
{
  TPE_Unit l = TPE_LENGTH(*v);

  if (l != 0)
  {
    v->x = (v->x * TPE_FRACTIONS_PER_UNIT) / l;
    v->y = (v->y * TPE_FRACTIONS_PER_UNIT) / l;
    v->z = (v->z * TPE_FRACTIONS_PER_UNIT) / l;
  }
}

TPE_Vec3 TPE_bodyGetCenter(const TPE_Body *body)
{
// TODO: take into account possibly different joint sizes? does it even matter?

  TPE_Vec3 result = TPE_vec3(0,0,0);

  const TPE_Joint *j = body->joints;

  for (uint16_t i = 0; i < body->jointCount; ++i)
  {
    result = TPE_vec3Plus(result,j->position);
    j++;
  }

  result.x /= body->jointCount;
  result.y /= body->jointCount;
  result.z /= body->jointCount;
 
  return result;
}

void TPE_bodySpin(TPE_Body *body, TPE_Vec3 rotation)
{
  TPE_Vec3 center = TPE_bodyGetCenter(body);

  for (uint16_t i = 0; i < body->jointCount; ++i)
  {
    TPE_Joint *j = body->joints + i;

    TPE_Vec3 toPoint = TPE_vec3Minus(j->position,center);

    toPoint = TPE_vec3Project(toPoint,rotation);
    toPoint = TPE_vec3Plus(center,toPoint);
    toPoint = TPE_vec3Minus(j->position,toPoint);
    toPoint = TPE_vec3Cross(toPoint,rotation);

    j->velocity[0] += toPoint.x;
    j->velocity[1] += toPoint.y;
    j->velocity[2] += toPoint.z;
  }
}

void TPE_bodyRotateByAxis(TPE_Body *body, TPE_Vec3 rotation)
{
  TPE_Vec3 bodyCenter = TPE_bodyGetCenter(body);
  TPE_Unit angle = TPE_LENGTH(rotation);

  TPE_vec3Normalize(&rotation);

  for (uint16_t i = 0; i < body->jointCount; ++i)
  {
    TPE_Joint *j = body->joints + i;

    TPE_Vec3 toPoint = TPE_vec3Minus(j->position,bodyCenter);

    toPoint = TPE_vec3Project(toPoint,rotation);

    TPE_Vec3 rotationCenter = TPE_vec3Plus(bodyCenter,toPoint);

    toPoint = TPE_vec3Minus(j->position,rotationCenter);

    TPE_Vec3 toPoint2 = TPE_vec3Cross(toPoint,rotation);

    j->position = TPE_vec3Plus(rotationCenter,
        TPE_vec3Plus(
          TPE_vec3Times(toPoint,TPE_cos(angle)),
          TPE_vec3Times(toPoint2,TPE_sin(angle))));
  }
}

TPE_Vec3 TPE_vec3Cross(TPE_Vec3 v1, TPE_Vec3 v2)
{
  TPE_Vec3 r;

  r.x = (v1.y * v2.z - v1.z * v2.y) / TPE_FRACTIONS_PER_UNIT;
  r.y = (v1.z * v2.x - v1.x * v2.z) / TPE_FRACTIONS_PER_UNIT;
  r.z = (v1.x * v2.y - v1.y * v2.x) / TPE_FRACTIONS_PER_UNIT;

  return r;
}

TPE_Vec3 TPE_vec3Project(TPE_Vec3 v, TPE_Vec3 base)
{
  TPE_Vec3 r;

  TPE_vec3Normalize(&base);

  TPE_Unit p = TPE_vec3Dot(v,base);

  r.x = (p * base.x) / TPE_FRACTIONS_PER_UNIT;
  r.y = (p * base.y) / TPE_FRACTIONS_PER_UNIT;
  r.z = (p * base.z) / TPE_FRACTIONS_PER_UNIT;
  
  return r;
}

void TPE_bodyMove(TPE_Body *body, TPE_Vec3 offset)
{
  for (uint16_t i = 0; i < body->jointCount; ++i)
    body->joints[i].position = TPE_vec3Plus(body->joints[i].position,
      offset);
}

void TPE_bodyAccelerate(TPE_Body *body, TPE_Vec3 velocity)
{
  TPE_bodyActivate(body);

  for (uint16_t i = 0; i < body->jointCount; ++i)
  {
    body->joints[i].velocity[0] += velocity.x;
    body->joints[i].velocity[1] += velocity.y;
    body->joints[i].velocity[2] += velocity.z;
  }
}

void TPE_bodyStop(TPE_Body *body)
{
  for (uint16_t i = 0; i < body->jointCount; ++i)
  {
    body->joints[i].velocity[0] = 0;
    body->joints[i].velocity[1] = 0;
    body->joints[i].velocity[2] = 0;
  }
}

void _TPE_bodyNonrotatingJointCollided(TPE_Body *b, int16_t jointIndex, 
  TPE_Vec3 origPos, uint8_t success)
{
  origPos = TPE_vec3Minus(b->joints[jointIndex].position,origPos);

  for (uint16_t i = 0; i < b->jointCount; ++i)
    if (i != jointIndex)
    {
      b->joints[i].position = TPE_vec3Plus(b->joints[i].position,origPos);
     
      if (success) 
        for (uint8_t j = 0; j < 3; ++j)
          b->joints[i].velocity[j] = b->joints[jointIndex].velocity[j];
    }
}

TPE_Unit TPE_vec3Dot(TPE_Vec3 v1, TPE_Vec3 v2)
{
  return (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z) / TPE_FRACTIONS_PER_UNIT;
}

TPE_Unit TPE_cos(TPE_Unit x)  
{
  return TPE_sin(x + TPE_FRACTIONS_PER_UNIT / 4);
}

TPE_Unit TPE_sin(TPE_Unit x)  
{
  int8_t sign = 1;
    
  if (x < 0) // odd function
  {
    x *= -1;
    sign = -1;
  }
    
  x %= TPE_FRACTIONS_PER_UNIT;
  
  if (x > TPE_FRACTIONS_PER_UNIT / 2)
  {
    x -= TPE_FRACTIONS_PER_UNIT / 2;
    sign *= -1;
  }

  TPE_Unit tmp = TPE_FRACTIONS_PER_UNIT - 2 * x;
 
  #define _PI2 ((TPE_Unit) (9.8696044 * TPE_FRACTIONS_PER_UNIT))
  return sign * // Bhaskara's approximation
    (((32 * x * _PI2) / TPE_FRACTIONS_PER_UNIT) * tmp) / 
    ((_PI2 * (5 * TPE_FRACTIONS_PER_UNIT - (8 * x * tmp) / 
      TPE_FRACTIONS_PER_UNIT)) / TPE_FRACTIONS_PER_UNIT);
  #undef _PI2
}

uint8_t TPE_bodiesResolveCollision(TPE_Body *b1, TPE_Body *b2)
{
  uint8_t r = 0;

  for (uint16_t i = 0; i < b1->jointCount; ++i)
    for (uint16_t j = 0; j < b2->jointCount; ++j)
    {
      TPE_Vec3 origPos2 = b2->joints[j].position;
      TPE_Vec3 origPos1 = b1->joints[i].position;

      if (TPE_jointsResolveCollision(&(b1->joints[i]),&(b2->joints[j]),
        b1->jointMass,b2->jointMass,(b1->elasticity + b2->elasticity) / 2,
        (b1->friction + b2->friction) / 2))
      {
        r = 1;

        if (b1->flags & TPE_BODY_FLAG_NONROTATING)
          _TPE_bodyNonrotatingJointCollided(b1,i,origPos1,1);

        if (b2->flags & TPE_BODY_FLAG_NONROTATING)
          _TPE_bodyNonrotatingJointCollided(b2,j,origPos2,1);
      }
    }

  return r;
}

uint8_t TPE_jointsResolveCollision(TPE_Joint *j1, TPE_Joint *j2,
  TPE_Unit mass1, TPE_Unit mass2, TPE_Unit elasticity, TPE_Unit friction)
{
  TPE_Vec3 dir = TPE_vec3Minus(j2->position,j1->position);

  TPE_Unit d = TPE_LENGTH(dir) - TPE_JOINT_SIZE(*j1) - TPE_JOINT_SIZE(*j2);

  if (d < 0) // collision?
  {
    // separate bodies, the shift distance will depend on the weight ratio:

    d = -1 * d + TPE_COLLISION_RESOLUTION_MARGIN;

    TPE_vec3Normalize(&dir);

    TPE_Unit ratio = (mass2 * TPE_FRACTIONS_PER_UNIT) / 
      TPE_nonZero(mass1 + mass2);

    TPE_Unit shiftDistance = (ratio * d) / TPE_FRACTIONS_PER_UNIT;

    TPE_Vec3 shift = TPE_vec3Times(dir,shiftDistance);

    j1->position = TPE_vec3Minus(j1->position,shift);

    shiftDistance = d - shiftDistance;

    shift = TPE_vec3Times(dir,shiftDistance);

    j2->position = TPE_vec3Plus(j2->position,shift);

    // compute new velocities:

    TPE_Unit v1, v2;

    TPE_Vec3 vel = TPE_vec3(j1->velocity[0],j1->velocity[1],j1->velocity[2]);

    vel = TPE_vec3Project(vel,dir);

    j1->velocity[0] = j1->velocity[0] - vel.x;
    j1->velocity[1] = j1->velocity[1] - vel.y;
    j1->velocity[2] = j1->velocity[2] - vel.z;

    /* friction explanation: Not physically correct (doesn't depend on load), 
    friction basically means we weighted average the velocities of the bodies
    in the direction perpendicular to the hit normal, in the ratio of their
    masses, friction coefficient just says how much of this effect we apply
    (it multiplies the friction vectors we are subtracting) */

    TPE_Vec3 frictionVec =
      TPE_vec3(j1->velocity[0],j1->velocity[1],j1->velocity[2]);

    v1 = TPE_vec3Dot(vel,dir);

    vel = TPE_vec3(j2->velocity[0],j2->velocity[1],j2->velocity[2]);

    vel = TPE_vec3Project(vel,dir);

    j2->velocity[0] = j2->velocity[0] - vel.x;
    j2->velocity[1] = j2->velocity[1] - vel.y;
    j2->velocity[2] = j2->velocity[2] - vel.z;

    frictionVec = TPE_vec3Minus(
      TPE_vec3(j2->velocity[0],j2->velocity[1],j2->velocity[2]),
      frictionVec);

    v2 = TPE_vec3Dot(vel,dir);

    TPE_getVelocitiesAfterCollision(&v1,&v2,mass1,mass2,elasticity);

    vel = TPE_vec3Times(dir,v1);

#define assignVec(j,i,d,o) \
  j->velocity[i] = j->velocity[i] + vel.d o (((frictionVec.d * ratio) / \
    TPE_FRACTIONS_PER_UNIT) * friction) / TPE_FRACTIONS_PER_UNIT;

    assignVec(j1,0,x,+)
    assignVec(j1,1,y,+)
    assignVec(j1,2,z,+)

    vel = TPE_vec3Times(dir,v2);

    ratio = TPE_FRACTIONS_PER_UNIT - ratio;

    assignVec(j2,0,x,-)
    assignVec(j2,1,y,-)
    assignVec(j2,2,z,-)

#undef assignVec

    return 1;
  }

  return 0;
}

TPE_Vec3 TPE_vec3Times(TPE_Vec3 v, TPE_Unit units)
{
  v.x = (v.x * units) / TPE_FRACTIONS_PER_UNIT;
  v.y = (v.y * units) / TPE_FRACTIONS_PER_UNIT;
  v.z = (v.z * units) / TPE_FRACTIONS_PER_UNIT;

  return v;
}

TPE_Vec3 TPE_vec3TimesNonNormalized(TPE_Vec3 v, TPE_Unit q)
{
  v.x *= q;
  v.y *= q;
  v.z *= q;

  return v;
}

void TPE_getVelocitiesAfterCollision(
  TPE_Unit *v1,
  TPE_Unit *v2,
  TPE_Unit m1,
  TPE_Unit m2,
  TPE_Unit elasticity
)
{
  /* In the following a lot of TPE_FRACTIONS_PER_UNIT cancel out, feel free to
     check if confused. */

  TPE_Unit m1Pm2 = TPE_nonZero(m1 + m2);
  TPE_Unit v2Mv1 = TPE_nonZero(*v2 - *v1);

  TPE_Unit m1v1Pm2v2 = ((m1 * *v1) + (m2 * *v2));

  *v1 = (((elasticity * m2 / TPE_FRACTIONS_PER_UNIT) * v2Mv1)
    + m1v1Pm2v2) / m1Pm2;

  *v2 = (((elasticity * m1 / TPE_FRACTIONS_PER_UNIT) * -1 * v2Mv1)
    + m1v1Pm2v2) / m1Pm2;
}

uint8_t TPE_jointEnvironmentResolveCollision(TPE_Joint *joint, TPE_Unit elasticity,
  TPE_Unit friction, TPE_ClosestPointFunction env)
{
  TPE_Vec3 toJoint = TPE_vec3Minus(joint->position,env(joint->position,TPE_JOINT_SIZE(*joint)));

  TPE_Unit len = TPE_LENGTH(toJoint);

  if (len <= TPE_JOINT_SIZE(*joint))
  {
    // colliding

    TPE_Vec3 positionBackup = joint->position, shift;
    uint8_t success = 0;

    if (len > 0)
    {
      /* Joint center is still outside the geometry so we can determine the
         normal and use it to shift it outside. This can still leave the joint
         colliding though, so try to repeat it a few times. */

      for (int i = 0; i < TPE_COLLISION_RESOLUTION_ITERATIONS; ++i)
      {
        shift = toJoint;

        TPE_vec3Normalize(&shift); 

        shift = TPE_vec3Times(shift,TPE_JOINT_SIZE(*joint) - len + 
          TPE_FRACTIONS_PER_UNIT / TPE_COLLISION_RESOLUTION_MARGIN);
          
        joint->position = TPE_vec3Plus(joint->position,shift);
  
        toJoint = TPE_vec3Minus(joint->position,env(joint->position,TPE_JOINT_SIZE(*joint)));

        len = TPE_LENGTH(toJoint); // still colliding?

        if (len >= TPE_JOINT_SIZE(*joint))
        {
          success = 1;
          break;
        }
      }
    }

    if (!success)
    {
      /* Shifting along normal was unsuccessfull, now try different approach:
         shift back by joint velocity. */

      shift = TPE_vec3(-1 * joint->velocity[0],-1 * joint->velocity[1],
        -1 * joint->velocity[2]);
      
      for (int i = 0; i < TPE_COLLISION_RESOLUTION_ITERATIONS; ++i)
      {
        joint->position = TPE_vec3Plus(joint->position,shift);

        toJoint = TPE_vec3Minus(joint->position,env(joint->position,TPE_JOINT_SIZE(*joint)));

        len = TPE_LENGTH(toJoint); // still colliding?

        if (len >= TPE_JOINT_SIZE(*joint))
        {
          success = 1;
          break;
        }

        shift.x /= 2; // decrease the step a bit
        shift.y /= 2;
        shift.z /= 2;
      }
    }
    if (success)
    {
      TPE_Vec3 vel = TPE_vec3(joint->velocity[0],joint->velocity[1],
        joint->velocity[2]);

      vel = TPE_vec3Project(vel,shift);

      TPE_Vec3 vel2 = TPE_vec3Minus(
        TPE_vec3(joint->velocity[0],joint->velocity[1],joint->velocity[2]),vel);

      vel2 = TPE_vec3Times(vel2,friction);

      vel = TPE_vec3Times(vel,TPE_FRACTIONS_PER_UNIT + elasticity);

      joint->velocity[0] -= vel.x + vel2.x;
      joint->velocity[1] -= vel.y + vel2.y;
      joint->velocity[2] -= vel.z + vel2.z;
    }
    else
    {
      TPE_LOG("WARNING: joint-environment collision couldn't be resolved");

      joint->position = positionBackup;
      joint->velocity[0] = 0;
      joint->velocity[1] = 0;
      joint->velocity[2] = 0;

      return 2;
    }

    return 1;
  }

  return 0;
}

uint8_t TPE_bodyEnvironmentCollide(const TPE_Body *body,
  TPE_ClosestPointFunction env)
{
  // TODO: should bounding vol check be here? maybe in param?

  for (uint16_t i = 0; i < body->jointCount; ++i)
  {
    const TPE_Joint *joint = body->joints + i;

    TPE_Unit size = TPE_JOINT_SIZE(*joint);

  if (TPE_DISTANCE(joint->position,env(joint->position,size)) <= size)
    return 1;
  }

  return 0;
 
}

uint8_t TPE_bodyEnvironmentResolveCollision(TPE_Body *body, 
  TPE_ClosestPointFunction env)
{
  /* Bounding sphere test first. NOTE: This is not a minimal bounding sphere
  but one created from the bounding box. Hopes are that this can be faster.
  TODO: actually test if the minimal B sphere is faster */

  TPE_Vec3 v1, v2;

  TPE_bodyGetAABB(body,&v1,&v2);

  v1.x = (v1.x + v2.x) / 2;
  v1.y = (v1.y + v2.y) / 2;
  v1.z = (v1.z + v2.z) / 2;

  TPE_Unit d = TPE_DISTANCE(v1,v2);

  if (TPE_DISTANCE(v1,env(v1,d)) > d)
    return 0;

  // now test the full body collision:

  uint8_t collision = 0;

  for (uint16_t i = 0; i < body->jointCount; ++i)
  {
    TPE_Vec3 previousPos = body->joints[i].position;

    uint8_t r = TPE_jointEnvironmentResolveCollision(
      body->joints + i,body->elasticity,body->friction,env);

    if (r)
    {
      collision = 1;

      if (body->flags & TPE_BODY_FLAG_NONROTATING)
        _TPE_bodyNonrotatingJointCollided(body,i,previousPos,r == 1);
    }
  }

  return collision;
}

TPE_Vec3 TPE_vec3Normalized(TPE_Vec3 v)
{
  TPE_vec3Normalize(&v);
  return v;
}

TPE_Unit TPE_atan(TPE_Unit x)
{
  /* atan approximation by polynomial 
     WARNING: this will break with different value of TPE_FRACTIONS_PER_UNIT */

  TPE_Unit sign = 1, x2 = x * x;

  if (x < 0)
  {
    x *= -1;
    sign = -1;
  }

  if (x > 30000) // anti overflow
    return sign * (TPE_FRACTIONS_PER_UNIT / 4);

  return sign *
    (307 * x + x2) / ((267026 + 633 * x + x2) / 128);
}

void _TPE_vec2Rotate(TPE_Unit *x, TPE_Unit *y, TPE_Unit angle)
{
  TPE_Unit tmp = *x;

  TPE_Unit s = TPE_sin(angle);
  TPE_Unit c = TPE_cos(angle);

  *x = (c * *x - s * *y) / TPE_FRACTIONS_PER_UNIT;
  *y = (s * tmp + c * *y) / TPE_FRACTIONS_PER_UNIT;
}

TPE_Unit _TPE_vec2Angle(TPE_Unit x, TPE_Unit y)
{
  TPE_Unit r = 0;

  if (x != 0)
  {
    r = TPE_atan((y * TPE_FRACTIONS_PER_UNIT) / x);

    if (x < 0)
      r += TPE_FRACTIONS_PER_UNIT / 2;
    else if (r < 0)
      r += TPE_FRACTIONS_PER_UNIT;
  }
  else
  {
    if (y < 0)
      r = (3 * TPE_FRACTIONS_PER_UNIT) / 4;
    else if (y > 0)
      r = TPE_FRACTIONS_PER_UNIT / 4;
    // else (y == 0) r stays 0
  }

  return r;
}

TPE_Vec3 TPE_orientationFromVecs(TPE_Vec3 forward, TPE_Vec3 right)
{
  TPE_Vec3 result;

  // get rotation around Y:

  result.y = _TPE_vec2Angle(forward.z,-1 * forward.x);

  // now rotate back by this angle to align with x = 0 plane:

  _TPE_vec2Rotate(&forward.z,&forward.x,result.y);
  _TPE_vec2Rotate(&right.z,&right.x,result.y);

  // now do the same for the second axis:

  result.x = 
    _TPE_vec2Angle(forward.z,forward.y);

  _TPE_vec2Rotate(&right.z,&right.y,-1 * result.x);

  result.z = _TPE_vec2Angle(right.x,-1 * right.y);

  return result;
}

TPE_Vec3 _TPE_project3DPoint(TPE_Vec3 p, TPE_Vec3 camPos, TPE_Vec3 camRot,
  TPE_Vec3 camView)
{
  // transform to camera space:

  p = TPE_vec3Minus(p,camPos);

  _TPE_vec2Rotate(&p.z,&p.x,camRot.y);
  _TPE_vec2Rotate(&p.z,&p.y,-1 * camRot.x);
  _TPE_vec2Rotate(&p.y,&p.x,-1 * camRot.z);

  if (p.z <= 0)
    return p;

  p.x = (p.x * camView.z) / p.z;
  p.y = (p.y * camView.z) / p.z;

  p.x = camView.x / 2 + (p.x * camView.x) / (2 * TPE_FRACTIONS_PER_UNIT);
  p.y = camView.y / 2 - (p.y * camView.x) / (2 * TPE_FRACTIONS_PER_UNIT);
                                    // ^ x here intentional
  return p;
}

void _TPE_drawDebugPixel(
  TPE_Unit x, TPE_Unit y, TPE_Unit w, TPE_Unit h, uint8_t c,
  TPE_DebugDrawFunction f)
{
  if (x >= 0 && x < w && y >= 0 && y < h)
    f(x,y,c);
}

void TPE_worldDebugDraw(TPE_World *world, TPE_DebugDrawFunction drawFunc,
  TPE_Vec3 camPos, TPE_Vec3 camRot, TPE_Vec3 camView, uint16_t envGridRes,
  TPE_Unit envGridSize)
{
  if (world->environmentFunction != 0)
  {
    // environment:

    TPE_Vec3 testPoint;

    TPE_Vec3 center = TPE_vec3(0,TPE_sin(camRot.x),TPE_cos(camRot.x));

    _TPE_vec2Rotate(&center.x,&center.z,camRot.y);

    TPE_Unit gridHalfSize = (envGridSize * envGridRes) / 2;

    center = TPE_vec3Times(center,gridHalfSize);
    center = TPE_vec3Plus(camPos,center);

    center.x = (center.x / envGridSize) * envGridSize;
    center.y = (center.y / envGridSize) * envGridSize;
    center.z = (center.z / envGridSize) * envGridSize;

    testPoint.y = center.y - gridHalfSize;

    for (uint8_t j = 0; j < envGridRes; ++j)
    {
      testPoint.x = center.x - gridHalfSize;

      for (uint8_t k = 0; k < envGridRes; ++k)
      {
        testPoint.z = center.z - gridHalfSize;

        for (uint8_t l = 0; l < envGridRes; ++l)
        {
          TPE_Vec3 r = world->environmentFunction(testPoint,envGridSize);

          if ((r.x != testPoint.x || r.y != testPoint.y || r.z != testPoint.z))
          {
// TODO: accel. by testing cheb dist first?
            r = _TPE_project3DPoint(r,camPos,camRot,camView);
        
            if (r.z > 0)
              _TPE_drawDebugPixel(r.x,r.y,camView.x,camView.y,2,drawFunc);
          }

          testPoint.z += envGridSize;
        }

        testPoint.x += envGridSize;
      }

      testPoint.y += envGridSize;
    }
  }

  for (uint16_t i = 0; i < world->bodyCount; ++i)
  {
    // connections:
    for (uint16_t j = 0; j < world->bodies[i].connectionCount; ++j)
    {
      TPE_Vec3
        p1 = world->bodies[i].joints[world->bodies[i].connections[j].joint1].position,
        p2 = world->bodies[i].joints[world->bodies[i].connections[j].joint2].position;

      p1 = _TPE_project3DPoint(p1,camPos,camRot,camView);
      p2 = _TPE_project3DPoint(p2,camPos,camRot,camView);

      if (p1.z <= 0 || p2.z <= 0)
        continue;

      TPE_Vec3 diff = TPE_vec3Minus(p2,p1);

#define SEGS 16
      for (uint16_t k = 0; k < SEGS; ++k)
      {
        p2.x = p1.x + (diff.x * k) / SEGS;
        p2.y = p1.y + (diff.y * k) / SEGS;

        if (p2.z > 0)
          _TPE_drawDebugPixel(p2.x,p2.y,camView.x,camView.y,0,drawFunc);
      }
#undef SEGS
    }

    // joints:
    for (uint16_t j = 0; j < world->bodies[i].jointCount; ++j)
    {
      TPE_Vec3 p = _TPE_project3DPoint(world->bodies[i].joints[j].position,
        camPos,camRot,camView);

      if (p.z > 0)
      {
        _TPE_drawDebugPixel(p.x,p.y,camView.x,camView.y,1,drawFunc);

        TPE_Unit size = TPE_JOINT_SIZE(world->bodies[i].joints[j]) / 2;
        size = (size * camView.x) / TPE_FRACTIONS_PER_UNIT;
        size = (size * camView.z) / p.z;

#define SEGS 4
        for (uint8_t k = 0; k < SEGS + 1; ++k)
        {
          TPE_Unit 
            dx = (TPE_sin(TPE_FRACTIONS_PER_UNIT * k / (8 * SEGS)) * size)
              / TPE_FRACTIONS_PER_UNIT,
            dy = (TPE_cos(TPE_FRACTIONS_PER_UNIT * k / (8 * SEGS)) * size)
              / TPE_FRACTIONS_PER_UNIT;

#define dp(a,b,c,d) \
  _TPE_drawDebugPixel(p.x a b,p.y c d,camView.x,camView.y,1,drawFunc);
          dp(+,dx,+,dy) dp(+,dx,-,dy) dp(-,dx,+,dy) dp(-,dx,-,dy)
          dp(+,dy,+,dx) dp(+,dy,-,dx) dp(-,dy,+,dx) dp(-,dy,-,dx)
#undef dp
#undef SEGS
        }
      }
    }
  }
}

TPE_Vec3 TPE_envBox(TPE_Vec3 point, TPE_Vec3 center, TPE_Vec3 maxCornerVec,
  TPE_Vec3 rotation)
{
  point = TPE_pointRotate(TPE_vec3Minus(point,center),
    TPE_rotationInverse(rotation));

  return TPE_vec3Plus(center,TPE_pointRotate(TPE_envAABox(point,TPE_vec3(0,0,0),
    maxCornerVec),rotation));
}

TPE_Vec3 TPE_envAABox(TPE_Vec3 point, TPE_Vec3 center, TPE_Vec3 maxCornerVec)
{
  TPE_Vec3 shifted = TPE_vec3Minus(point,center);
 
  int8_t sign[3] = {1, 1, 1};

  if (shifted.x < 0)
  {
    shifted.x *= -1;
    sign[0] = -1;
  }

  if (shifted.y < 0)
  {
    shifted.y *= -1;
    sign[1] = -1;
  }

  if (shifted.z < 0)
  {
    shifted.z *= -1;
    sign[2] = -1;
  }

  uint8_t region =
    (shifted.x > maxCornerVec.x) |
    ((shifted.y > maxCornerVec.y) << 1) |
    ((shifted.z > maxCornerVec.z) << 2);

  switch (region)
  {
#define align(c,i) point.c = center.c + sign[i] * maxCornerVec.c

    case 0x01: align(x,0); break;
    case 0x02: align(y,1); break;
    case 0x04: align(z,2); break;

    case 0x03: align(x,0); align(y,1); break;
    case 0x05: align(x,0); align(z,2); break;
    case 0x06: align(y,1); align(z,2); break;

    case 0x07: align(x,0); align(y,1); align(z,2); break; 
    default: break;

#undef align
  }

  return point;
}

TPE_Vec3 TPE_envAABoxInside(TPE_Vec3 point, TPE_Vec3 center, TPE_Vec3 size)
{
  size.x /= 2;
  size.y /= 2;
  size.z /= 2;

  TPE_Vec3 shifted = TPE_vec3Minus(point,center);

  TPE_Vec3 a = TPE_vec3Minus(size,shifted),
           b = TPE_vec3Plus(shifted,size);

  int8_t sx = 1, sy = 1, sz = 1;

  if (b.x < a.x)
  {
    a.x = b.x;
    sx = -1;
  }

  if (b.y < a.y)
  {
    a.y = b.y;
    sy = -1;
  }

  if (b.z < a.z)
  {
    a.z = b.z;
    sz = -1;
  }

  if (a.x < 0 || a.y < 0 || a.z < 0)
    return point;

  if (a.x < a.y)
  {
    if (a.x < a.z)
      point.x = center.x + sx * size.x;
    else
      point.z = center.z + sz * size.z;
  }
  else
  {
    if (a.y < a.z)
      point.y = center.y + sy * size.y;
    else
      point.z = center.z + sz * size.z;
  }

  return point;
}

TPE_Vec3 TPE_envSphere(TPE_Vec3 point, TPE_Vec3 center, TPE_Unit radius)
{
  // TODO: optim?

  TPE_Vec3 dir = TPE_vec3Minus(point,center);

  TPE_Unit l = TPE_LENGTH(dir);

  if (l <= radius)
    return point;

  dir.x = (dir.x * radius) / l;
  dir.y = (dir.y * radius) / l;
  dir.z = (dir.z * radius) / l;

  return TPE_vec3Plus(center,dir);
}

TPE_Vec3 TPE_envHalfPlane(TPE_Vec3 point, TPE_Vec3 center, TPE_Vec3 normal)
{
  TPE_Vec3 point2 = TPE_vec3Minus(point,center);

  TPE_Unit tmp = point2.x * normal.x + point2.y * normal.y + point2.z * normal.z;

  if (tmp < 0)
    return point;

  TPE_Unit l = TPE_LENGTH(normal);

  tmp /= l;

  normal.x = (normal.x * TPE_FRACTIONS_PER_UNIT) / l;
  normal.y = (normal.y * TPE_FRACTIONS_PER_UNIT) / l;
  normal.z = (normal.z * TPE_FRACTIONS_PER_UNIT) / l;

  return TPE_vec3Minus(point,
    TPE_vec3Times(normal,tmp));
}

uint8_t TPE_checkOverlapAABB(TPE_Vec3 v1Min, TPE_Vec3 v1Max, TPE_Vec3 v2Min,
  TPE_Vec3 v2Max)
{
  TPE_Unit dist;

#define test(c) \
  dist = v1Min.c + v1Max.c - v2Max.c - v2Min.c; \
  if (dist < 0) dist *= -1; \
  if (dist > v1Max.c - v1Min.c + v2Max.c - v2Min.c) return 0;

  test(x)
  test(y)
  test(z)

#undef test

  return 1;
}

void TPE_bodyGetAABB(const TPE_Body *body, TPE_Vec3 *vMin, TPE_Vec3 *vMax)
{
  *vMin = body->joints[0].position;
  *vMax = *vMin;

  TPE_Unit js = TPE_JOINT_SIZE(body->joints[0]);

  vMin->x -= js;
  vMin->y -= js;
  vMin->z -= js;

  vMax->x += js;
  vMax->y += js;
  vMax->z += js;

  for (uint16_t i = 1; i < body->jointCount; ++i)
  {
    TPE_Unit v;
  
    js = TPE_JOINT_SIZE(body->joints[i]);
  
#define test(c) \
  v = body->joints[i].position.c - js; \
  if (v < vMin->c) \
    vMin->c = v; \
  v += 2 * js; \
  if (v > vMax->c) \
    vMax->c = v;

    test(x)
    test(y)
    test(z)

#undef test
  }
}

void TPE_jointPin(TPE_Joint *joint, TPE_Vec3 position)
{
  joint->position = position;
  joint->velocity[0] = 0;
  joint->velocity[1] = 0;
  joint->velocity[2] = 0;
}

TPE_Vec3 TPE_pointRotate(TPE_Vec3 point, TPE_Vec3 rotation)
{
  _TPE_vec2Rotate(&point.y,&point.x,rotation.z);
  _TPE_vec2Rotate(&point.z,&point.y,rotation.x);
  _TPE_vec2Rotate(&point.x,&point.z,rotation.y);

  return point;
}

TPE_Vec3 TPE_rotationInverse(TPE_Vec3 rotation)
{
  /* If r1 = (X,Y,Z) is rotation in convention ABC then r1^-1 = (-X,-Y,-Z) in
     convention CBA is its inverse rotation. We exploit this, i.e. we rotate
     forward/right vectors in opposite axis order and then turn the result
     into normal rotation/orientation. */

  TPE_Vec3 f = TPE_vec3(0,0,TPE_FRACTIONS_PER_UNIT);
  TPE_Vec3 r = TPE_vec3(TPE_FRACTIONS_PER_UNIT,0,0);

  rotation.x *= -1;
  rotation.y *= -1;
  rotation.z *= -1;

  _TPE_vec2Rotate(&f.x,&f.z,rotation.y);
  _TPE_vec2Rotate(&f.z,&f.y,rotation.x);
  _TPE_vec2Rotate(&f.y,&f.x,rotation.z);

  _TPE_vec2Rotate(&r.x,&r.z,rotation.y);
  _TPE_vec2Rotate(&r.z,&r.y,rotation.x);
  _TPE_vec2Rotate(&r.y,&r.x,rotation.z);

  return TPE_orientationFromVecs(f,r);
}

#endif // guard
