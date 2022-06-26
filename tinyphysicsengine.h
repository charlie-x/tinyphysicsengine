#ifndef _TINYPHYSICSENGINE_H
#define _TINYPHYSICSENGINE_H

#include <stdint.h>

typedef int32_t TPE_Unit;

#define TPE_FRACTIONS_PER_UNIT 512

#define TPE_JOINT_SIZE_MULTIPLIER 32

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
  int16_t velocity[3]; ///< for saving space only uses int16
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
#define TPE_BODY_FLAG_DISABLED 3     /**< Disabled, not taking part in
                                          simulation. */
#define TPE_BODY_FLAG_SOFT 4         /**< Soft connections, effort won't be made
                                          to keep the body's shape. */

typedef TPE_Vec3 (*TPE_ClosestPointFunction)(TPE_Vec3);

typedef struct
{
  TPE_Joint *joints;
  uint8_t jointCount;
  TPE_Connection *connections;
  uint8_t connectionCount;
  uint16_t jointMass;
  uint8_t flags;
  uint8_t disableCount;
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

void TPE_getVelocitiesAfterCollision(
  TPE_Unit *v1,
  TPE_Unit *v2,
  TPE_Unit m1,
  TPE_Unit m2,
  TPE_Unit elasticity);

TPE_Vec3 TPE_vec3(TPE_Unit x, TPE_Unit y, TPE_Unit z);
TPE_Vec3 TPE_vec3Plus(TPE_Vec3 v1, TPE_Vec3 v2);
TPE_Vec3 TPE_vec3Minus(TPE_Vec3 v1, TPE_Vec3 v2);
TPE_Vec3 TPE_vec3Cross(TPE_Vec3 v1, TPE_Vec3 v2);
TPE_Vec3 TPE_vec3Project(TPE_Vec3 v, TPE_Vec3 base);
TPE_Vec3 TPE_vec3Times(TPE_Vec3 v, TPE_Unit units);
TPE_Vec3 TPE_vec3TimesNonNormalized(TPE_Vec3 v, TPE_Unit q);
TPE_Vec3 TPE_vec3Normalized(TPE_Vec3 v);

TPE_Unit TPE_vec3Dot(TPE_Vec3 v1, TPE_Vec3 v2);

TPE_Unit TPE_sqrt(TPE_Unit value);

TPE_Unit TPE_vec3Len(TPE_Vec3 v);
TPE_Unit TPE_vec3LenApprox(TPE_Vec3 v);

static inline TPE_Unit TPE_nonZero(TPE_Unit x);

static inline TPE_Unit TPE_dist(TPE_Vec3 p1, TPE_Vec3 p2);
static inline TPE_Unit TPE_distApprox(TPE_Vec3 p1, TPE_Vec3 p2);

TPE_Joint TPE_joint(TPE_Vec3 position, TPE_Unit size);

void TPE_jointsResolveCollision(TPE_Joint *j1, TPE_Joint *j2,
  TPE_Unit mass1, TPE_Unit mass2, TPE_Unit elasticity);

void TPE_jointEnvironmentResolveCollision(TPE_Joint *joint, TPE_Unit elasticity,
  TPE_Unit friction, TPE_ClosestPointFunction env);

void TPE_bodyEnvironmentResolveCollision(TPE_Body *body, TPE_Unit elasticity, 
  TPE_Unit friction, TPE_ClosestPointFunction env);

void TPE_bodiesResolveCollision(TPE_Body *b1, TPE_Body *b2);

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

//---------------------------

void TPE_worldStep(TPE_World *world);

TPE_Unit TPE_bodyNetSpeed(const TPE_Body *body);
TPE_Unit TPE_bodyAverageSpeed(const TPE_Body *body);

void TPE_bodyLimitNetSpeed(TPE_Body *body, TPE_Unit speedMin,
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

void TPE_bodyWake(TPE_Body *body);

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
void TPE_bodyRotate(TPE_Body *body, TPE_Vec3 rotation);

/** Compute the center of mass of a soft body. */
TPE_Vec3 TPE_bodyGetCenter(const TPE_Body *body);

TPE_Unit TPE_cos(TPE_Unit x);
TPE_Unit TPE_sin(TPE_Unit x);

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
  body->disableCount = 0;

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
        (i % 2) ? -1 * width : width,
        ((i >> 1) % 2) ? -1 * height : height,
        ((i >> 2) % 2) ? -1 * depth : depth),
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
      joint->position.x += joint->velocity[0];
      joint->position.y += joint->velocity[1];
      joint->position.z += joint->velocity[2];

      joint++;
    }

    TPE_Connection *connection = body->connections;
  
    TPE_bodyEnvironmentResolveCollision(body,256,256,
      world->environmentFunction);

    TPE_Unit bodyTension = 0;

    for (uint16_t j = 0; j < body->connectionCount; ++j) // update velocities
    {
      joint  = &(body->joints[connection->joint1]);
      joint2 = &(body->joints[connection->joint2]);

      TPE_Vec3 dir = TPE_vec3Minus(joint2->position,joint->position);

      TPE_Unit len = TPE_LENGTH(dir);

      len = (len * TPE_FRACTIONS_PER_UNIT) /
        connection->length - TPE_FRACTIONS_PER_UNIT;

bodyTension += len > 0 ? len : -len;


      if (  len > 5 || len < -5   ) //len != 0) // TODO: magic
      {

TPE_vec3Normalize(&dir);

dir.x /= 32;
dir.y /= 32;
dir.z /= 32;

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


/*
        dir.x = (dir.x * len) / (2 * TPE_FRACTIONS_PER_UNIT);
        dir.y = (dir.y * len) / (2 * TPE_FRACTIONS_PER_UNIT);
        dir.z = (dir.z * len) / (2 * TPE_FRACTIONS_PER_UNIT);

        joint->velocity[0] += dir.x;
        joint->velocity[1] += dir.y;
        joint->velocity[2] += dir.z;

        joint2->velocity[0] -= dir.x;
        joint2->velocity[1] -= dir.y;
        joint2->velocity[2] -= dir.z;
*/
      }

      connection++;
    }

    if (!(body->flags & TPE_BODY_FLAG_SOFT))
    {
      TPE_bodyReshape(body,world->environmentFunction);

      bodyTension /= body->connectionCount;
    
      if (bodyTension > 20)
      {
        TPE_bodyReshape(body,world->environmentFunction);
        TPE_bodyReshape(body,world->environmentFunction);
        TPE_bodyReshape(body,world->environmentFunction);
        TPE_bodyReshape(body,world->environmentFunction);
      }
    }

   printf("%d\n",body->disableCount); 

if (body->disableCount >= 64)
    {
      body->disableCount = 0;
      body->flags |= TPE_BODY_FLAG_DEACTIVATED;
    }
    else if (TPE_bodyAverageSpeed(body) <= 25) // TODO: magic + optimize
    {
      body->disableCount++;
    }
    else
      body->disableCount = 0;

  }
}

void TPE_bodyWake(TPE_Body *body)
{
  TPE_bodyStop(body);
  body->flags &= ~TPE_BODY_FLAG_DEACTIVATED;
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

void TPE_bodyLimitNetSpeed(TPE_Body *body, TPE_Unit speedMin,
  TPE_Unit speedMax)
{
for (uint8_t i = 0; i < 16; ++i)
{
  TPE_Unit speed = TPE_bodyNetSpeed(body);

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

if (environmentFunction != 0 &&
  TPE_LENGTH(TPE_vec3Minus(j1->position,environmentFunction(j1->position)))
  < TPE_JOINT_SIZE(*j1))
  j1->position = positionBackup;
  
positionBackup = j2->position;

    j2->position.x = j1->position.x + dir.x;
    j2->position.y = j1->position.y + dir.y;
    j2->position.z = j1->position.z + dir.z; 

if (environmentFunction != 0 &&
  TPE_LENGTH(TPE_vec3Minus(j2->position,environmentFunction(j2->position)))
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

  TPE_Unit anuglarV = TPE_LENGTH(rotation);

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

void TPE_bodyRotate(TPE_Body *body, TPE_Vec3 rotation)
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

    j->position = 
      TPE_vec3Plus(rotationCenter,
        TPE_vec3Plus(
          TPE_vec3Times(toPoint,TPE_cos(angle)),
          TPE_vec3Times(toPoint2,TPE_sin(angle))
        )
      );
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

void TPE_bodiesResolveCollision(TPE_Body *b1, TPE_Body *b2)
{
// TODO: bounding sphere (or AABB? maybe ifdef)

  for (uint16_t i = 0; i < b1->jointCount; ++i)
    for (uint16_t j = 0; j < b2->jointCount; ++j)
    {
      TPE_jointsResolveCollision( 
&(b1->joints[i]),
&(b2->joints[j]),
b1->jointMass,
b2->jointMass,
512
 );
    }
}

void TPE_jointsResolveCollision(TPE_Joint *j1, TPE_Joint *j2,
  TPE_Unit mass1, TPE_Unit mass2, TPE_Unit elasticity)
{
  TPE_Vec3 dir = TPE_vec3Minus(j2->position,j1->position);

  TPE_Unit d = TPE_LENGTH(dir) - TPE_JOINT_SIZE(*j1) - TPE_JOINT_SIZE(*j2);

  if (d < 0) // collision?
  {
    // separate bodies, the shift distance will depend on the weight ratio:

    d *= -1;

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

    v1 = TPE_vec3Dot(vel,dir);

    vel = TPE_vec3(j2->velocity[0],j2->velocity[1],j2->velocity[2]);

    vel = TPE_vec3Project(vel,dir);

    j2->velocity[0] = j2->velocity[0] - vel.x;
    j2->velocity[1] = j2->velocity[1] - vel.y;
    j2->velocity[2] = j2->velocity[2] - vel.z;

    v2 = TPE_vec3Dot(vel,dir);

    TPE_getVelocitiesAfterCollision(&v1,&v2,mass1,mass2,elasticity);

    vel = TPE_vec3Times(dir,v1);

    j1->velocity[0] = j1->velocity[0] + vel.x;
    j1->velocity[1] = j1->velocity[1] + vel.y;
    j1->velocity[2] = j1->velocity[2] + vel.z;

    vel = TPE_vec3Times(dir,v2);

    j2->velocity[0] = j2->velocity[0] + vel.x;
    j2->velocity[1] = j2->velocity[1] + vel.y;
    j2->velocity[2] = j2->velocity[2] + vel.z;
  }
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
  /* in the following a lot of TPE_FRACTIONS_PER_UNIT cancel out, feel free to
     check if confused */

  TPE_Unit m1Pm2 = TPE_nonZero(m1 + m2);
  TPE_Unit v2Mv1 = TPE_nonZero(*v2 - *v1);

  TPE_Unit m1v1Pm2v2 = ((m1 * *v1) + (m2 * *v2));

  *v1 = (((elasticity * m2 / TPE_FRACTIONS_PER_UNIT) * v2Mv1)
    + m1v1Pm2v2) / m1Pm2;

  *v2 = (((elasticity * m1 / TPE_FRACTIONS_PER_UNIT) * -1 * v2Mv1)
    + m1v1Pm2v2) / m1Pm2;
}

void TPE_jointEnvironmentResolveCollision(TPE_Joint *joint, TPE_Unit elasticity,
  TPE_Unit friction, TPE_ClosestPointFunction env)
{
  TPE_Vec3 toJoint = TPE_vec3Minus(joint->position,env(joint->position));

  TPE_Unit len = TPE_LENGTH(toJoint);

  if (len < TPE_JOINT_SIZE(*joint))
  {
    // colliding

    TPE_Vec3 positionBackup = joint->position, shift;
    uint8_t success = 0;

    if (len > 3) // TODO: magic constants
    {
      /* Joint center is still outside the geometry so we can determine the
         normal and use it to shift it outside. This can still leave the joint
         colliding though, so try to repeat it a few times. */

      for (int i = 0; i < 3; ++i)
      {
        shift = toJoint;
        TPE_vec3Normalize(&shift); 
        shift = TPE_vec3Times(shift,TPE_JOINT_SIZE(*joint) - len + 
          TPE_FRACTIONS_PER_UNIT / 64); // TODO: 128, magic val;
          
        joint->position = TPE_vec3Plus(joint->position,shift);
  
        toJoint = TPE_vec3Minus(joint->position,env(joint->position));

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
      
      for (int i = 0; i < 3; ++i)
      {
        joint->position = TPE_vec3Plus(joint->position,shift);

        toJoint = TPE_vec3Minus(joint->position,env(joint->position));

        len = TPE_LENGTH(toJoint); // still colliding?

        if (len >= TPE_JOINT_SIZE(*joint))
        {
          success = 1;
          break;
        }

        shift.x /= 8;
        shift.y /= 8;
        shift.z /= 8;
      }
    }
    if (success)
    {
      TPE_Vec3 vel = TPE_vec3(joint->velocity[0],joint->velocity[1],
        joint->velocity[2]);

      vel = TPE_vec3Project(vel,shift);

TPE_Vec3 vel2 = TPE_vec3Minus
(
  TPE_vec3(joint->velocity[0],joint->velocity[1],joint->velocity[2]),
  vel
);

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
    }
  }
}

void TPE_bodyEnvironmentResolveCollision(TPE_Body *body, TPE_Unit elasticity, 
  TPE_Unit friction, TPE_ClosestPointFunction env)
{
// TODO: bounding sphere

  for (uint16_t i = 0; i < body->jointCount; ++i)
    TPE_jointEnvironmentResolveCollision(
      body->joints + i,elasticity,friction,env);
}

TPE_Vec3 TPE_vec3Normalized(TPE_Vec3 v)
{
  TPE_vec3Normalize(&v);
  return v;
}

#endif // guard
