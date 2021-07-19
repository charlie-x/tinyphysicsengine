#ifndef TINYPHYSICSENGINE_H
#define TINYPHYSICSENGINE_H

/**
  author: Miloslav Ciz
  license: CC0 1.0 (public domain)
           found at https://creativecommons.org/publicdomain/zero/1.0/
           + additional waiver of all IP
  version: 0.1d

  This is a suckless library for simple 3D (and 2D) physics simulation. The
  physics is based on the Newtonian model but is further simplified,
  particularly in the area of rotation: there is no moment of inertia for
  objects, i.e. every object rotates as if it was a ball, and the object can be
  rotating around at most one axis at a time, i.e. it is not possible to
  simulate e.g. the Dzhanibekov effect.

  CONVENTIONS:

  - Compatibility and simple usage with small3dlib is intended, so most
    convention and data types copy those of small3dlib (which takes a lot of
    conventions of OpenGL).

  - No floating point is used, we instead use integers (effectively a fixed
    point). TPE_FRACTIONS_PER_UNIT is an equivalent to 1.0 in floating point and
    all numbers are normalized by this constant.

  - Units: for any measure only an abstract mathematical unit is used. This unit
    always has TPE_FRACTIONS_PER_UNIT parts. You can assign any correcpondence
    with real life units to these units. E.g. 1 spatial unit (which you can see
    as e.g. 1 meter) is equal to TPE_FRACTIONS_PER_UNIT. Same with temporatl
    (e.g. 1 second) and mass (e.g. 1 kilogram) units, and also any derived
    units, e.g. a unit of velocity (e.g. 1 m/s) is also equal to 1
    TPE_FRACTIONS_PER_UNIT. A full angle is also split into
    TPE_FRACTIONS_PER_UNIT parts (instead of 2 * PI or degrees).

  - Quaternions are represented as vec4 where x ~ i, y ~ j, z ~ k, w ~ real.
*/

#include <stdint.h>

typedef int32_t TPE_Unit;

/** How many fractions a unit is split into. This is NOT SUPPOSED TO BE
  REDEFINED, so rather don't do it (otherwise things may overflow etc.). */
#define TPE_FRACTIONS_PER_UNIT 512

#define TPE_INFINITY 2147483647

#define TPE_PI 1608              ///< pi in TPE_Units

#define TPE_SHAPE_POINT     0    ///< single point in space
#define TPE_SHAPE_SPHERE    1    ///< sphere, params.: radius
#define TPE_SHAPE_CUBOID    2    ///< cuboid, params.: width, height, depth
#define TPE_SHAPE_PLANE     3    ///< plane, params.: width, depth
#define TPE_SHAPE_CYLINDER  4    ///< cylinder, params.: radius, height
#define TPE_SHAPE_TRIMESH   5    /**< triangle mesh, params.:
                                        vertex count,
                                        triangle count
                                        vertices (int32_t pointer),
                                        indices (uint16_t pointer) */

#define TPE_MAX_SHAPE_PARAMS 3
#define TPE_MAX_SHAPE_PARAMPOINTERS 2

#define TPE_BODY_FLAG_DISABLED     0x00 ///< won't take part in simul. at all
#define TPE_BODY_FLAG_NONCOLLIDING 0x01 ///< simulated but won't collide

TPE_Unit TPE_wrap(TPE_Unit value, TPE_Unit mod);
TPE_Unit TPE_clamp(TPE_Unit v, TPE_Unit v1, TPE_Unit v2);
static inline TPE_Unit TPE_nonZero(TPE_Unit x);

/** Returns an integer square root of given value. */
TPE_Unit TPE_sqrt(TPE_Unit value);

/** Returns a sine of given arguments, both in TPE_Units (see the library
  conventions). */
TPE_Unit TPE_sin(TPE_Unit x);

TPE_Unit TPE_cos(TPE_Unit x);
TPE_Unit TPE_asin(TPE_Unit x);
TPE_Unit TPE_acos(TPE_Unit x);

typedef struct
{
  TPE_Unit x;
  TPE_Unit y;
  TPE_Unit z;
  TPE_Unit w;
} TPE_Vec4;

#define TPE_PRINTF_VEC4(v) printf("[%d %d %d %d] ",(v).x,(v).y,(v).z,(v).w);

/** Initializes vec4 to a zero vector. */
void TPE_initVec4(TPE_Vec4 *v);

TPE_Vec4 TPE_vec4(TPE_Unit x, TPE_Unit y, TPE_Unit z, TPE_Unit w);

void TPE_vec4Set(TPE_Vec4 *v, TPE_Unit x, TPE_Unit y, TPE_Unit z, TPE_Unit w);
void TPE_vec3Add(TPE_Vec4 a, TPE_Vec4 b, TPE_Vec4 *result);
void TPE_vec4Add(TPE_Vec4 a, TPE_Vec4 b, TPE_Vec4 *result);
void TPE_vec3Substract(TPE_Vec4 a, TPE_Vec4 b, TPE_Vec4 *result);
void TPE_vec3Average(TPE_Vec4 a, TPE_Vec4 b, TPE_Vec4 *result);
void TPE_vec4Substract(TPE_Vec4 a, TPE_Vec4 b, TPE_Vec4 *result);
void TPE_vec3Multiply(TPE_Vec4 v, TPE_Unit f, TPE_Vec4 *result);
void TPE_vec4Multiply(TPE_Vec4 v, TPE_Unit f, TPE_Vec4 *result);
TPE_Unit TPE_vec3Len(TPE_Vec4 v);
TPE_Unit TPE_vec4Len(TPE_Vec4 v);
TPE_Unit TPE_vec3DotProduct(TPE_Vec4 v1, TPE_Vec4 v2);
void TPE_vec3CrossProduct(TPE_Vec4 a, TPE_Vec4 b, TPE_Vec4 *result);
void TPE_vec3Normalize(TPE_Vec4 *v);
void TPE_vec4Normalize(TPE_Vec4 *v);
void TPE_vec3Project(TPE_Vec4 v, TPE_Vec4 base, TPE_Vec4 *result);

TPE_Vec4 TPE_vec3Plus(TPE_Vec4 a, TPE_Vec4 b);
TPE_Vec4 TPE_vec3Minus(TPE_Vec4 a, TPE_Vec4 b);
TPE_Vec4 TPE_vec3Times(TPE_Vec4 a, TPE_Vec4 b);
TPE_Vec4 TPE_vec3Cross(TPE_Vec4 a, TPE_Vec4 b);

/** Converts a linear velocity of an orbiting point to the angular velocity
  (angle units per time units). This depends on the distance of the point from
  the center of rotation. */
TPE_Unit TPE_linearVelocityToAngular(TPE_Unit velocity, TPE_Unit distance);

/** Holds a rotation state around a single axis, in a way that prevents rounding
  errors from distorting the rotation over time. In theory rotation of a body
  could be represented as 

  [current orientation, axis of rotation,angular velocity]

  However applying the rotation and normalizing the orientation quaternion each
  simulation step leads to error cumulation and the rotation gets aligned with
  one principal axis after some time. Because of this we rather represent the
  rotation state as

  [original orientation, axis of rotation, angular velocity, current angle]

  From this we can at each simulation step compute the current orientation by
  applying rotation by current angle to the original rotation without error
  cumulation. */
typedef struct
{
  TPE_Vec4 originalOrientation;  /**< quaternion holding the original 
                                   orientation of the body at the time when it
                                   has taken on this rotational state */
  TPE_Vec4 axisVelocity;         /**< axis of rotation (x,y,z) and a 
                                   non-negative angular velocity around this
                                   axis (w), determined ny the right hand
                                   rule */
  TPE_Unit currentAngle;         /**< angle the body has already rotated along
                                   the rotation axis (from the original 
                                   orientation)  */
} TPE_RotationState;

typedef struct
{
  uint8_t shape;

  TPE_Unit shapeParams[TPE_MAX_SHAPE_PARAMS];  ///< parameters of the body type
  void *shapeParamPointers[TPE_MAX_SHAPE_PARAMPOINTERS]; ///< pointer parameters

  uint8_t flags;

  TPE_Unit mass;            /**< body mass, setting this to TPE_INFINITY will
                                 make the object static (not moving at all) 
                                 which may help performance */

  TPE_Vec4 position;        ///< position of the body's center of mass

  TPE_Vec4 velocity;        ///< linear velocity vector

  TPE_RotationState rotation; /**< holds the state related to rotation, i.e.
                                 the rotation axis, angular momentum and data
                                 from which current orientation can be
                                 inferred */
} TPE_Body;

/** Initializes a physical body, this should be called on all TPE_Bodys that
  are created.*/
void TPE_bodyInit(TPE_Body *body);

/** Computes a 4x4 transform matrix of given body. The matrix has the same
  format as S3L_Mat4 from small3dlib. */
void TPE_bodyGetTransformMatrix(const TPE_Body *body, TPE_Unit matrix[4][4]);

void TPE_bodyGetOrientation(const TPE_Body *body, TPE_Vec4 *quaternion);

void TPE_bodyStep(TPE_Body *body);

void TPE_bodySetRotation(TPE_Body *body, TPE_Vec4 axis, TPE_Unit velocity);

void TPE_bodyAddRotation(TPE_Body *body, TPE_Vec4 axis, TPE_Unit velocity);

/** Applies a velocity change to a body at a specific point (relative to the
  body center), which will change its linear and/or angular velocity. This is
  similar to an impulse but doesn't take mass into account, only velocity. */
void TPE_bodyApplyVelocity(TPE_Body *body, TPE_Vec4 point, TPE_Vec4 velocity);

/** Collision detection: checks if two bodies are colliding. The return value is
  the collision depth along the collision normal (0 if the bodies are not
  colliding). World-space collision point is returned via a pointer. Collision
  normal is also returned via a pointer and its direction is "away from body1",
  i.e. if you move body1 in the opposite direction of this normal by the
  collision depth (return value), the bodies should no longer be colliding
  (in some cases another collision may still occur). */
TPE_Unit TPE_bodyCollides(const TPE_Body *body1, const TPE_Body *body2, 
  TPE_Vec4 *collisionPoint, TPE_Vec4 *collisionNormal);

/** Gets a velocity of a single point on a rigid body, taking into account its
  linear velocity and rotation. The point coordinates are relative to the body
  center. */
TPE_Vec4 TPE_bodyGetPointVelocity(const TPE_Body *body, TPE_Vec4 point);

void TPE_resolveCollision(TPE_Body *body1 ,TPE_Body *body2, 
  TPE_Vec4 collisionPoint, TPE_Vec4 collisionNormal);

/** Gets a uint16_t integer type of collision depending on two shapes, the order
  of shapes doesn't matter. */
#define TPE_COLLISION_TYPE(shape1,shape2) \
  ((shape1) <= (shape2) ? \
  (((uint16_t) (shape1)) << 8) | (shape2) : \
  (((uint16_t) (shape2)) << 8) | (shape1))

typedef struct
{
  uint16_t bodyCount;
  TPE_Body *bodies;
} TPE_PhysicsWorld;

/** Multiplies two quaternions which can be seen as chaining two rotations
  represented by them. This is not commutative (a*b != b*a)! Rotations a is
  performed firth, then rotation b is performed. */
void TPE_quaternionMultiply(TPE_Vec4 a, TPE_Vec4 b, TPE_Vec4 *result);

/** Initializes quaternion to the a rotation identity (i.e. NOT zero
  quaternion). */
void TPE_quaternionInit(TPE_Vec4 *quaternion);

/** Converts a rotation given as an axis and angle around this axis (by right
  hand rule) to a rotation quaternion. */
void TPE_rotationToQuaternion(TPE_Vec4 axis, TPE_Unit angle, 
  TPE_Vec4 *quaternion);

void TPE_quaternionToRotation(TPE_Vec4 quaternion, TPE_Vec4 *axis, 
  TPE_Unit *angle);

/** Converts a rotation quaternion to a 4x4 rotation matrix. The matrix is
  indexed as [column][row] and is in the same format as S3L_Mat4 from
  small3dlib. */
void TPE_quaternionToRotationMatrix(TPE_Vec4 quaternion, TPE_Unit matrix[4][4]);

void TPE_getVelocitiesAfterCollision(
  TPE_Unit *v1,
  TPE_Unit *v2,
  TPE_Unit m1,
  TPE_Unit m2,
  TPE_Unit elasticity
);

//------------------------------------------------------------------------------

void TPE_initVec4(TPE_Vec4 *v)
{
  v->x = 0;
  v->y = 0;
  v->z = 0;
  v->w = 0;
}

TPE_Vec4 TPE_vec4(TPE_Unit x, TPE_Unit y, TPE_Unit z, TPE_Unit w)
{
  TPE_Vec4 r;

  r.x = x;
  r.y = y;
  r.z = z;
  r.w = w;

  return r;
}

void TPE_vec4Set(TPE_Vec4 *v, TPE_Unit x, TPE_Unit y, TPE_Unit z, TPE_Unit w)
{
  v->x = x;
  v->y = y;
  v->z = z;
  v->w = w;
}

TPE_Unit TPE_wrap(TPE_Unit value, TPE_Unit mod)
{
  return value >= 0 ? (value % mod) : (mod + (value % mod) - 1);
}

TPE_Unit TPE_clamp(TPE_Unit v, TPE_Unit v1, TPE_Unit v2)
{
  return v >= v1 ? (v <= v2 ? v : v2) : v1;
}

TPE_Unit TPE_nonZero(TPE_Unit x)
{
  return x + (x == 0);
}

#define TPE_SIN_TABLE_LENGTH 128

static const TPE_Unit TPE_sinTable[TPE_SIN_TABLE_LENGTH] =
{
  /* 511 was chosen here as a highest number that doesn't overflow during
     compilation for TPE_FRACTIONS_PER_UNIT == 1024 */

  (0*TPE_FRACTIONS_PER_UNIT)/511, (6*TPE_FRACTIONS_PER_UNIT)/511, 
  (12*TPE_FRACTIONS_PER_UNIT)/511, (18*TPE_FRACTIONS_PER_UNIT)/511, 
  (25*TPE_FRACTIONS_PER_UNIT)/511, (31*TPE_FRACTIONS_PER_UNIT)/511, 
  (37*TPE_FRACTIONS_PER_UNIT)/511, (43*TPE_FRACTIONS_PER_UNIT)/511, 
  (50*TPE_FRACTIONS_PER_UNIT)/511, (56*TPE_FRACTIONS_PER_UNIT)/511, 
  (62*TPE_FRACTIONS_PER_UNIT)/511, (68*TPE_FRACTIONS_PER_UNIT)/511, 
  (74*TPE_FRACTIONS_PER_UNIT)/511, (81*TPE_FRACTIONS_PER_UNIT)/511, 
  (87*TPE_FRACTIONS_PER_UNIT)/511, (93*TPE_FRACTIONS_PER_UNIT)/511, 
  (99*TPE_FRACTIONS_PER_UNIT)/511, (105*TPE_FRACTIONS_PER_UNIT)/511, 
  (111*TPE_FRACTIONS_PER_UNIT)/511, (118*TPE_FRACTIONS_PER_UNIT)/511, 
  (124*TPE_FRACTIONS_PER_UNIT)/511, (130*TPE_FRACTIONS_PER_UNIT)/511, 
  (136*TPE_FRACTIONS_PER_UNIT)/511, (142*TPE_FRACTIONS_PER_UNIT)/511, 
  (148*TPE_FRACTIONS_PER_UNIT)/511, (154*TPE_FRACTIONS_PER_UNIT)/511, 
  (160*TPE_FRACTIONS_PER_UNIT)/511, (166*TPE_FRACTIONS_PER_UNIT)/511, 
  (172*TPE_FRACTIONS_PER_UNIT)/511, (178*TPE_FRACTIONS_PER_UNIT)/511, 
  (183*TPE_FRACTIONS_PER_UNIT)/511, (189*TPE_FRACTIONS_PER_UNIT)/511, 
  (195*TPE_FRACTIONS_PER_UNIT)/511, (201*TPE_FRACTIONS_PER_UNIT)/511, 
  (207*TPE_FRACTIONS_PER_UNIT)/511, (212*TPE_FRACTIONS_PER_UNIT)/511, 
  (218*TPE_FRACTIONS_PER_UNIT)/511, (224*TPE_FRACTIONS_PER_UNIT)/511, 
  (229*TPE_FRACTIONS_PER_UNIT)/511, (235*TPE_FRACTIONS_PER_UNIT)/511, 
  (240*TPE_FRACTIONS_PER_UNIT)/511, (246*TPE_FRACTIONS_PER_UNIT)/511, 
  (251*TPE_FRACTIONS_PER_UNIT)/511, (257*TPE_FRACTIONS_PER_UNIT)/511, 
  (262*TPE_FRACTIONS_PER_UNIT)/511, (268*TPE_FRACTIONS_PER_UNIT)/511, 
  (273*TPE_FRACTIONS_PER_UNIT)/511, (278*TPE_FRACTIONS_PER_UNIT)/511, 
  (283*TPE_FRACTIONS_PER_UNIT)/511, (289*TPE_FRACTIONS_PER_UNIT)/511, 
  (294*TPE_FRACTIONS_PER_UNIT)/511, (299*TPE_FRACTIONS_PER_UNIT)/511, 
  (304*TPE_FRACTIONS_PER_UNIT)/511, (309*TPE_FRACTIONS_PER_UNIT)/511, 
  (314*TPE_FRACTIONS_PER_UNIT)/511, (319*TPE_FRACTIONS_PER_UNIT)/511, 
  (324*TPE_FRACTIONS_PER_UNIT)/511, (328*TPE_FRACTIONS_PER_UNIT)/511, 
  (333*TPE_FRACTIONS_PER_UNIT)/511, (338*TPE_FRACTIONS_PER_UNIT)/511, 
  (343*TPE_FRACTIONS_PER_UNIT)/511, (347*TPE_FRACTIONS_PER_UNIT)/511, 
  (352*TPE_FRACTIONS_PER_UNIT)/511, (356*TPE_FRACTIONS_PER_UNIT)/511, 
  (361*TPE_FRACTIONS_PER_UNIT)/511, (365*TPE_FRACTIONS_PER_UNIT)/511, 
  (370*TPE_FRACTIONS_PER_UNIT)/511, (374*TPE_FRACTIONS_PER_UNIT)/511, 
  (378*TPE_FRACTIONS_PER_UNIT)/511, (382*TPE_FRACTIONS_PER_UNIT)/511, 
  (386*TPE_FRACTIONS_PER_UNIT)/511, (391*TPE_FRACTIONS_PER_UNIT)/511, 
  (395*TPE_FRACTIONS_PER_UNIT)/511, (398*TPE_FRACTIONS_PER_UNIT)/511, 
  (402*TPE_FRACTIONS_PER_UNIT)/511, (406*TPE_FRACTIONS_PER_UNIT)/511, 
  (410*TPE_FRACTIONS_PER_UNIT)/511, (414*TPE_FRACTIONS_PER_UNIT)/511, 
  (417*TPE_FRACTIONS_PER_UNIT)/511, (421*TPE_FRACTIONS_PER_UNIT)/511, 
  (424*TPE_FRACTIONS_PER_UNIT)/511, (428*TPE_FRACTIONS_PER_UNIT)/511, 
  (431*TPE_FRACTIONS_PER_UNIT)/511, (435*TPE_FRACTIONS_PER_UNIT)/511, 
  (438*TPE_FRACTIONS_PER_UNIT)/511, (441*TPE_FRACTIONS_PER_UNIT)/511, 
  (444*TPE_FRACTIONS_PER_UNIT)/511, (447*TPE_FRACTIONS_PER_UNIT)/511, 
  (450*TPE_FRACTIONS_PER_UNIT)/511, (453*TPE_FRACTIONS_PER_UNIT)/511, 
  (456*TPE_FRACTIONS_PER_UNIT)/511, (459*TPE_FRACTIONS_PER_UNIT)/511, 
  (461*TPE_FRACTIONS_PER_UNIT)/511, (464*TPE_FRACTIONS_PER_UNIT)/511, 
  (467*TPE_FRACTIONS_PER_UNIT)/511, (469*TPE_FRACTIONS_PER_UNIT)/511, 
  (472*TPE_FRACTIONS_PER_UNIT)/511, (474*TPE_FRACTIONS_PER_UNIT)/511, 
  (476*TPE_FRACTIONS_PER_UNIT)/511, (478*TPE_FRACTIONS_PER_UNIT)/511, 
  (481*TPE_FRACTIONS_PER_UNIT)/511, (483*TPE_FRACTIONS_PER_UNIT)/511, 
  (485*TPE_FRACTIONS_PER_UNIT)/511, (487*TPE_FRACTIONS_PER_UNIT)/511, 
  (488*TPE_FRACTIONS_PER_UNIT)/511, (490*TPE_FRACTIONS_PER_UNIT)/511, 
  (492*TPE_FRACTIONS_PER_UNIT)/511, (494*TPE_FRACTIONS_PER_UNIT)/511, 
  (495*TPE_FRACTIONS_PER_UNIT)/511, (497*TPE_FRACTIONS_PER_UNIT)/511, 
  (498*TPE_FRACTIONS_PER_UNIT)/511, (499*TPE_FRACTIONS_PER_UNIT)/511, 
  (501*TPE_FRACTIONS_PER_UNIT)/511, (502*TPE_FRACTIONS_PER_UNIT)/511, 
  (503*TPE_FRACTIONS_PER_UNIT)/511, (504*TPE_FRACTIONS_PER_UNIT)/511, 
  (505*TPE_FRACTIONS_PER_UNIT)/511, (506*TPE_FRACTIONS_PER_UNIT)/511, 
  (507*TPE_FRACTIONS_PER_UNIT)/511, (507*TPE_FRACTIONS_PER_UNIT)/511, 
  (508*TPE_FRACTIONS_PER_UNIT)/511, (509*TPE_FRACTIONS_PER_UNIT)/511, 
  (509*TPE_FRACTIONS_PER_UNIT)/511, (510*TPE_FRACTIONS_PER_UNIT)/511, 
  (510*TPE_FRACTIONS_PER_UNIT)/511, (510*TPE_FRACTIONS_PER_UNIT)/511, 
  (510*TPE_FRACTIONS_PER_UNIT)/511, (510*TPE_FRACTIONS_PER_UNIT)/511
};

#define TPE_SIN_TABLE_UNIT_STEP\
  (TPE_FRACTIONS_PER_UNIT / (TPE_SIN_TABLE_LENGTH * 4))

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

TPE_Unit TPE_sin(TPE_Unit x)
{
  x = TPE_wrap(x / TPE_SIN_TABLE_UNIT_STEP,TPE_SIN_TABLE_LENGTH * 4);
  int8_t positive = 1;

  if (x < TPE_SIN_TABLE_LENGTH)
  {
  }
  else if (x < TPE_SIN_TABLE_LENGTH * 2)
  {
    x = TPE_SIN_TABLE_LENGTH * 2 - x - 1;
  }
  else if (x < TPE_SIN_TABLE_LENGTH * 3)
  {
    x = x - TPE_SIN_TABLE_LENGTH * 2;
    positive = 0;
  }
  else
  {
    x = TPE_SIN_TABLE_LENGTH - (x - TPE_SIN_TABLE_LENGTH * 3) - 1;
    positive = 0;
  }

  return positive ? TPE_sinTable[x] : -1 * TPE_sinTable[x];
}

TPE_Unit TPE_cos(TPE_Unit x)
{
  return TPE_sin(x + TPE_FRACTIONS_PER_UNIT / 4);
}

void TPE_bodyInit(TPE_Body *body)
{
  // TODO

  TPE_initVec4(&(body->position));
  TPE_initVec4(&(body->velocity));

  // init orientation to identity unit quaternion (1,0,0,0):

  TPE_quaternionInit(&(body->rotation.originalOrientation));
  TPE_vec4Set(&(body->rotation.axisVelocity),TPE_FRACTIONS_PER_UNIT,0,0,0);
  body->rotation.currentAngle = 0;
}

void TPE_bodyGetOrientation(const TPE_Body *body, TPE_Vec4 *quaternion)
{
  TPE_Vec4 axisRotation;

  TPE_rotationToQuaternion(
    body->rotation.axisVelocity,
    body->rotation.currentAngle,
    &axisRotation);

  TPE_quaternionMultiply(
    body->rotation.originalOrientation,
    axisRotation,
    quaternion);

  TPE_vec4Normalize(quaternion);
}

void TPE_vec3CrossProduct(TPE_Vec4 a, TPE_Vec4 b, TPE_Vec4 *result)
{
  TPE_Vec4 r;

  r.x = a.y * b.z - a.z * b.y;
  r.y = a.z * b.x - a.x * b.z;
  r.z = a.x * b.y - a.y * b.x;

  *result = r;
}

TPE_Vec4 TPE_vec3Cross(TPE_Vec4 a, TPE_Vec4 b)
{
  TPE_vec3CrossProduct(a,b,&a);
  return a;
}

void TPE_bodyApplyVelocity(TPE_Body *body, TPE_Vec4 point, TPE_Vec4 velocity)
{  
  TPE_Vec4 linearVelocity, angularVelocity, rotationAxis;
    
  TPE_vec3Add(body->velocity,velocity,&(body->velocity));

  TPE_Unit pointDistance = TPE_vec3Len(point);

  if (pointDistance != 0)  
  {
    /* normalize the point, we don't use the function as we don't want to    
       recompute the vector length */

    point.x = (point.x * TPE_FRACTIONS_PER_UNIT) / pointDistance;
    point.y = (point.y * TPE_FRACTIONS_PER_UNIT) / pointDistance;
    point.z = (point.z * TPE_FRACTIONS_PER_UNIT) / pointDistance;

    /* Now we take only a part of the applied velocity, the part projected
       to a plane perpendicular to the point vector, and this part will
       contribute to the body rotation. */

    TPE_Vec4 tmp;

    TPE_vec3Project(velocity,point,&tmp);
    TPE_vec3Substract(velocity,tmp,&angularVelocity);

    TPE_vec3CrossProduct(point,angularVelocity,&rotationAxis);

    TPE_bodyAddRotation(body,rotationAxis,
      TPE_linearVelocityToAngular(
        TPE_vec3Len(angularVelocity),-1 * pointDistance));
  }
}

void _TPE_getShapes(const TPE_Body *b1, const TPE_Body *b2, uint8_t shape1,
  const TPE_Body **first, const TPE_Body **second)
{
  if (b1->shape == shape1)
  {
    *first = b1;
    *second = b2;
  }
  else
  {
    *first = b2;
    *second = b1;
  }
}

TPE_Unit TPE_bodyCollides(const TPE_Body *body1, const TPE_Body *body2, 
  TPE_Vec4 *collisionPoint, TPE_Vec4 *collisionNormal)
{
  // now check the actual collisions:

  switch (TPE_COLLISION_TYPE(body1->shape,body2->shape))
  {
    case TPE_COLLISION_TYPE(TPE_SHAPE_SPHERE,TPE_SHAPE_SPHERE):
    {
      TPE_Vec4 distanceVec;
      TPE_vec3Substract(body2->position,body1->position,&distanceVec);
      TPE_Unit distance = TPE_vec3Len(distanceVec);

      distance -= body1->shapeParams[0] + body2->shapeParams[0];

      if (distance < 0)
      {
        TPE_vec3Average(body1->position,body2->position,collisionPoint);
        *collisionNormal = distanceVec;
        TPE_vec3Normalize(collisionNormal);

        return -1 * distance;
      } 

      break;
    } 

    case TPE_COLLISION_TYPE(TPE_SHAPE_SPHERE,TPE_SHAPE_CUBOID):
    {
      const TPE_Body *sphere;
      const TPE_Body *cuboid;

      _TPE_getShapes(body1,body2,TPE_SHAPE_SPHERE,&sphere,&cuboid);

      break;
    } 

    default:
      break;
  }
  
  return 0;
}

TPE_Vec4 TPE_bodyGetPointVelocity(const TPE_Body *body, TPE_Vec4 point)
{
  TPE_Vec4 result = body->velocity;

 



  // point-line distance (distance of the point from rotation axis):

  TPE_Unit dist =
    (
    TPE_vec3Len(  
      TPE_vec3Cross(
        point,
        TPE_vec3Minus(point,body->rotation.axisVelocity)
      )
    ) * TPE_FRACTIONS_PER_UNIT)

    /

    TPE_vec3Len(body->rotation.axisVelocity)
    ;


  return result;
}

void TPE_resolveCollision(TPE_Body *body1 ,TPE_Body *body2, 
  TPE_Vec4 collisionPoint, TPE_Vec4 collisionNormal)
{
}

TPE_Unit TPE_linearVelocityToAngular(TPE_Unit velocity, TPE_Unit distance)
{
  TPE_Unit circumfence = (2 * TPE_PI * distance) / TPE_FRACTIONS_PER_UNIT;
  return (velocity * TPE_FRACTIONS_PER_UNIT) / circumfence;
}

void TPE_bodyStep(TPE_Body *body)
{
  TPE_vec3Add(body->position,body->velocity,&(body->position));
  body->rotation.currentAngle += body->rotation.axisVelocity.w;
}

void TPE_bodySetRotation(TPE_Body *body, TPE_Vec4 axis, TPE_Unit velocity)
{
  TPE_bodyGetOrientation(body,&(body->rotation.originalOrientation));

  if (velocity < 0)
  {
    axis.x *= -1;
    axis.y *= -1;
    axis.z *= -1;
    velocity *= -1;
  }

  TPE_vec3Normalize(&axis);

  body->rotation.axisVelocity = axis;
  body->rotation.axisVelocity.w = velocity;
  body->rotation.currentAngle = 0;
}

void TPE_bodyAddRotation(TPE_Body *body, TPE_Vec4 axis, TPE_Unit velocity)
{
  /* Rotation is added like this: we convert both the original and added
     rotation to vectors whose direction is along the rotations axis and
     magnitude is the rotation speed, then we add these vectors and convert
     the final vector back to normalized rotation axis + scalar rotation 
     speed.*/

  body->rotation.axisVelocity.x = 
    (body->rotation.axisVelocity.x * body->rotation.axisVelocity.w)
    / TPE_FRACTIONS_PER_UNIT;

  body->rotation.axisVelocity.y = 
    (body->rotation.axisVelocity.y * body->rotation.axisVelocity.w)
    / TPE_FRACTIONS_PER_UNIT;

  body->rotation.axisVelocity.z = 
    (body->rotation.axisVelocity.z * body->rotation.axisVelocity.w)
    / TPE_FRACTIONS_PER_UNIT;

  TPE_vec3Normalize(&axis);

  axis.x = (axis.x * velocity) / TPE_FRACTIONS_PER_UNIT;
  axis.y = (axis.y * velocity) / TPE_FRACTIONS_PER_UNIT;
  axis.z = (axis.z * velocity) / TPE_FRACTIONS_PER_UNIT;

  TPE_vec3Add(body->rotation.axisVelocity,axis,&axis);

  axis.w = TPE_vec3Len(axis);

  TPE_bodySetRotation(body,axis,axis.w);
}

void TPE_quaternionMultiply(TPE_Vec4 a, TPE_Vec4 b, TPE_Vec4 *result)
{
  TPE_Vec4 r; // in case result is identical to a or b

  r.x =
    (a.w * b.x +
     a.x * b.w +
     a.y * b.z -
     a.z * b.y) / TPE_FRACTIONS_PER_UNIT;

  r.y =
    (a.w * b.y -
     a.x * b.z +
     a.y * b.w +
     a.z * b.x) / TPE_FRACTIONS_PER_UNIT;

  r.z =
    (a.w * b.z +
     a.x * b.y -
     a.y * b.x +
     a.z * b.w) / TPE_FRACTIONS_PER_UNIT;

  r.w =
    (a.w * b.w -
     a.x * b.x -
     a.y * b.y -
     a.z * b.z) / TPE_FRACTIONS_PER_UNIT;

  result->x = r.x;
  result->y = r.y;
  result->z = r.z;
  result->w = r.w;
}

void TPE_rotationToQuaternion(TPE_Vec4 axis, TPE_Unit angle, TPE_Vec4 *quaternion)
{
  TPE_vec3Normalize(&axis);

  angle /= 2;

  TPE_Unit s = TPE_sin(angle);

  quaternion->x = (s * axis.x) / TPE_FRACTIONS_PER_UNIT;
  quaternion->y = (s * axis.y) / TPE_FRACTIONS_PER_UNIT;
  quaternion->z = (s * axis.z) / TPE_FRACTIONS_PER_UNIT;

  quaternion->w = TPE_cos(angle);
}

TPE_Unit TPE_asin(TPE_Unit x)
{
  x = TPE_clamp(x,-TPE_FRACTIONS_PER_UNIT,TPE_FRACTIONS_PER_UNIT);

  int8_t sign = 1;

  if (x < 0)
  {
    sign = -1;
    x *= -1;
  }

  int16_t low = 0;
  int16_t high = TPE_SIN_TABLE_LENGTH -1;
  int16_t middle;

  while (low <= high) // binary search
  {
    middle = (low + high) / 2;

    TPE_Unit v = TPE_sinTable[middle];

    if (v > x)
      high = middle - 1;
    else if (v < x)
      low = middle + 1;
    else
      break;
  }

  middle *= TPE_SIN_TABLE_UNIT_STEP;

  return sign * middle;
}

TPE_Unit TPE_acos(TPE_Unit x)
{
  return TPE_asin(-1 * x) + TPE_FRACTIONS_PER_UNIT / 4;
}

void TPE_quaternionToRotation(TPE_Vec4 quaternion, TPE_Vec4 *axis, TPE_Unit *angle)
{
  *angle = 2 * TPE_acos(quaternion.x);

  TPE_Unit tmp =
    TPE_nonZero(TPE_sqrt(
      (TPE_FRACTIONS_PER_UNIT - 
        (quaternion.x * quaternion.x) / TPE_FRACTIONS_PER_UNIT
      ) * TPE_FRACTIONS_PER_UNIT));

  axis->x = (quaternion.x * TPE_FRACTIONS_PER_UNIT) / tmp;
  axis->y = (quaternion.y * TPE_FRACTIONS_PER_UNIT) / tmp;
  axis->z = (quaternion.z * TPE_FRACTIONS_PER_UNIT) / tmp;
}

void TPE_quaternionToRotationMatrix(TPE_Vec4 quaternion, TPE_Unit matrix[4][4])
{
  TPE_Unit 
    _2x2 = (2 * quaternion.x * quaternion.x) / TPE_FRACTIONS_PER_UNIT,
    _2y2 = (2 * quaternion.y * quaternion.y) / TPE_FRACTIONS_PER_UNIT,
    _2z2 = (2 * quaternion.z * quaternion.z) / TPE_FRACTIONS_PER_UNIT,
    _2xy = (2 * quaternion.x * quaternion.y) / TPE_FRACTIONS_PER_UNIT,
    _2xw = (2 * quaternion.x * quaternion.w) / TPE_FRACTIONS_PER_UNIT,
    _2zw = (2 * quaternion.z * quaternion.w) / TPE_FRACTIONS_PER_UNIT,
    _2xz = (2 * quaternion.x * quaternion.z) / TPE_FRACTIONS_PER_UNIT,
    _2yw = (2 * quaternion.y * quaternion.w) / TPE_FRACTIONS_PER_UNIT,
    _2yz = (2 * quaternion.y * quaternion.z) / TPE_FRACTIONS_PER_UNIT;

  #define ONE TPE_FRACTIONS_PER_UNIT

  matrix[0][0]  = ONE - _2y2 - _2z2;
  matrix[1][0]  = _2xy - _2zw;
  matrix[2][0]  = _2xz + _2yw;
  matrix[3][0]  = 0;

  matrix[0][1]  = _2xy + _2zw;
  matrix[1][1]  = ONE - _2x2 - _2z2;
  matrix[2][1]  = _2yz - _2xw;
  matrix[3][1]  = 0;

  matrix[0][2]  = _2xz - _2yw;
  matrix[1][2]  = _2yz + _2xw;
  matrix[2][2] = ONE - _2x2 - _2y2;
  matrix[3][2] = 0;

  matrix[0][3] = 0;
  matrix[1][3] = 0;
  matrix[2][3] = 0;
  matrix[3][3] = ONE;

  #undef ONE
}

void TPE_vec3Add(const TPE_Vec4 a, const TPE_Vec4 b, TPE_Vec4 *result)
{
  result->x = a.x + b.x;
  result->y = a.y + b.y;
  result->z = a.z + b.z;
}

void TPE_vec4Add(const TPE_Vec4 a, const TPE_Vec4 b, TPE_Vec4 *result)
{
  result->x = a.x + b.x;
  result->y = a.y + b.y;
  result->z = a.z + b.z;
  result->w = a.w + b.w;
}

void TPE_vec3Substract(const TPE_Vec4 a, const TPE_Vec4 b, TPE_Vec4 *result)
{
  result->x = a.x - b.x;
  result->y = a.y - b.y;
  result->z = a.z - b.z;
}

TPE_Vec4 TPE_vec3Plus(TPE_Vec4 a, TPE_Vec4 b)
{
  a.x += b.x;
  a.y += b.y;
  a.z += a.z;

  return a;
}

TPE_Vec4 TPE_vec3Minus(TPE_Vec4 a, TPE_Vec4 b)
{
  a.x -= b.x;
  a.y -= b.y;
  a.z -= a.z;

  return a;
}

TPE_Vec4 TPE_vec3Times(TPE_Vec4 a, TPE_Vec4 b)
{
  a.x = (a.x * b.x) / TPE_FRACTIONS_PER_UNIT;
  a.y = (a.y * b.y) / TPE_FRACTIONS_PER_UNIT;
  a.z = (a.z * a.z) / TPE_FRACTIONS_PER_UNIT;

  return a;
}

void TPE_vec3Average(TPE_Vec4 a, TPE_Vec4 b, TPE_Vec4 *result)
{
  result->x = (a.x + b.x) / 2;
  result->y = (a.y + b.y) / 2;
  result->z = (a.z + b.z) / 2;
}

void TPE_vec4Substract(const TPE_Vec4 a, const TPE_Vec4 b, TPE_Vec4 *result)
{
  result->x = a.x - b.x;
  result->y = a.y - b.y;
  result->z = a.z - b.z;
  result->w = a.w - b.w;
}

void TPE_vec3Multiply(const TPE_Vec4 v, TPE_Unit f, TPE_Vec4 *result)
{
  result->x = (v.x * f) / TPE_FRACTIONS_PER_UNIT;
  result->y = (v.y * f) / TPE_FRACTIONS_PER_UNIT;
  result->z = (v.z * f) / TPE_FRACTIONS_PER_UNIT;
}

void TPE_vec4Multiply(const TPE_Vec4 v, TPE_Unit f, TPE_Vec4 *result)
{
  result->x = (v.x * f) / TPE_FRACTIONS_PER_UNIT;
  result->y = (v.y * f) / TPE_FRACTIONS_PER_UNIT;
  result->z = (v.z * f) / TPE_FRACTIONS_PER_UNIT;
  result->w = (v.w * f) / TPE_FRACTIONS_PER_UNIT;
}

TPE_Unit TPE_vec3Len(TPE_Vec4 v)
{
  return TPE_sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

TPE_Unit TPE_vec4Len(TPE_Vec4 v)
{
  return TPE_sqrt(v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w);
}

TPE_Unit TPE_vec3DotProduct(const TPE_Vec4 v1, const TPE_Vec4 v2)
{
  return
    (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z) / TPE_FRACTIONS_PER_UNIT;
}

void TPE_vec3Normalize(TPE_Vec4 *v)
{ 
  TPE_Unit l = TPE_vec3Len(*v);

  if (l == 0)
  {
    v->x = TPE_FRACTIONS_PER_UNIT;
    return;
  }

  v->x = (v->x * TPE_FRACTIONS_PER_UNIT) / l;
  v->y = (v->y * TPE_FRACTIONS_PER_UNIT) / l;
  v->z = (v->z * TPE_FRACTIONS_PER_UNIT) / l;
}

void TPE_vec4Normalize(TPE_Vec4 *v)
{ 
  TPE_Unit l = TPE_vec4Len(*v);

  if (l == 0)
  {
    v->x = TPE_FRACTIONS_PER_UNIT;
    return;
  }

  v->x = (v->x * TPE_FRACTIONS_PER_UNIT) / l;
  v->y = (v->y * TPE_FRACTIONS_PER_UNIT) / l;
  v->z = (v->z * TPE_FRACTIONS_PER_UNIT) / l;
  v->w = (v->w * TPE_FRACTIONS_PER_UNIT) / l;
}

void TPE_vec3Project(TPE_Vec4 v, TPE_Vec4 base, TPE_Vec4 *result)
{
  TPE_Unit p = TPE_vec3DotProduct(v,base);

  result->x = (p * base.x) / TPE_FRACTIONS_PER_UNIT;
  result->y = (p * base.y) / TPE_FRACTIONS_PER_UNIT;
  result->z = (p * base.z) / TPE_FRACTIONS_PER_UNIT;
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

  #define ANTI_OVERFLOW 30000
  #define ANTI_OVERFLOW_SCALE 128

  uint8_t overflowDanger = m1 > ANTI_OVERFLOW || *v1 > ANTI_OVERFLOW ||
    m2 > ANTI_OVERFLOW || *v2 > ANTI_OVERFLOW;

  if (overflowDanger)
  {
    m1 = (m1 != 0) ? TPE_nonZero(m1 / ANTI_OVERFLOW_SCALE) : 0;
    m2 = (m2 != 0) ? TPE_nonZero(m2 / ANTI_OVERFLOW_SCALE) : 0;
    *v1 = (*v1 != 0) ? TPE_nonZero(*v1 / ANTI_OVERFLOW_SCALE) : 0;
    *v2 = (*v2 != 0) ? TPE_nonZero(*v2 / ANTI_OVERFLOW_SCALE) : 0;
  }

  TPE_Unit m1Pm2 = m1 + m2;
  TPE_Unit v2Mv1 = *v2 - *v1;

  TPE_Unit m1v1Pm2v2 = ((m1 * *v1) + (m2 * *v2));

  *v1 = (((elasticity * m2 / TPE_FRACTIONS_PER_UNIT) * v2Mv1)
    + m1v1Pm2v2) / m1Pm2;

  *v2 = (((elasticity * m1 / TPE_FRACTIONS_PER_UNIT) * -1 * v2Mv1)
    + m1v1Pm2v2) / m1Pm2;

  if (overflowDanger)
  {
    *v1 *= ANTI_OVERFLOW_SCALE;
    *v2 *= ANTI_OVERFLOW_SCALE;
  }

  #undef ANTI_OVERFLOW
  #undef ANTI_OVERFLOW_SCALE
}

void TPE_resolvePointCollision(
  TPE_Vec4 collisionPoint,
  TPE_Vec4 collisionNormal,
  TPE_Unit elasticity,
  TPE_Vec4 linVelocity1,
  TPE_Vec4 rotVelocity1,
  TPE_Unit m1,
  TPE_Vec4 linVelocity2,
  TPE_Vec4 rotVelocity2,
  TPE_Unit m2)
{
  TPE_Vec4 v1, v2, v1New, v2New;

  TPE_initVec4(&v1);
  TPE_initVec4(&v2);
  TPE_initVec4(&v1New);
  TPE_initVec4(&v2New);

  // add lin. and rot. velocities to get the overall vel. of both points:

  TPE_vec4Add(linVelocity1,rotVelocity1,&v1);
  TPE_vec4Add(linVelocity2,rotVelocity2,&v2);

  /* project both of these velocities to the collision normal as we'll apply
     the collision equation only in the direction of this normal: */

  TPE_vec3Project(v1,collisionNormal,&v1New);
  TPE_vec3Project(v2,collisionNormal,&v2New);

  // get the velocities of the components

  TPE_Unit
    v1NewMag = TPE_vec3Len(v1New),
    v2NewMag = TPE_vec3Len(v2New);

  /* now also substract this component from the original velocity (so that it
     will now be in the collision plane), we'll later add back the updated
     velocity to it */

  TPE_vec4Substract(v1,v1New,&v1); 
  TPE_vec4Substract(v2,v2New,&v2); 

  // apply the 1D collision equation to velocities along the normal:

  TPE_getVelocitiesAfterCollision(
    &v1NewMag,
    &v2NewMag,
    m1,
    m2,
    elasticity);

  // add back the updated velocities to get the new overall velocities:

  v1New.x += (collisionNormal.x * v1NewMag) / TPE_FRACTIONS_PER_UNIT;
  v1New.y += (collisionNormal.y * v1NewMag) / TPE_FRACTIONS_PER_UNIT;
  v1New.z += (collisionNormal.z * v1NewMag) / TPE_FRACTIONS_PER_UNIT;
 
  v2New.x += (collisionNormal.x * v2NewMag) / TPE_FRACTIONS_PER_UNIT;
  v2New.y += (collisionNormal.y * v2NewMag) / TPE_FRACTIONS_PER_UNIT;
  v2New.z += (collisionNormal.z * v2NewMag) / TPE_FRACTIONS_PER_UNIT;

// TODO
}

void TPE_bodyGetTransformMatrix(const TPE_Body *body, TPE_Unit matrix[4][4])
{
  TPE_Vec4 orientation;

  TPE_bodyGetOrientation(body,&orientation);

  TPE_quaternionToRotationMatrix(orientation,matrix);
  matrix[0][3] = body->position.x;
  matrix[1][3] = body->position.y;
  matrix[2][3] = body->position.z;
}

void TPE_quaternionInit(TPE_Vec4 *quaternion)
{
  quaternion->x = 0;
  quaternion->y = 0;
  quaternion->z = 0;
  quaternion->w = TPE_FRACTIONS_PER_UNIT;
}

#endif // guard
