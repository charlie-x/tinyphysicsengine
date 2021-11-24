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
  simulate e.g. the Dzhanibekov effect. Therefore the library is mostly intended
  for entertainment software.

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

  - There is no vec3 type, vec4 is usead for all vectors, for simplicity.
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
#define TPE_SHAPE_CAPSULE   2    ///< capsule: radius, height
#define TPE_SHAPE_CUBOID    3    ///< cuboid, params.: width, height, depth
#define TPE_SHAPE_PLANE     4    ///< plane, params.: width, depth
#define TPE_SHAPE_CYLINDER  5    ///< cylinder, params.: radius, height
#define TPE_SHAPE_TRIMESH   6    /**< triangle mesh, params.:
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
static inline TPE_Unit TPE_abs(TPE_Unit x);
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
void TPE_vec4Set(TPE_Vec4 *v, TPE_Unit x, TPE_Unit y, TPE_Unit z, TPE_Unit w);
void TPE_vec3Add(TPE_Vec4 a, TPE_Vec4 b, TPE_Vec4 *result);
void TPE_vec4Add(TPE_Vec4 a, TPE_Vec4 b, TPE_Vec4 *result);
void TPE_vec3Substract(TPE_Vec4 a, TPE_Vec4 b, TPE_Vec4 *result);
void TPE_vec3Average(TPE_Vec4 a, TPE_Vec4 b, TPE_Vec4 *result);
void TPE_vec4Substract(TPE_Vec4 a, TPE_Vec4 b, TPE_Vec4 *result);
void TPE_vec3Multiply(TPE_Vec4 v, TPE_Unit f, TPE_Vec4 *result);
void TPE_vec3MultiplyPlain(TPE_Vec4 v, TPE_Unit f, TPE_Vec4 *result);
void TPE_vec4Multiply(TPE_Vec4 v, TPE_Unit f, TPE_Vec4 *result);
void TPE_vec3CrossProduct(TPE_Vec4 a, TPE_Vec4 b, TPE_Vec4 *result);
void TPE_vec3Normalize(TPE_Vec4 *v);
void TPE_vec4Normalize(TPE_Vec4 *v);
void TPE_vec3Project(TPE_Vec4 v, TPE_Vec4 base, TPE_Vec4 *result);

TPE_Unit TPE_vec3Len(TPE_Vec4 v);
TPE_Unit TPE_vec3LenTaxicab(TPE_Vec4 v);
TPE_Unit TPE_vec3Dist(TPE_Vec4 a, TPE_Vec4 b);
TPE_Unit TPE_vec4Len(TPE_Vec4 v);
TPE_Unit TPE_vec3DotProduct(TPE_Vec4 v1, TPE_Vec4 v2);
TPE_Unit TPE_vec3DotProductPlain(TPE_Vec4 v1, TPE_Vec4 v2);

TPE_Vec4 TPE_vec4(TPE_Unit x, TPE_Unit y, TPE_Unit z, TPE_Unit w);
TPE_Vec4 TPE_vec3Plus(TPE_Vec4 a, TPE_Vec4 b);
TPE_Vec4 TPE_vec3Minus(TPE_Vec4 a, TPE_Vec4 b);
TPE_Vec4 TPE_vec3Times(TPE_Vec4 a, TPE_Unit f);
TPE_Vec4 TPE_vec3TimesAntiZero(TPE_Vec4 a, TPE_Unit f);
TPE_Vec4 TPE_vec3Cross(TPE_Vec4 a, TPE_Vec4 b);
static inline TPE_Vec4 TPE_vec3Normalized(TPE_Vec4 v);
static inline TPE_Vec4 TPE_vec3Projected(TPE_Vec4 v, TPE_Vec4 base);

/** Returns the closest point on given line segment (a,b) to given point (p). */
TPE_Vec4 TPE_lineSegmentClosestPoint(TPE_Vec4 a, TPE_Vec4 b, TPE_Vec4 p);

/** Converts a linear velocity of an orbiting point to the angular velocity
  (angle units per time units). This depends on the distance of the point from
  the center of rotation. */
TPE_Unit TPE_linearVelocityToAngular(TPE_Unit velocity, TPE_Unit distance);

/** Performs the opposite conversion of TPE_linearVelocityToAngular. */
TPE_Unit TPE_angularVelocityToLinear(TPE_Unit velocity, TPE_Unit distance);

/** Holds a rotation state around a single axis, in a way that prevents rounding
  errors from distorting the rotation over time. In theory rotation of a body
  could be represented as 

  [current orientation, axis of rotation, angular velocity]

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
  TPE_Unit boundingSphereRadius;
} TPE_Body;

/** Initializes a physical body, this should be called on all TPE_Body objects
  that are created.*/
void TPE_bodyInit(TPE_Body *body);

/** Recomputes the body bounding sphere, must be called every time the body's
  shape parameters change. */
void TPE_bodyRecomputeBounds(TPE_Body *body);

/** Computes a 4x4 transform matrix of given body. The matrix has the same
  format as S3L_Mat4 from small3dlib. */
void TPE_bodyGetTransformMatrix(const TPE_Body *body, TPE_Unit matrix[4][4]);

/** Gets the current orientation of a body as a quaternion. */
TPE_Vec4 TPE_bodyGetOrientation(const TPE_Body *body);

void TPE_bodySetOrientation(TPE_Body *body, TPE_Vec4 orientation);

/** Updates the body position and rotation according to its current velocity
  and rotation state. */
void TPE_bodyStep(TPE_Body *body);

/** Sets the rotation state of a body as an axis of rotation and angular
  velocity around this axis. */
void TPE_bodySetRotation(TPE_Body *body, TPE_Vec4 axis, TPE_Unit velocity);

/** Adds a rotation to the current rotation of a body. This addition is perfomed
  as a vector addition of the current and new rotation represented as vectors
  whose direction is the rotation axis and magnitude is the angular velocity 
  around that axis. */
void TPE_bodyAddRotation(TPE_Body *body, TPE_Vec4 axis, TPE_Unit velocity);

/** Applies a velocity change to a body at a specific point (relative to the
  body center), which will change its linear and/or angular velocity. This is
  similar to an impulse but doesn't take mass into account, only velocity. */
void TPE_bodyApplyVelocity(TPE_Body *body, TPE_Vec4 point, TPE_Vec4 velocity);

/** Computes and returns a body's bounding sphere radius, i.e. the maximum
  extent from its center point. */
TPE_Unit TPE_bodyGetMaxExtent(const TPE_Body *body);

/** Computes and returns a body's total kinetic energy (sum of linear and
  rotational kin. energy). In rotating bodies this may not be physically
  accurate as, for simplicity, we operate with the moment of inertia of sphere
  for all bodies (when in reality moment of inertia depends on shape). */
TPE_Unit TPE_bodyGetKineticEnergy(const TPE_Body *body);

/** Collision detection: checks if two bodies are colliding. The return value is
  the collision depth along the collision normal (0 if the bodies are not
  colliding). World-space collision point is returned via a pointer. Collision
  normal (normalized) is also returned via a pointer and its direction is
  "away from body1", i.e. if you move body1 in the opposite direction of this
  normal by the collision depth (return value), the bodies should no longer
  exhibit this particular collision. This function checks the bounding spheres
  to quickly opt out of impossible collisions. */
TPE_Unit TPE_bodyCollides(const TPE_Body *body1, const TPE_Body *body2, 
  TPE_Vec4 *collisionPoint, TPE_Vec4 *collisionNormal);

/** Gets a velocity of a single point on a rigid body, taking into account its
  linear velocity and rotation. The point coordinates are relative to the body
  center. The point does NOT have to be on the surface, it can be inside and
  even outside the body too. */
TPE_Vec4 TPE_bodyGetPointVelocity(const TPE_Body *body, TPE_Vec4 point);

void TPE_resolveCollision(TPE_Body *body1 ,TPE_Body *body2, 
  TPE_Vec4 collisionPoint, TPE_Vec4 collisionNormal, TPE_Unit collisionDepth);

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

/** Initializes quaternion to the rotation identity (i.e. NOT zero
  quaternion). */
void TPE_quaternionInit(TPE_Vec4 *quaternion);

/** Converts a rotation given as an axis and angle around this axis (by right
  hand rule) to a rotation quaternion. */
void TPE_rotationToQuaternion(TPE_Vec4 axis, TPE_Unit angle, 
  TPE_Vec4 *quaternion);

void TPE_quaternionToRotation(TPE_Vec4 quaternion, TPE_Vec4 *axis, 
  TPE_Unit *angle);

/** Computes the conjugate of a quaternion (analogous to matrix inversion, the
quaternion will represent the opposite rotation). */
TPE_Vec4 TPE_quaternionConjugate(TPE_Vec4 quaternion);

/** Converts a rotation quaternion to a 4x4 rotation matrix. The matrix is
  indexed as [column][row] and is in the same format as S3L_Mat4 from
  small3dlib. */
void TPE_quaternionToRotationMatrix(TPE_Vec4 quaternion, TPE_Unit matrix[4][4]);

void TPE_rotatePoint(TPE_Vec4 *point, TPE_Vec4 quaternion);

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

  body->mass = TPE_FRACTIONS_PER_UNIT;

  body->boundingSphereRadius = 0;
}

void TPE_bodySetOrientation(TPE_Body *body, TPE_Vec4 orientation)
{
  body->rotation.originalOrientation = orientation;
  body->rotation.currentAngle = 0;
}

TPE_Vec4 TPE_bodyGetOrientation(const TPE_Body *body)
{
  TPE_Vec4 axisRotation, result;

  TPE_rotationToQuaternion(
    body->rotation.axisVelocity,
    body->rotation.currentAngle,
    &axisRotation);

  TPE_quaternionMultiply(
    body->rotation.originalOrientation,
    axisRotation,
    &result);

  TPE_vec4Normalize(&result);

  return result;
}

void TPE_vec3CrossProduct(TPE_Vec4 a, TPE_Vec4 b, TPE_Vec4 *result)
{
  TPE_Vec4 r;

  r.x = (a.y * b.z - a.z * b.y) / TPE_FRACTIONS_PER_UNIT;
  r.y = (a.z * b.x - a.x * b.z) / TPE_FRACTIONS_PER_UNIT;
  r.z = (a.x * b.y - a.y * b.x) / TPE_FRACTIONS_PER_UNIT;

  *result = r;
}

TPE_Vec4 TPE_vec3Cross(TPE_Vec4 a, TPE_Vec4 b)
{
  TPE_vec3CrossProduct(a,b,&a);
  return a;
}

void TPE_bodyApplyVelocity(TPE_Body *body, TPE_Vec4 point, TPE_Vec4 velocity)
{ 
  TPE_Unit pointDistance = TPE_vec3Len(point);

  if (pointDistance != 0)  
  {
    TPE_Vec4 angularVelocity, rotationAxis;
  
    TPE_vec3Add(body->velocity,velocity,&(body->velocity));

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

//    TPE_bodyAddRotation(body,rotationAxis,
//      TPE_linearVelocityToAngular(
//        TPE_vec3Len(angularVelocity),-1 * pointDistance));
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

void _TPE_getCapsuleCyllinderEndpoints(const TPE_Body *body, 
  TPE_Vec4 *a, TPE_Vec4 *b)
{
  TPE_Vec4 quat = TPE_bodyGetOrientation(body);

  *a = TPE_vec4(0,body->shapeParams[1] / 2,0,0);
  *b = TPE_vec4(0,-1 * a->y,0,0);

  TPE_rotatePoint(a,quat);
  TPE_rotatePoint(b,quat);

  TPE_vec3Add(*a,body->position,a);
  TPE_vec3Add(*b,body->position,b);
}

/** Helpter function for cuboid collision detection. Given a line segment
  as a line equation limited by parameter bounds t1 and t2, center point C and
  side offset from the center point O, the function further limits the parameter
  bounds (t1, t2) to restrict the line only to the region between two planes:
  both with normal O, one passing throung point C + O and the other through
  C - O. If t2 > t1 after this function finishes, the line segment is completely
  outside the region. */
void _TPE_cutLineSegmentByPlanes(TPE_Vec4 center, TPE_Vec4 sideOffset, 
  TPE_Vec4 lineStart, TPE_Vec4 lineDir, TPE_Unit *t1, TPE_Unit *t2)
{
  TPE_Unit da = TPE_vec3DotProductPlain(sideOffset,lineStart);

  TPE_Vec4 dc;

  dc.z = 0;

  // TODO: dor(d,dc) could be cached for all sides between calls to save recomputing

  dc = TPE_vec3Plus(center,sideOffset);

  TPE_Unit denom = TPE_nonZero(TPE_vec3DotProductPlain(sideOffset,lineDir));

#define tAntiOverflow(t) \
  TPE_Unit t = TPE_vec3DotProductPlain(sideOffset,dc) - da;\
  t = (TPE_abs(t) < 500000) ? (t * TPE_FRACTIONS_PER_UNIT) / denom :\
    (((t / 64) * TPE_FRACTIONS_PER_UNIT) / TPE_nonZero(denom / 64));

  tAntiOverflow(tA)  

  dc = TPE_vec3Minus(center,sideOffset);

  tAntiOverflow(tB)

#undef tAntiOverflow

  if (tB < tA)
  {
    TPE_Unit tmp = tA;
    tA = tB;
    tB = tmp;
  }

  if (tA > *t1)
    *t1 = tA;

  if (tB < *t2)
    *t2 = tB;
}

TPE_Unit TPE_bodyCollides(const TPE_Body *body1, const TPE_Body *body2, 
  TPE_Vec4 *collisionPoint, TPE_Vec4 *collisionNormal)
{
  // handle collision of different shapes each in a specific case:

  uint16_t collType = TPE_COLLISION_TYPE(body1->shape,body2->shape);

  if (collType != TPE_COLLISION_TYPE(TPE_SHAPE_SPHERE,TPE_SHAPE_SPHERE))
  {
    /* initial bounding sphere check to quickly discard impossible collisions,
       plus this also prevents overflow errors in long-distance computations */

    // TODO: taxicab could be also considered here

    if (TPE_vec3Len(TPE_vec3Minus(body1->position,body2->position)) >
        body1->boundingSphereRadius + body2->boundingSphereRadius)
      return 0;
  } 

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

    case TPE_COLLISION_TYPE(TPE_SHAPE_SPHERE,TPE_SHAPE_CAPSULE):
    {
      const TPE_Body *sphere;
      const TPE_Body *capsule;

      _TPE_getShapes(body1,body2,TPE_SHAPE_SPHERE,&sphere,&capsule);

      TPE_Vec4 cA, cB;

      _TPE_getCapsuleCyllinderEndpoints(capsule,&cA,&cB);

      TPE_Body sphere2; // sphere at the capsule's closest point

      TPE_bodyInit(&sphere2);
      sphere2.shape = TPE_SHAPE_SPHERE;
      sphere2.shapeParams[0] = capsule->shapeParams[0];
      sphere2.position = TPE_lineSegmentClosestPoint(cA,cB,sphere->position);

      uint8_t swap = sphere == body2;

      return TPE_bodyCollides(swap ? &sphere2 : sphere,swap ? sphere : &sphere2,
        collisionPoint,collisionNormal);

      break;
    }

    case TPE_COLLISION_TYPE(TPE_SHAPE_CAPSULE,TPE_SHAPE_CAPSULE):
    {
      TPE_Vec4 a1, b1, a2, b2;

      _TPE_getCapsuleCyllinderEndpoints(body1,&a1,&b1);
      _TPE_getCapsuleCyllinderEndpoints(body2,&a2,&b2);

      TPE_Unit aa, ab, ba, bb; // squared distances between points

      TPE_Vec4 tmp; 

      tmp = TPE_vec3Minus(a1,a2);
      aa = tmp.x * tmp.x + tmp.y * tmp.y + tmp.z * tmp.z;

      tmp = TPE_vec3Minus(a1,b2);
      ab = tmp.x * tmp.x + tmp.y * tmp.y + tmp.z * tmp.z;

      tmp = TPE_vec3Minus(b1,a2);
      ba = tmp.x * tmp.x + tmp.y * tmp.y + tmp.z * tmp.z;

      tmp = TPE_vec3Minus(b1,b2);
      bb = tmp.x * tmp.x + tmp.y * tmp.y + tmp.z * tmp.z;

      // let a1 hold the point figuring in the shortest distance:

      if (ab < aa)
        aa = ab; // means: aa = min(aa,ab)

      if (bb < ba)
        ba = bb; // means: ba = min(ba,bb)

      if (ba < aa) // means: min(ba,bb) < min(aa,ab)
        a1 = b1;

      a2 = TPE_lineSegmentClosestPoint(a2,b2,a1);
      a1 = TPE_lineSegmentClosestPoint(a1,b1,a2);

      // now a1 and a2 are the closest two points on capsule axes

      TPE_Body sphere1, sphere2;
      
      TPE_bodyInit(&sphere1);
      sphere1.shape = TPE_SHAPE_SPHERE;
      sphere1.shapeParams[0] = body1->shapeParams[0];
      sphere1.position = a1;
      
      TPE_bodyInit(&sphere2);
      sphere2.shape = TPE_SHAPE_SPHERE;
      sphere2.shapeParams[0] = body2->shapeParams[0];
      sphere2.position = a2;

      return TPE_bodyCollides(&sphere1,&sphere2,collisionPoint,collisionNormal);
      break;
    }

    case TPE_COLLISION_TYPE(TPE_SHAPE_SPHERE,TPE_SHAPE_CYLINDER):
    {
      // TODO: would this be better to do via sphere-capsule collision?

      const TPE_Body *sphere;
      const TPE_Body *cylinder;

      _TPE_getShapes(body1,body2,TPE_SHAPE_SPHERE,&sphere,&cylinder);

      TPE_Vec4 sphereRelativePos = // by this we shift the cylinder to [0,0,0]
        TPE_vec3Minus(sphere->position,cylinder->position);

      // vector along the cylinder height:
      TPE_Vec4 cylinderAxis = TPE_vec4(0,TPE_FRACTIONS_PER_UNIT,0,0);

      TPE_rotatePoint(&cylinderAxis,TPE_bodyGetOrientation(cylinder));

      TPE_Vec4 sphereAxisPos = // sphere pos projected to the cylinder axis
        TPE_vec3Projected(sphereRelativePos,cylinderAxis);

      TPE_Unit sphereAxisDistance = TPE_vec3Len(sphereAxisPos);

      TPE_Unit tmp = cylinder->shapeParams[1] / 2; // half of cylinder height

      /* now we have three possible regions the sphere can occupy:

           C :B:  A  :B: C
             : :_____: :
             : |_____| : cylinder
             : :     : :
             : :     : : */

      if (sphereAxisDistance >= tmp + sphere->shapeParams[0]) // case C: no col.
        break; 

      TPE_Vec4 sphereAxisToRelative =
        TPE_vec3Minus(sphereRelativePos,sphereAxisPos);

      TPE_Unit sphereCylinderDistance = TPE_vec3Len(sphereAxisToRelative);

      tmp = sphereAxisDistance - tmp;

      if (tmp < 0) // case A: potential collision with cylinder body
      {
        TPE_Unit penetration = cylinder->shapeParams[0] 
          - (sphereCylinderDistance - sphere->shapeParams[0]);

        if (penetration > 0)
        {
          TPE_vec3Normalize(&sphereAxisToRelative);

          *collisionPoint = TPE_vec3Plus(cylinder->position,
            TPE_vec3Plus(sphereAxisPos,TPE_vec3Times(
            sphereAxisToRelative,cylinder->shapeParams[0])));

          *collisionNormal = sphereAxisToRelative;

          if (sphere == body1)
            TPE_vec3MultiplyPlain(*collisionNormal,-1,collisionNormal);

          return penetration;
        }
        else
          break;
      }

      /* case B: here we have two subcases, one with the sphere center being
         within the cylinder radius (collision with the cylinder top/bottom),
         and the other case (collision with the cylinder top/bottom edge). */

      TPE_Vec4 cylinderPlaneMiddle = TPE_vec3Times(
          TPE_vec3Normalized(sphereAxisPos),
             cylinder->shapeParams[1] / 2);

      if (sphereCylinderDistance < cylinder->shapeParams[0]) // top/bottom cap
      {
        TPE_Unit penetration = cylinder->shapeParams[1] / 2 - 
          (sphereAxisDistance - sphere->shapeParams[0]);

        if (penetration <= 0) // shouldn't normally happen, but rounding errors 
          penetration = 1;

        *collisionNormal = TPE_vec3Normalized(sphereAxisPos);

        *collisionPoint = 
          TPE_vec3Plus(
            cylinder->position,
            TPE_vec3Plus(sphereAxisToRelative,cylinderPlaneMiddle));

        if (body1 == sphere)
          TPE_vec3MultiplyPlain(*collisionNormal,-1,collisionNormal);

        return penetration;
      }
      else // potential edge collision
      {
        TPE_Vec4 edgePoint = TPE_vec3Plus(cylinderPlaneMiddle,
            TPE_vec3Times(TPE_vec3Normalized(sphereAxisToRelative),
              cylinder->shapeParams[0]));

        TPE_Unit penetration = sphere->shapeParams[0] -
          TPE_vec3Dist(edgePoint,sphereRelativePos);

        if (penetration > 0)
        {
          *collisionPoint = TPE_vec3Plus(cylinder->position,edgePoint);

          *collisionNormal = 
            TPE_vec3Normalized(TPE_vec3Minus(sphereRelativePos,edgePoint));

          if (body1 == sphere)
            TPE_vec3MultiplyPlain(*collisionNormal,-1,collisionNormal);

          return penetration;
        }
      }

      break;
    }

    case TPE_COLLISION_TYPE(TPE_SHAPE_CUBOID,TPE_SHAPE_CUBOID):
    {
      TPE_Vec4 // min/max extent of the colliding area:
        collisionExtentMax = 
          TPE_vec4(-TPE_INFINITY,-TPE_INFINITY,-TPE_INFINITY,0),
        collisionExtentMin = 
          TPE_vec4(TPE_INFINITY,TPE_INFINITY,TPE_INFINITY,0);

      uint8_t collisionHappened = 0;

      TPE_Vec4 aX1, aY1, aZ1, // first cuboid axes
               aX2, aY2, aZ2; // second cuboid axes

      for (uint8_t i = 0; i < 2; ++i) // for each body
      {
        TPE_Vec4 q = TPE_bodyGetOrientation(body1);

        // construct the cuboid axes:

        aX1 = TPE_vec4(body1->shapeParams[0] / 2,0,0,0);
        TPE_rotatePoint(&aX1,q);

        aY1 = TPE_vec4(0,body1->shapeParams[1] / 2,0,0);
        TPE_rotatePoint(&aY1,q);

        aZ1 = TPE_vec4(0,0,body1->shapeParams[2] / 2,0);
        TPE_rotatePoint(&aZ1,q);

        q = TPE_bodyGetOrientation(body2);

        aX2 = TPE_vec4(body2->shapeParams[0] / 2,0,0,0);
        TPE_rotatePoint(&aX2,q);

        aY2 = TPE_vec4(0,body2->shapeParams[1] / 2,0,0);
        TPE_rotatePoint(&aY2,q);

        aZ2 = TPE_vec4(0,0,body2->shapeParams[2] / 2,0);
        TPE_rotatePoint(&aZ2,q);

        uint8_t edges[12] = // list of all cuboid edges as combinations of axes
        {       // xyz xyz 
          0x3b, // +++ -++ |
          0x3e, // +++ ++- | top
          0x13, // -+- -++ |
          0x16, // -+- ++- |
          0x29, // +-+ --+  |
          0x2c, // +-+ +--  | bottom
          0x01, // --- --+  |
          0x04, // --- +--  |
          0x3d, // +++ +-+ |
          0x19, // -++ --+ | sides
          0x10, // -+- --- |
          0x34  // ++- +-- |
        };
 
        for (uint8_t j = 0; j < 12; ++j) // for each edge
        {
          // we check the edge against all sides of the other cuboid

          TPE_Vec4 lineStart = body1->position;
          TPE_Vec4 lineEnd = body1->position;

          uint8_t edge = edges[j];

#define offsetCenter(c,v,a) \
  v = (edge & c) ? TPE_vec3Plus(v,a) : TPE_vec3Minus(v,a);

          offsetCenter(0x04,lineStart,aX1)
          offsetCenter(0x02,lineStart,aY1)
          offsetCenter(0x01,lineStart,aZ1)

          offsetCenter(0x20,lineEnd,aX1)
          offsetCenter(0x10,lineEnd,aY1)
          offsetCenter(0x08,lineEnd,aZ1)

#undef offsetCenter 

          TPE_Unit t1 = 0, t2 = TPE_FRACTIONS_PER_UNIT;

          TPE_Vec4 edgeDir = TPE_vec3Minus(lineEnd,lineStart);

          for (uint8_t k = 0; k < 3; ++k) // for each axis (pair of sides)
          {
            TPE_Vec4 *sideOffset;

            if (k == 0)
              sideOffset = &aX2;
            else if (k == 1)
              sideOffset = &aY2;
            else
              sideOffset = &aZ2;

            _TPE_cutLineSegmentByPlanes(body2->position,*sideOffset,lineStart,
            edgeDir,&t1,&t2);

            if (t1 > t2)
              break; // no solution already, no point checking on
          }

          if (t2 > t1) // if part of edge exists between all side planes
          {
            // edge collided with the cuboid
 
            collisionHappened = 1;

            *collisionPoint = edgeDir;
            collisionPoint->x = (collisionPoint->x * t1) / TPE_FRACTIONS_PER_UNIT;
            collisionPoint->y = (collisionPoint->y * t1) / TPE_FRACTIONS_PER_UNIT;
            collisionPoint->z = (collisionPoint->z * t1) / TPE_FRACTIONS_PER_UNIT;
            *collisionPoint = TPE_vec3Plus(lineStart,*collisionPoint);

            if (collisionPoint->x > collisionExtentMax.x)
              collisionExtentMax.x = collisionPoint->x;

            if (collisionPoint->x < collisionExtentMin.x)
              collisionExtentMin.x = collisionPoint->x;

            if (collisionPoint->y > collisionExtentMax.y)
              collisionExtentMax.y = collisionPoint->y;

            if (collisionPoint->y < collisionExtentMin.y)
              collisionExtentMin.y = collisionPoint->y;

            if (collisionPoint->z > collisionExtentMax.z)
              collisionExtentMax.z = collisionPoint->z;

            if (collisionPoint->z < collisionExtentMin.z)
              collisionExtentMin.z = collisionPoint->z;

            *collisionPoint = edgeDir;
            collisionPoint->x = (collisionPoint->x * t2) / TPE_FRACTIONS_PER_UNIT;
            collisionPoint->y = (collisionPoint->y * t2) / TPE_FRACTIONS_PER_UNIT;
            collisionPoint->z = (collisionPoint->z * t2) / TPE_FRACTIONS_PER_UNIT;
            *collisionPoint = TPE_vec3Plus(lineStart,*collisionPoint);

            if (collisionPoint->x > collisionExtentMax.x)
              collisionExtentMax.x = collisionPoint->x;

            if (collisionPoint->x < collisionExtentMin.x)
              collisionExtentMin.x = collisionPoint->x;

            if (collisionPoint->y > collisionExtentMax.y)
              collisionExtentMax.y = collisionPoint->y;

            if (collisionPoint->y < collisionExtentMin.y)
              collisionExtentMin.y = collisionPoint->y;

            if (collisionPoint->z > collisionExtentMax.z)
              collisionExtentMax.z = collisionPoint->z;

            if (collisionPoint->z < collisionExtentMin.z)
              collisionExtentMin.z = collisionPoint->z;
          }
        } // for each edge

        if (i == 0)
        {
          // now swap the bodies and do it again:

          const TPE_Body *tmp = body1;
          body1 = body2;
          body2 = tmp;
        }
      } // for each body

      if (collisionHappened)
      {
        // average all collision points to get the center point

        *collisionPoint = TPE_vec3Plus(collisionExtentMin,collisionExtentMax);

        collisionPoint->x /= 2;
        collisionPoint->y /= 2;
        collisionPoint->z /= 2;
        collisionPoint->w = 0;

        // compute the coll. normal as the axis closest to the coll. point

        TPE_Vec4 bestAxis = TPE_vec4(1,0,0,0);
        TPE_Unit bestDot = -1;
        TPE_Unit currentDot;

        uint8_t currentBody = 0;
        uint8_t bestBody = 0;

        // TODO: optimize this shit? create array instead of aX1, aX2 etc.?

        collisionExtentMin = TPE_vec3Minus(*collisionPoint,body1->position); // reuse

        #define checkAxis(a) \
          currentDot = (TPE_vec3DotProduct(a,collisionExtentMin) * TPE_FRACTIONS_PER_UNIT) / \
            TPE_nonZero(TPE_vec3DotProduct(a,a)); \
          if (currentDot > bestDot) \
            { bestDot = currentDot; bestAxis = a; bestBody = currentBody; } \
          else { \
            currentDot *= -1; \
            if (currentDot > bestDot) { \
              bestDot = currentDot; bestAxis = a; bestBody = currentBody; \
              TPE_vec3MultiplyPlain(bestAxis,-1,&bestAxis); } \
          }

        checkAxis(aX1)
        checkAxis(aY1)
        checkAxis(aZ1)

        collisionExtentMin = TPE_vec3Minus(*collisionPoint,body2->position);

        currentBody = 1;

        checkAxis(aX2)
        checkAxis(aY2)
        checkAxis(aZ2)

        #undef checkAxis

// TODO: optimize/refactor this mess

        *collisionNormal = bestAxis;

        if (bestBody == 0)
          TPE_vec3MultiplyPlain(*collisionNormal,-1,collisionNormal); 

        TPE_vec3Normalize(collisionNormal);

        TPE_Unit len = TPE_nonZero(TPE_vec3Len(bestAxis));

        return len -
          TPE_vec3DotProductPlain(bestAxis,
          TPE_vec3Minus(*collisionPoint,
            bestBody == 0 ? body1->position : body2->position)) / len;
      }

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

  TPE_Vec4 normal = TPE_vec3Cross(
    point,TPE_vec3Minus(point,body->rotation.axisVelocity));

TPE_vec3MultiplyPlain(normal,-1,&normal); // TODO: think about WHY

  TPE_Unit dist = TPE_vec3Len(normal);  // point-line distance

  TPE_Unit velocity = 
    TPE_angularVelocityToLinear(body->rotation.axisVelocity.w,dist);

  TPE_vec3Normalize(&normal);

  return TPE_vec3Plus(result,TPE_vec3Times(normal,velocity));
}

void TPE_resolveCollision(TPE_Body *body1 ,TPE_Body *body2, 
  TPE_Vec4 collisionPoint, TPE_Vec4 collisionNormal, TPE_Unit collisionDepth)
{
printf("---\n");
  TPE_Vec4 v1, v2, p1, p2;

  p1 = TPE_vec3Minus(collisionPoint,body1->position);
  p2 = TPE_vec3Minus(collisionPoint,body2->position);

/*
  TODO:

  We don't have to compute second body's final velocity! From momentum
  preservation it can be computer from the initial total momentum and first
  body's new momentum.

  ALSO: compute the rotation velocity from preservaion of rotational kinetic
  energy!
*/

// separate the bodies:

collisionPoint = collisionNormal; // reuse collisionPoint
TPE_vec3Multiply(collisionPoint,collisionDepth,&collisionPoint);
TPE_vec3Add(body2->position,collisionPoint,&body2->position);
TPE_vec3Substract(body1->position,collisionPoint,&body1->position);



  v1 = TPE_bodyGetPointVelocity(body1,p1); 
  v2 = TPE_bodyGetPointVelocity(body2,p2);
 
  TPE_Unit tmp = TPE_vec3DotProduct(v1,collisionNormal);
  int8_t v1Sign = tmp > 0 ? 1 : (tmp < 0 ? -1 : 0);

  tmp = TPE_vec3DotProduct(v2,collisionNormal);
  int8_t v2Sign = tmp > 0 ? 1 : (tmp < 0 ? -1 : 0);

  if (v1Sign == -1 && v2Sign != -1)
    return; // opposite going velocities => not a real collision

  /* if the velocities are too small, weird behavior occurs, so we define a min
    velocity for collisions and potentially modify the velocities: */

  // TODO: something more elegant?

  #define MIN_V 5

  if (v1.x != 0 || v1.y != 0 || v1.z != 0)
    while (TPE_vec3LenTaxicab(v1) < MIN_V)
    {
      v1.x *= 2;
      v1.y *= 2;
      v1.z *= 2;
    } 

  if (v2.x != 0 || v2.y != 0 || v2.z != 0)
    while (TPE_vec3LenTaxicab(v2) < MIN_V)
    {
      v2.x *= 2;
      v2.y *= 2;
      v2.z *= 2;
    } 

  #undef MIN_V 

  TPE_vec3Project(v1,collisionNormal,&v1);
  TPE_vec3Project(v2,collisionNormal,&v2);

  TPE_Unit 
    v1Scalar = TPE_vec3Len(v1) * v1Sign,
    v2Scalar = TPE_vec3Len(v2) * v2Sign;

  if ((v1Sign == 1 && v2Sign == 1 && (v2Scalar > v1Scalar)) ||
    (v1Sign == -1 && v2Sign == -1 && (v1Scalar > v2Scalar)))
    return; // not a valid collision

  TPE_Unit
    v1ScalarNew = v1Scalar,
    v2ScalarNew = v2Scalar;

  TPE_getVelocitiesAfterCollision(
    &v1ScalarNew,
    &v2ScalarNew,
    body1->mass,
    body2->mass,
    512); // TODO: elasticity

// TODO: ACTUALLY MAKE SURE MOMENTUM IS CONSERVED (rounding errors may add energy!)

//TPE_vec3MultiplyPlain(collisionNormal,-1,&collisionNormal);

  TPE_bodyApplyVelocity(body1,p1,
//    TPE_vec3TimesAntiZero(collisionNormal,v1ScalarNew - v1Scalar));
    TPE_vec3Times(collisionNormal,v1ScalarNew - v1Scalar));
  
  TPE_bodyApplyVelocity(body2,p2,
//    TPE_vec3TimesAntiZero(collisionNormal,v2ScalarNew - v2Scalar));
    TPE_vec3Times(collisionNormal,v2ScalarNew - v2Scalar));
}

TPE_Unit TPE_linearVelocityToAngular(TPE_Unit velocity, TPE_Unit distance)
{
  TPE_Unit circumfence = (2 * TPE_PI * distance) / TPE_FRACTIONS_PER_UNIT;
  return (velocity * TPE_FRACTIONS_PER_UNIT) / circumfence;
}

TPE_Unit TPE_angularVelocityToLinear(TPE_Unit velocity, TPE_Unit distance)
{
  TPE_Unit circumfence = (2 * TPE_PI * distance) / TPE_FRACTIONS_PER_UNIT;
  return (velocity * circumfence) / TPE_FRACTIONS_PER_UNIT;
}

void TPE_bodyStep(TPE_Body *body)
{
  TPE_vec3Add(body->position,body->velocity,&(body->position));
  body->rotation.currentAngle += body->rotation.axisVelocity.w;
}

void TPE_bodySetRotation(TPE_Body *body, TPE_Vec4 axis, TPE_Unit velocity)
{
  body->rotation.originalOrientation = TPE_bodyGetOrientation(body);

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
  a.z += b.z;

  return a;
}

TPE_Vec4 TPE_vec3Minus(TPE_Vec4 a, TPE_Vec4 b)
{
  a.x -= b.x;
  a.y -= b.y;
  a.z -= b.z;

  return a;
}

TPE_Vec4 TPE_vec3Times(TPE_Vec4 a, TPE_Unit f)
{
  a.x = (a.x * f) / TPE_FRACTIONS_PER_UNIT;
  a.y = (a.y * f) / TPE_FRACTIONS_PER_UNIT;
  a.z = (a.z * f) / TPE_FRACTIONS_PER_UNIT;

  return a;
}

TPE_Vec4 TPE_vec3TimesAntiZero(TPE_Vec4 a, TPE_Unit f)
{
  a.x *= f;

  if (a.x != 0)
    a.x = a.x >= TPE_FRACTIONS_PER_UNIT ? a.x / TPE_FRACTIONS_PER_UNIT : 
      (a.x > 0 ? 1 : -1);

  a.y *= f;

  if (a.y != 0)
    a.y = a.y >= TPE_FRACTIONS_PER_UNIT ? a.y / TPE_FRACTIONS_PER_UNIT :
      (a.y > 0 ? 1 : -1);

  a.z *= f;

  if (a.z != 0)
    a.z = a.z >= TPE_FRACTIONS_PER_UNIT ? a.z / TPE_FRACTIONS_PER_UNIT :
      (a.z > 0 ? 1 : -1);

/*
  if (a.x != 0)
    a.x = a.x >= TPE_FRACTIONS_PER_UNIT ? a.x / TPE_FRACTIONS_PER_UNIT : 
      (a.x > 0 ? 1 : -1);

  if (a.y != 0)
    a.y = a.y >= TPE_FRACTIONS_PER_UNIT ? a.y / TPE_FRACTIONS_PER_UNIT :
      (a.y > 0 ? 1 : -1);

  if (a.z != 0)
    a.z = a.z >= TPE_FRACTIONS_PER_UNIT ? a.z / TPE_FRACTIONS_PER_UNIT :
      (a.z > 0 ? 1 : -1);
*/

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

void TPE_vec3MultiplyPlain(TPE_Vec4 v, TPE_Unit f, TPE_Vec4 *result)
{
  result->x = v.x * f;
  result->y = v.y * f;
  result->z = v.z * f;
}

void TPE_vec4Multiply(const TPE_Vec4 v, TPE_Unit f, TPE_Vec4 *result)
{
  result->x = (v.x * f) / TPE_FRACTIONS_PER_UNIT;
  result->y = (v.y * f) / TPE_FRACTIONS_PER_UNIT;
  result->z = (v.z * f) / TPE_FRACTIONS_PER_UNIT;
  result->w = (v.w * f) / TPE_FRACTIONS_PER_UNIT;
}

TPE_Unit TPE_abs(TPE_Unit x)
{
  return (x >= 0) ? x : (-1 * x);
}

TPE_Unit TPE_vec3Len(TPE_Vec4 v)
{
  return TPE_sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

TPE_Unit TPE_vec3Dist(TPE_Vec4 a, TPE_Vec4 b)
{
  return TPE_vec3Len(TPE_vec3Minus(a,b));
}

TPE_Unit TPE_vec4Len(TPE_Vec4 v)
{
  return TPE_sqrt(v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w);
}

TPE_Unit TPE_vec3LenTaxicab(TPE_Vec4 v)
{
  return TPE_abs(v.x) + TPE_abs(v.y) + TPE_abs(v.z);
}

TPE_Unit TPE_vec3DotProduct(const TPE_Vec4 v1, const TPE_Vec4 v2)
{
  return
    (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z) / TPE_FRACTIONS_PER_UNIT;
}

TPE_Unit TPE_vec3DotProductPlain(const TPE_Vec4 v1, const TPE_Vec4 v2)
{
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
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

TPE_Vec4 TPE_vec3Projected(TPE_Vec4 v, TPE_Vec4 base)
{
  TPE_Vec4 r;

  TPE_vec3Project(v,base,&r);

  return r;
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

  TPE_Unit m1Pm2 = TPE_nonZero(m1 + m2);
  TPE_Unit v2Mv1 = TPE_nonZero(*v2 - *v1);

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

void TPE_bodyGetTransformMatrix(const TPE_Body *body, TPE_Unit matrix[4][4])
{
  TPE_Vec4 orientation;

  orientation = TPE_bodyGetOrientation(body);

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

void TPE_rotatePoint(TPE_Vec4 *point, TPE_Vec4 quaternion)
{
  // TODO: the first method is bugged, but maybe would be faster?

#if 0
  TPE_Vec4 quaternionConjugate = TPE_quaternionConjugate(quaternion);

  point->w = 0;

  TPE_quaternionMultiply(quaternion,*point,point);
  TPE_quaternionMultiply(*point,quaternionConjugate,point);
#else
  TPE_Unit m[4][4];

  TPE_quaternionToRotationMatrix(quaternion,m);

  TPE_Vec4 p = *point;

  point->x = (p.x * m[0][0] + p.y * m[0][1] + p.z * m[0][2]) / TPE_FRACTIONS_PER_UNIT;
  point->y = (p.x * m[1][0] + p.y * m[1][1] + p.z * m[1][2]) / TPE_FRACTIONS_PER_UNIT;
  point->z = (p.x * m[2][0] + p.y * m[2][1] + p.z * m[2][2]) / TPE_FRACTIONS_PER_UNIT;
#endif
}

TPE_Vec4 TPE_quaternionConjugate(TPE_Vec4 quaternion)
{
  quaternion.x *= -1;
  quaternion.y *= -1;
  quaternion.z *= -1;

  return quaternion;
}

TPE_Vec4 TPE_vec3Normalized(TPE_Vec4 v)
{
  TPE_vec3Normalize(&v);
  return v;
}

TPE_Vec4 TPE_lineSegmentClosestPoint(TPE_Vec4 a, TPE_Vec4 b, TPE_Vec4 p)
{
  TPE_Vec4 ab = TPE_vec3Minus(b,a);

  TPE_Unit t = ((TPE_vec3DotProduct(ab,TPE_vec3Minus(p,a)) * 
    TPE_FRACTIONS_PER_UNIT) / TPE_nonZero(TPE_vec3DotProduct(ab,ab)));

  if (t < 0)
    t = 0;
  else if (t > TPE_FRACTIONS_PER_UNIT)
    t = TPE_FRACTIONS_PER_UNIT;  

  TPE_vec3Multiply(ab,t,&ab);

  return TPE_vec3Plus(a,ab);
}

TPE_Unit TPE_bodyGetKineticEnergy(const TPE_Body *body)
{
  TPE_Unit v = TPE_vec3Len(body->velocity);

  v *= v;

  v = (v == 0 || v >= TPE_FRACTIONS_PER_UNIT) ?
    v / TPE_FRACTIONS_PER_UNIT : 1;

// TODO: handle small values

  return (body->mass * v) /
    (2 * TPE_FRACTIONS_PER_UNIT);

  // TODO: rot
}

TPE_Unit TPE_bodyGetMaxExtent(const TPE_Body *body)
{
  switch (body->shape)
  {
    case TPE_SHAPE_SPHERE:
      return body->shapeParams[0]; 
      break;

    case TPE_SHAPE_CUBOID:
      return TPE_vec3Len(TPE_vec4(
        body->shapeParams[0] / 2,
        body->shapeParams[1] / 2,
        body->shapeParams[2] / 2,0));
      break;

    // TODO: other shapes
    default: return 0; break;
  }
}

void TPE_bodyRecomputeBounds(TPE_Body *body)
{
  body->boundingSphereRadius = TPE_bodyGetMaxExtent(body);
}

#endif // guard
