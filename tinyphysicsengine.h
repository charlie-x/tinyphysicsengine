#ifndef TINYPHYSICSENGINE_H
#define TINYPHYSICSENGINE_H

/*
  author: Miloslav Ciz
  license: CC0 1.0 (public domain)
           found at https://creativecommons.org/publicdomain/zero/1.0/
           + additional waiver of all IP
  version: 0.1d

  CONVENTIONS:

  - No floating point is used, we instead use integers (effectively a fixed
    point). TPE_FRACTIONS_PER_UNIT is an equivalent to 1.0 in floating point and
    all numbers are normalized by this constant.

  - Units: for any measure only an abstract mathematical unit is used. This unit
    always has TPE_FRACTIONS_PER_UNIT parts. You can see assign any
    correcpondence with real life units to these units. E.g. 1 spatial unit
    (which you can see as e.g. 1 meter) is equal to TPE_FRACTIONS_PER_UNIT.
    Same with temporatl (e.g. 1 second) and mass (e.g. 1 kilogram) units, and
    also any derived units, e.g. a unit of velocity (e.g. 1 m/s) is also equal
    to 1 TPE_FRACTIONS_PER_UNIT. A full angle is also split into
    TPE_FRACTIONS_PER_UNIT parts (instead of 2 * PI or degrees).
*/

#include <stdint.h>

typedef int32_t TPE_Unit;

/** How many fractions a unit is split into. This is NOT SUPPOSED TO BE
  REDEFINED, so rather don't do it (otherwise things may overflow etc.). */
#define TPE_FRACTIONS_PER_UNIT 512

#define TPE_INFINITY 2147483647

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

typedef struct
{
  TPE_Unit x;
  TPE_Unit y;
  TPE_Unit z;
  TPE_Unit w;
} TPE_Vec4;

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
  TPE_Vec4 orientation;     ///< orientation as a quaternion

  TPE_Vec4 velocity;        ///< linear velocity vector
  TPE_Vec4 rotation;        /**< current rotational state: X, Y and Z are the
                                 normalized axis of rotation (we only allow
                                 one), W is a non-negative angular speed around
                                 this axis (one angle unit per temporal unit) in
                                 the direction given by right hand rule
                                 (mathematically we could have just X, Y and Z
                                 with the size of vector being angular speed,
                                 but for computational/performance it's better
                                 this way), DO NOT SET THIS MANUALLY (use a
                                 function) */
} TPE_Body;

#define TPE_PRINTF_VEC3(v) printf("[%d %d %d]",v[0],v[1],v[2]);

typedef struct
{
  uint16_t bodyCount;
  TPE_Body *bodies;
} TPE_PhysicsWorld;

//------------------------------------------------------------------------------

void TPE_initVec4(TPE_Vec4 *v)
{
  v->x = 0;
  v->y = 0;
  v->z = 0;
  v->w = 0;
}

TPE_Unit TPE_wrap(TPE_Unit value, TPE_Unit mod)
{
  return value >= 0 ? (value % mod) : (mod + (value % mod) - 1);
}

#define TPE_SIN_TABLE_LENGTH 128

static const TPE_Unit TPE_sinTable[TPE_SIN_TABLE_LENGTH] =
{
  /* 511 was chosen here as a highest number that doesn't overflow during
     compilation for TPE_FRACTIONS_PER_UNIT == 1024 */

  (0*S3L_FRACTIONS_PER_UNIT)/511, (6*S3L_FRACTIONS_PER_UNIT)/511, 
  (12*S3L_FRACTIONS_PER_UNIT)/511, (18*S3L_FRACTIONS_PER_UNIT)/511, 
  (25*S3L_FRACTIONS_PER_UNIT)/511, (31*S3L_FRACTIONS_PER_UNIT)/511, 
  (37*S3L_FRACTIONS_PER_UNIT)/511, (43*S3L_FRACTIONS_PER_UNIT)/511, 
  (50*S3L_FRACTIONS_PER_UNIT)/511, (56*S3L_FRACTIONS_PER_UNIT)/511, 
  (62*S3L_FRACTIONS_PER_UNIT)/511, (68*S3L_FRACTIONS_PER_UNIT)/511, 
  (74*S3L_FRACTIONS_PER_UNIT)/511, (81*S3L_FRACTIONS_PER_UNIT)/511, 
  (87*S3L_FRACTIONS_PER_UNIT)/511, (93*S3L_FRACTIONS_PER_UNIT)/511, 
  (99*S3L_FRACTIONS_PER_UNIT)/511, (105*S3L_FRACTIONS_PER_UNIT)/511, 
  (111*S3L_FRACTIONS_PER_UNIT)/511, (118*S3L_FRACTIONS_PER_UNIT)/511, 
  (124*S3L_FRACTIONS_PER_UNIT)/511, (130*S3L_FRACTIONS_PER_UNIT)/511, 
  (136*S3L_FRACTIONS_PER_UNIT)/511, (142*S3L_FRACTIONS_PER_UNIT)/511, 
  (148*S3L_FRACTIONS_PER_UNIT)/511, (154*S3L_FRACTIONS_PER_UNIT)/511, 
  (160*S3L_FRACTIONS_PER_UNIT)/511, (166*S3L_FRACTIONS_PER_UNIT)/511, 
  (172*S3L_FRACTIONS_PER_UNIT)/511, (178*S3L_FRACTIONS_PER_UNIT)/511, 
  (183*S3L_FRACTIONS_PER_UNIT)/511, (189*S3L_FRACTIONS_PER_UNIT)/511, 
  (195*S3L_FRACTIONS_PER_UNIT)/511, (201*S3L_FRACTIONS_PER_UNIT)/511, 
  (207*S3L_FRACTIONS_PER_UNIT)/511, (212*S3L_FRACTIONS_PER_UNIT)/511, 
  (218*S3L_FRACTIONS_PER_UNIT)/511, (224*S3L_FRACTIONS_PER_UNIT)/511, 
  (229*S3L_FRACTIONS_PER_UNIT)/511, (235*S3L_FRACTIONS_PER_UNIT)/511, 
  (240*S3L_FRACTIONS_PER_UNIT)/511, (246*S3L_FRACTIONS_PER_UNIT)/511, 
  (251*S3L_FRACTIONS_PER_UNIT)/511, (257*S3L_FRACTIONS_PER_UNIT)/511, 
  (262*S3L_FRACTIONS_PER_UNIT)/511, (268*S3L_FRACTIONS_PER_UNIT)/511, 
  (273*S3L_FRACTIONS_PER_UNIT)/511, (278*S3L_FRACTIONS_PER_UNIT)/511, 
  (283*S3L_FRACTIONS_PER_UNIT)/511, (289*S3L_FRACTIONS_PER_UNIT)/511, 
  (294*S3L_FRACTIONS_PER_UNIT)/511, (299*S3L_FRACTIONS_PER_UNIT)/511, 
  (304*S3L_FRACTIONS_PER_UNIT)/511, (309*S3L_FRACTIONS_PER_UNIT)/511, 
  (314*S3L_FRACTIONS_PER_UNIT)/511, (319*S3L_FRACTIONS_PER_UNIT)/511, 
  (324*S3L_FRACTIONS_PER_UNIT)/511, (328*S3L_FRACTIONS_PER_UNIT)/511, 
  (333*S3L_FRACTIONS_PER_UNIT)/511, (338*S3L_FRACTIONS_PER_UNIT)/511, 
  (343*S3L_FRACTIONS_PER_UNIT)/511, (347*S3L_FRACTIONS_PER_UNIT)/511, 
  (352*S3L_FRACTIONS_PER_UNIT)/511, (356*S3L_FRACTIONS_PER_UNIT)/511, 
  (361*S3L_FRACTIONS_PER_UNIT)/511, (365*S3L_FRACTIONS_PER_UNIT)/511, 
  (370*S3L_FRACTIONS_PER_UNIT)/511, (374*S3L_FRACTIONS_PER_UNIT)/511, 
  (378*S3L_FRACTIONS_PER_UNIT)/511, (382*S3L_FRACTIONS_PER_UNIT)/511, 
  (386*S3L_FRACTIONS_PER_UNIT)/511, (391*S3L_FRACTIONS_PER_UNIT)/511, 
  (395*S3L_FRACTIONS_PER_UNIT)/511, (398*S3L_FRACTIONS_PER_UNIT)/511, 
  (402*S3L_FRACTIONS_PER_UNIT)/511, (406*S3L_FRACTIONS_PER_UNIT)/511, 
  (410*S3L_FRACTIONS_PER_UNIT)/511, (414*S3L_FRACTIONS_PER_UNIT)/511, 
  (417*S3L_FRACTIONS_PER_UNIT)/511, (421*S3L_FRACTIONS_PER_UNIT)/511, 
  (424*S3L_FRACTIONS_PER_UNIT)/511, (428*S3L_FRACTIONS_PER_UNIT)/511, 
  (431*S3L_FRACTIONS_PER_UNIT)/511, (435*S3L_FRACTIONS_PER_UNIT)/511, 
  (438*S3L_FRACTIONS_PER_UNIT)/511, (441*S3L_FRACTIONS_PER_UNIT)/511, 
  (444*S3L_FRACTIONS_PER_UNIT)/511, (447*S3L_FRACTIONS_PER_UNIT)/511, 
  (450*S3L_FRACTIONS_PER_UNIT)/511, (453*S3L_FRACTIONS_PER_UNIT)/511, 
  (456*S3L_FRACTIONS_PER_UNIT)/511, (459*S3L_FRACTIONS_PER_UNIT)/511, 
  (461*S3L_FRACTIONS_PER_UNIT)/511, (464*S3L_FRACTIONS_PER_UNIT)/511, 
  (467*S3L_FRACTIONS_PER_UNIT)/511, (469*S3L_FRACTIONS_PER_UNIT)/511, 
  (472*S3L_FRACTIONS_PER_UNIT)/511, (474*S3L_FRACTIONS_PER_UNIT)/511, 
  (476*S3L_FRACTIONS_PER_UNIT)/511, (478*S3L_FRACTIONS_PER_UNIT)/511, 
  (481*S3L_FRACTIONS_PER_UNIT)/511, (483*S3L_FRACTIONS_PER_UNIT)/511, 
  (485*S3L_FRACTIONS_PER_UNIT)/511, (487*S3L_FRACTIONS_PER_UNIT)/511, 
  (488*S3L_FRACTIONS_PER_UNIT)/511, (490*S3L_FRACTIONS_PER_UNIT)/511, 
  (492*S3L_FRACTIONS_PER_UNIT)/511, (494*S3L_FRACTIONS_PER_UNIT)/511, 
  (495*S3L_FRACTIONS_PER_UNIT)/511, (497*S3L_FRACTIONS_PER_UNIT)/511, 
  (498*S3L_FRACTIONS_PER_UNIT)/511, (499*S3L_FRACTIONS_PER_UNIT)/511, 
  (501*S3L_FRACTIONS_PER_UNIT)/511, (502*S3L_FRACTIONS_PER_UNIT)/511, 
  (503*S3L_FRACTIONS_PER_UNIT)/511, (504*S3L_FRACTIONS_PER_UNIT)/511, 
  (505*S3L_FRACTIONS_PER_UNIT)/511, (506*S3L_FRACTIONS_PER_UNIT)/511, 
  (507*S3L_FRACTIONS_PER_UNIT)/511, (507*S3L_FRACTIONS_PER_UNIT)/511, 
  (508*S3L_FRACTIONS_PER_UNIT)/511, (509*S3L_FRACTIONS_PER_UNIT)/511, 
  (509*S3L_FRACTIONS_PER_UNIT)/511, (510*S3L_FRACTIONS_PER_UNIT)/511, 
  (510*S3L_FRACTIONS_PER_UNIT)/511, (510*S3L_FRACTIONS_PER_UNIT)/511, 
  (510*S3L_FRACTIONS_PER_UNIT)/511, (510*S3L_FRACTIONS_PER_UNIT)/511
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

void TPE_initBody(TPE_Body *body)
{
  // TODO

  // init orientation to identity unit quaternion (1,0,0,0):

  body->orientation.x = TPE_FRACTIONS_PER_UNIT;
  body->orientation.y = 0;
  body->orientation.z = 0;
  body->orientation.w = 0;
}

static inline TPE_Unit TPE_nonZero(TPE_Unit x)
{
  return x + (x == 0);
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

void TPE_vec4Substract(const TPE_Vec4 a, const TPE_Vec4 b, TPE_Vec4 *result)
{
  result->x = a.x - b.x;
  result->y = a.y - b.y;
  result->z = a.z - b.z;
  result->w = a.w - b.w;
}

void TPE_vec3Multiplay(const TPE_Vec4 v, TPE_Unit f, TPE_Vec4 *result)
{
  result->x = (v.x * f) / TPE_FRACTIONS_PER_UNIT;
  result->y = (v.y * f) / TPE_FRACTIONS_PER_UNIT;
  result->z = (v.z * f) / TPE_FRACTIONS_PER_UNIT;
}

void TPE_vec4Multiplay(const TPE_Vec4 v, TPE_Unit f, TPE_Vec4 *result)
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

static inline TPE_Unit TPE_vec3DotProduct(const TPE_Vec4 v1, const TPE_Vec4 v2)
{
  return
    (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z) / TPE_FRACTIONS_PER_UNIT;
}

void TPE_vec3Normalize(TPE_Vec4 v)
{ 
  TPE_Unit l = TPE_vec3Len(v);

  if (l == 0)
  {
    v.x = TPE_FRACTIONS_PER_UNIT;
    return;
  }

  v.x = (v.x * TPE_FRACTIONS_PER_UNIT) / l;
  v.y = (v.y * TPE_FRACTIONS_PER_UNIT) / l;
  v.z = (v.z * TPE_FRACTIONS_PER_UNIT) / l;
}

void TPE_vec4Normalize(TPE_Vec4 v)
{ 
  TPE_Unit l = TPE_vec4Len(v);

  if (l == 0)
  {
    v.x = TPE_FRACTIONS_PER_UNIT;
    return;
  }

  v.x = (v.x * TPE_FRACTIONS_PER_UNIT) / l;
  v.y = (v.y * TPE_FRACTIONS_PER_UNIT) / l;
  v.z = (v.z * TPE_FRACTIONS_PER_UNIT) / l;
  v.w = (v.w * TPE_FRACTIONS_PER_UNIT) / l;
}

void TPE_vec3Project(const TPE_Vec4 v, const TPE_Vec4 base, TPE_Vec4 *result)
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
  const TPE_Vec4 collisionPoint,
  const TPE_Vec4 collisionNormal,
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



#endif // guard
