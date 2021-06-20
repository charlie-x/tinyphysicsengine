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

typedef TPE_Unit TPE_Vec3[3];

typedef struct
{
  uint8_t shape;

  TPE_Unit shapeParams[TPE_MAX_SHAPE_PARAMS];  ///< parameters of the body type
  void *shapeParamPointers[TPE_MAX_SHAPE_PARAMPOINTERS]; ///< pointer parameters

  uint8_t flags;

  TPE_Unit position[3];     ///< position of the body's center of mass
  TPE_Unit mass;            /**< body mass, setting this to TPE_INFINITY will
                                 make the object static (not moving at all) 
                                 which may help performance */

  TPE_Unit velocity[0];

  TPE_Unit orientation[4];  ///< orientation as a quaternion


  TPE_Vec3 rotationAxis;    /**< normalized axis of rotation, direction of
                                 rotation is given by the right hand rule */

  TPE_Unit rotationSpeed;   /**< non-negative rotation speed around the 
                                 rotationAxis, TPE_FRACTIONS_PER_UNIT mean one
                                 rotation per one temporal unit (mathematically
                                 this could be represented as the length of 
                                 the rotationAxis vector, but for computational
                                 reasons it's better to have it this way) */
} TPE_Body;



#define TPE_PRINTF_VEC3(v) printf("[%d %d %d]",v[0],v[1],v[2]);

typedef struct
{
  uint16_t bodyCount;
  TPE_Body *bodies;  

} TPE_PhysicsWorld;

//------------------------------------------------------------------------------

static inline TPE_Unit TPE_nonZero(TPE_Unit x)
{
  return x + (x == 0);
}

void TPE_vec3Add(const TPE_Vec3 a, const TPE_Vec3 b, TPE_Vec3 result)
{
  result[0] = a[0] + b[0];
  result[1] = a[1] + b[1];
  result[2] = a[2] + b[2];
}

TPE_Unit TPE_vec3Len(TPE_Vec3 v)
{
  return TPE_sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static inline TPE_Unit TPE_vec3DotProduct(const TPE_Vec3 v1, const TPE_Vec3 v2)
{
  return
    (v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]) / TPE_FRACTIONS_PER_UNIT;
}

void TPE_vec3Normalize(TPE_Vec3 v)
{ 
  TPE_Unit l = TPE_vec3Len(v);

  if (l == 0)
  {
    v[0] = TPE_FRACTIONS_PER_UNIT;
    return;
  }

  v[0] = (v[0] * TPE_FRACTIONS_PER_UNIT) / l;
  v[1] = (v[1] * TPE_FRACTIONS_PER_UNIT) / l;
  v[2] = (v[2] * TPE_FRACTIONS_PER_UNIT) / l;
}

void TPE_vec3Project(const TPE_Vec3 v, const TPE_Vec3 base, TPE_Vec3 result)
{
  TPE_Unit p = TPE_vec3DotProduct(v,base);

printf("%d\n",p);

  result[0] = (p * base[0]) / TPE_FRACTIONS_PER_UNIT;
  result[1] = (p * base[1]) / TPE_FRACTIONS_PER_UNIT;
  result[2] = (p * base[2]) / TPE_FRACTIONS_PER_UNIT;
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

void TPE_resolvePointCollision(
  const TPE_Vec3 collisionPoint,
  const TPE_Vec3 collisionNormal,
  TPE_Vec3 linVelocity1,
  TPE_Vec3 rotVelocity1,
  TPE_Unit m1,
  TPE_Vec3 linVelocity2,
  TPE_Vec3 rotVelocity2,
  TPE_Unit m2)
{
  TPE_Vec3 v1, v2;
  
  TPE_vec3Add(linVelocity1,rotVelocity1,v1);
  TPE_vec3Add(linVelocity2,rotVelocity2,v2);

// TODO
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


#endif // guard
