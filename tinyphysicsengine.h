#ifndef TINYPHYSICSENGINE_H
#define TINYPHYSICSENGINE_H

/*
  author: Miloslav Ciz
  license: CC0 1.0 (public domain)
           found at https://creativecommons.org/publicdomain/zero/1.0/
           + additional waiver of all IP
  version: 0.1d
*/


#include <stdint.h>


typedef int32_t TPE_Unit;

/** How many fractions a spatial or temporal unit is split into. This is NOT
  SUPPOSED TO BE REDEFINED, so rather don't do it (otherwise things may
  overflow etc.). */
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
  uint8_t shape;

  TPE_Unit shapeParams[TPE_MAX_SHAPE_PARAMS];  ///< parameters of the body type
  void *shapeParamPointers[TPE_MAX_SHAPE_PARAMPOINTERS]; ///< pointer parameters

  uint8_t flags;

  TPE_Unit position[3];     ///< position of the body's center of mass
  TPE_Unit mass;            /**< body mass, setting this to TPE_INFINITY will
                                 make the object static (not moving at all) 
                                 which may help performance */

  TPE_Unit velocity[3];

  TPE_Unit orientation[4];  ///< orientation as a quaternion

  TPE_Unit rotation[4];     /**< current rotation state, first 3 numbers
                                 specify the axis of rotation, the 4th says the
                                 angular velocity */
} TPE_Body;




typedef struct
{
  uint16_t bodyCount;
  TPE_Body *bodies;  

} TPE_PhysicsWorld;


#endif // guard
