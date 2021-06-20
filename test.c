#include "tinyphysicsengine.h"
#include <stdio.h>

int main(void)
{



TPE_Unit 
  v1 = 6000,
  v2 = 0,
  m1 = 2000,
  m2 = 2000;


TPE_getVelocitiesAfterCollision(&v1,&v2,m1,m2,512);


TPE_Vec3 a, b;

a[0] = 1024;
a[1] = 0;
a[2] = 0;

b[0] = 512;
b[1] = 512;
b[2] = 0;


TPE_vec3Normalize(b);


TPE_vec3Project(a,b,a);


TPE_PRINTF_VEC3(a);




  return 0;
}
