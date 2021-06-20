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

printf("%d %d\n",v1,v2);


  return 0;
}
