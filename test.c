#include "tinyphysicsengine.h"
#include <stdio.h>

#define F TPE_FRACTIONS_PER_UNIT

int testRotToQuat(
  TPE_Unit x, TPE_Unit y, TPE_Unit z, TPE_Unit angle,
  TPE_Unit expX, TPE_Unit expY, TPE_Unit expZ, TPE_Unit expW)
{
  printf("testing axis + rot -> quaternion ([%d,%d,%d] %d -> %d %d %d): ",
    x,y,z,angle,expW,expX,expY,expZ);

  TPE_Vec4 q, axis;

  TPE_setVec4(&axis,x,y,z,0);
  TPE_rotationToQuaternion(axis,angle,&q);

  #define TOLERANCE 10

  if (q.x > expX + TOLERANCE || q.x < expX - TOLERANCE ||
      q.y > expY + TOLERANCE || q.y < expY - TOLERANCE ||
      q.z > expZ + TOLERANCE || q.z < expZ - TOLERANCE ||
      q.w > expW + TOLERANCE || q.w < expW - TOLERANCE)
  {
    printf("%d %d %d %d, ERROR",q.x,q.y,q.z,q.w);
    return;
  }
  
  puts("OK");
  return 1;

  #undef TOLERANCE
}

int testQuatToEuler(TPE_Unit yaw, TPE_Unit pitch, TPE_Unit roll)
{

  TPE_Vec4 q, q2, axis;



  TPE_setVec4(&axis,0,0,F,0);

  TPE_rotationToQuaternion(axis,pitch,&q);

  

  TPE_Unit y,p,r;

  TPE_quaternionToEulerAngles(q,&y,&p,&r);

  printf("%d %d %d\n",y,p,r);
  
}

int main(void)
{
  TPE_Vec4 q1, q2, q3, axis;

  testRotToQuat(F,0,0,    0,    0,0,0,F);
  testRotToQuat(F,0,0,    F/4,  361,0,0,361);
  testRotToQuat(0,F,0,    F/4,  0,361,0,361);
  testRotToQuat(0,0,F,    F/2,  0,0,F,0);
  testRotToQuat(-F,F,F,   -F/8, 195,-195,-195,472);


  testQuatToEuler(0,F/2,0);

  return 0;
}
