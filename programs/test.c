/** General automatic test for tinyphysicsengine, it should always pass. */

#include "../tinyphysicsengine.h"
#include <stdio.h>

#define ass(cond,text) { printf(text ": "); if (!(cond)) { puts("ERROR"); return 1; } else puts("OK"); }

int main(void)
{
  puts("== testing tinyphysicsengine ==");

  {
    TPE_Unit l;

    l = TPE_LENGTH(TPE_vec3Normalized(TPE_vec3(100,0,0)));
    ass(TPE_abs(l - TPE_FRACTIONS_PER_UNIT) < 5,"vec3 normalize")

    l = TPE_LENGTH(TPE_vec3Normalized(TPE_vec3(0,0,0)));
    ass(TPE_abs(l - TPE_FRACTIONS_PER_UNIT) < 5,"zero vec3 normalize")

    l = TPE_LENGTH(TPE_vec3Normalized(TPE_vec3(0,-1,0)));
    ass(TPE_abs(l - TPE_FRACTIONS_PER_UNIT) < 5,"small vec3 normalize")

    l = TPE_LENGTH(TPE_vec3Normalized(TPE_vec3(500000,300000,-700000)));

    ass(TPE_abs(l - TPE_FRACTIONS_PER_UNIT) < 5,"big vec3 normalize")
  }

  {
    TPE_Joint joints[16];
    TPE_Connection cons[16];

    joints[0] = TPE_joint(TPE_vec3(200,100,-400),300);
    joints[1] = TPE_joint(TPE_vec3(100,200,-400),300);
    joints[2] = TPE_joint(TPE_vec3(200,-400,200),300);
    joints[3] = TPE_joint(TPE_vec3(200,100,-400),400);

    cons[0].joint1 = 0; cons[0].joint2 = 1; cons[0].length = 1000;
    cons[1].joint1 = 1; cons[1].joint2 = 0; cons[1].length = 1000;
    cons[2].joint1 = 0; cons[2].joint2 = 1; cons[2].length = 1100;
    cons[3].joint1 = 2; cons[3].joint2 = 3; cons[3].length = 100;

    uint32_t jHashes[4], cHashes[4];

    for (int i = 0; i < 4; ++i)
    {
      jHashes[i] = TPE_jointHash(&joints[i]);
      cHashes[i] = TPE_connectionHash(&cons[i]);
    }

    for (int i = 0; i < 4; ++i)
      for (int j = i + 1; j < 4; ++j)
        ass(jHashes[i] != jHashes[j] && cHashes[i] != cHashes[j],"joints/connection hash");

    TPE_Body bodies[8];
    uint32_t bHashes[4];

    TPE_bodyInit(&bodies[0],joints,4,cons,4,300);
    TPE_bodyInit(&bodies[1],joints + 1,3,cons,4,300);
    TPE_bodyInit(&bodies[2],joints,4,cons,4,300);
    bodies[2].flags |= TPE_BODY_FLAG_SOFT | TPE_BODY_FLAG_NONROTATING;
    TPE_bodyInit(&bodies[3],joints,4,cons,4,200);

    for (int i = 0; i < 4; ++i)
      bHashes[i] = TPE_bodyHash(&bodies[i]);

    for (int i = 0; i < 4; ++i)
      for (int j = i + 1; j < 4; ++j)
        ass(bHashes[i] != bHashes[j],"body hash");

    TPE_World world;

    uint32_t wHashes[4];

    TPE_worldInit(&world,bodies,4,0);

    wHashes[0] = TPE_worldHash(&world);
    bodies[0].jointCount--;
    wHashes[1] = TPE_worldHash(&world);
    bodies[4] = bodies[0]; bodies[0] = bodies[3]; bodies[3] = bodies[4];
    wHashes[2] = TPE_worldHash(&world);
    world.bodyCount--;
    wHashes[3] = TPE_worldHash(&world);

    for (int i = 0; i < 4; ++i)
      for (int j = i + 1; j < 4; ++j)
        ass(wHashes[i] != wHashes[j],"world hash");
  }


  return 0;
}