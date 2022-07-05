#define TPE_LOG puts

#include "tinyphysicsengine.h"
#include <SDL2/SDL.h>
#include <math.h>

#define S3L_RESOLUTION_X 640
#define S3L_RESOLUTION_Y 480
#define S3L_PIXEL_FUNCTION drawPixel

#define S3L_Z_BUFFER 1
#include "small3dlib.h"

#define PIXELS_SIZE (S3L_RESOLUTION_X * S3L_RESOLUTION_Y * 4)

#define FPS 30
#define MSPF (1000 / (FPS))

TPE_World world;
TPE_Body bodies[128];

S3L_Unit cubeVertices[] = { S3L_CUBE_VERTICES(1600) };  
S3L_Index cubeTriangles[] = { S3L_CUBE_TRIANGLES };
S3L_Model3D cube;

uint8_t pixels[PIXELS_SIZE];

uint8_t red = 100;

TPE_Vec3 environmentDistance(TPE_Vec3 p)
{
/*
TODO: This function should have another parameter "distance", if the closest
point should be further away than this distance, it won't matter and in that
case the function may return any arbitrary point further away than "distance",
this can speed up detections in vast empty areas.
*/

#define WWW 3000

  if (p.x < -WWW || p.x > WWW || p.y < -WWW || p.y > WWW ||
      p.z < -WWW || p.z > WWW)
    return p; 

  int xx = p.x * p.x;  
  int yy = p.y * p.y;  
  int zz = p.z * p.z;

  if (xx > yy)
  {
    if (xx > zz)
      p.x = p.x > 0 ? WWW : -WWW;
    else
      p.z = p.z > 0 ? WWW : -WWW;
  }
  else
  {
    if (yy > zz)
      p.y = p.y > 0 ? WWW : -WWW;
    else
      p.z = p.z > 0 ? WWW : -WWW;
  }

#undef WWW
  return p;
}

void drawPixel(S3L_PixelInfo *p)
{
  uint32_t index = (p->y * S3L_RESOLUTION_X + p->x) * 4;
  pixels[index + 1] = p->triangleIndex * 16;
  pixels[index + 2] = 255 - p->triangleIndex * 16;
  pixels[index + 3] = red;
}

void draw2DPoint(int x, int y, int r, int g, int b)
{
  if (x < 1 || x > S3L_RESOLUTION_X - 3 ||
      y < 1 || y > S3L_RESOLUTION_Y - 3)
    return;

  uint32_t index = ((y - 1) * S3L_RESOLUTION_X + x) * 4;

  #define d pixels[index] = 0; pixels[index + 1] = b; pixels[index + 2] = g; pixels[index + 3] = r;

  d
  index += S3L_RESOLUTION_X * 4 - 4;
  d
  index += 4;
  d
  index += 4;
  d
  index += S3L_RESOLUTION_X * 4 - 4;
  d

  #undef d
}

void drawLine(int x1, int y1, int x2, int y2, int r, int g, int b)
{
  #define STEPS 20

  float dx = (x2 - x1) / ((float) STEPS);
  float dy = (y2 - y1) / ((float) STEPS);

  for (int i = 0; i < STEPS; ++i)
    draw2DPoint(x1 + dx * i, y1 + dy * i,r,g,b);

  #undef STEPS
}

S3L_Scene sphereScene;

void draw3DLine(int x1, int y1, int z1, int x2, int y2, int z2)
{
  S3L_Vec4 p1, p2, r1, r2;

  S3L_vec4Set(&p1,x1,y1,z1,0);
  S3L_vec4Set(&p2,x2,y2,z2,0);

  S3L_project3DPointToScreen(p1,sphereScene.camera,&r1);
  S3L_project3DPointToScreen(p2,sphereScene.camera,&r2);

  if (r1.z > 0 && r2.z > 0)
    drawLine(r1.x,r1.y,r2.x,r2.y,200,100,200);
}

void drawSphere(S3L_Unit x, S3L_Unit y, S3L_Unit z, S3L_Unit r)
{
  sphereScene.models[0].transform.translation.x = x;
  sphereScene.models[0].transform.translation.y = y;
  sphereScene.models[0].transform.translation.z = z;
  sphereScene.models[0].transform.scale.x = r;
  sphereScene.models[0].transform.scale.y = r;
  sphereScene.models[0].transform.scale.z = r;
  S3L_drawScene(sphereScene);
}

void drawBody(TPE_Body *body, uint8_t color)
{
  red = color;

  for (int i = 0; i < body->jointCount; ++i)
    drawSphere(
      body->joints[i].position.x,
      body->joints[i].position.y,
      body->joints[i].position.z,
      TPE_JOINT_SIZE(body->joints[i]));

  for (int i = 0; i < body->connectionCount; ++i)
  {
    S3L_Vec4 p1, p2, r1, r2;

    S3L_vec4Set(&p1,
      body->joints[body->connections[i].joint1].position.x,
      body->joints[body->connections[i].joint1].position.y,
      body->joints[body->connections[i].joint1].position.z,0);

    S3L_vec4Set(&p2,
      body->joints[body->connections[i].joint2].position.x,
      body->joints[body->connections[i].joint2].position.y,
      body->joints[body->connections[i].joint2].position.z,0);

    S3L_project3DPointToScreen(p1,sphereScene.camera,&r1);
    S3L_project3DPointToScreen(p2,sphereScene.camera,&r2);

    if (r1.z > 0 && r2.z > 0)
      drawLine(r1.x,r1.y,r2.x,r2.y,100,200,300);  
  }
}

void drawEnv(TPE_Vec3 p, int stepLength, int steps)
{
  TPE_Vec3 p2 = p;

  for (int k = 0; k < steps; ++k)
  {
    p2.y = p.y;

    for (int j = 0; j < steps; ++j)
    {
      p2.x = p.x;

      for (int i = 0; i < steps; ++i)
      {
        TPE_Vec3 p3 = environmentDistance(p2);

        S3L_Vec4 p4, p5;
        p4.x = p3.x;
        p4.y = p3.y;
        p4.z = p3.z;

        S3L_project3DPointToScreen(p4,sphereScene.camera,&p5);

        draw2DPoint(p5.x,p5.y,100,200,255);

        p2.x += stepLength;
      }

      p2.y += stepLength;
    }

    p2.z += stepLength;
  }
}

#define SPHERE_VERTEX_COUNT 42
const S3L_Unit sphereVertices[SPHERE_VERTEX_COUNT * 3] = {
      0,  -512,     0,        // 0
    370,  -228,  -269,        // 3
   -141,  -228,  -435,        // 6
   -457,  -228,     0,        // 9
   -141,  -228,   435,        // 12
    370,  -228,   269,        // 15
    141,   228,  -435,        // 18
   -370,   228,  -269,        // 21
   -370,   228,   269,        // 24
    141,   228,   435,        // 27
    457,   228,     0,        // 30
      0,   512,     0,        // 33
    -83,  -435,  -255,        // 36
    217,  -435,  -158,        // 39
    134,  -269,  -414,        // 42
    435,  -269,     0,        // 45
    217,  -435,   158,        // 48
   -269,  -435,     0,        // 51
   -352,  -269,  -255,        // 54
    -83,  -435,   255,        // 57
   -352,  -269,   255,        // 60
    134,  -269,   414,        // 63
    486,     0,  -158,        // 66
    486,     0,   158,        // 69
      0,     0,  -512,        // 72
    300,     0,  -414,        // 75
   -486,     0,  -158,        // 78
   -300,     0,  -414,        // 81
   -300,     0,   414,        // 84
   -486,     0,   158,        // 87
    300,     0,   414,        // 90
      0,     0,   512,        // 93
    352,   269,  -255,        // 96
   -134,   269,  -414,        // 99
   -435,   269,     0,        // 102
   -134,   269,   414,        // 105
    352,   269,   255,        // 108
     83,   435,  -255,        // 111
    269,   435,     0,        // 114
   -217,   435,  -158,        // 117
   -217,   435,   158,        // 120
     83,   435,   255         // 123
}; // sphereVertices

#define SPHERE_TRIANGLE_COUNT 80
const S3L_Index sphereTriangleIndices[SPHERE_TRIANGLE_COUNT * 3] = {
      0,    13,    12,        // 0
      1,    13,    15,        // 3
      0,    12,    17,        // 6
      0,    17,    19,        // 9
      0,    19,    16,        // 12
      1,    15,    22,        // 15
      2,    14,    24,        // 18
      3,    18,    26,        // 21
      4,    20,    28,        // 24
      5,    21,    30,        // 27
      1,    22,    25,        // 30
      2,    24,    27,        // 33
      3,    26,    29,        // 36
      4,    28,    31,        // 39
      5,    30,    23,        // 42
      6,    32,    37,        // 45
      7,    33,    39,        // 48
      8,    34,    40,        // 51
      9,    35,    41,        // 54
     10,    36,    38,        // 57
     38,    41,    11,        // 60
     38,    36,    41,        // 63
     36,     9,    41,        // 66
     41,    40,    11,        // 69
     41,    35,    40,        // 72
     35,     8,    40,        // 75
     40,    39,    11,        // 78
     40,    34,    39,        // 81
     34,     7,    39,        // 84
     39,    37,    11,        // 87
     39,    33,    37,        // 90
     33,     6,    37,        // 93
     37,    38,    11,        // 96
     37,    32,    38,        // 99
     32,    10,    38,        // 102
     23,    36,    10,        // 105
     23,    30,    36,        // 108
     30,     9,    36,        // 111
     31,    35,     9,        // 114
     31,    28,    35,        // 117
     28,     8,    35,        // 120
     29,    34,     8,        // 123
     29,    26,    34,        // 126
     26,     7,    34,        // 129
     27,    33,     7,        // 132
     27,    24,    33,        // 135
     24,     6,    33,        // 138
     25,    32,     6,        // 141
     25,    22,    32,        // 144
     22,    10,    32,        // 147
     30,    31,     9,        // 150
     30,    21,    31,        // 153
     21,     4,    31,        // 156
     28,    29,     8,        // 159
     28,    20,    29,        // 162
     20,     3,    29,        // 165
     26,    27,     7,        // 168
     26,    18,    27,        // 171
     18,     2,    27,        // 174
     24,    25,     6,        // 177
     24,    14,    25,        // 180
     14,     1,    25,        // 183
     22,    23,    10,        // 186
     22,    15,    23,        // 189
     15,     5,    23,        // 192
     16,    21,     5,        // 195
     16,    19,    21,        // 198
     19,     4,    21,        // 201
     19,    20,     4,        // 204
     19,    17,    20,        // 207
     17,     3,    20,        // 210
     17,    18,     3,        // 213
     17,    12,    18,        // 216
     12,     2,    18,        // 219
     15,    16,     5,        // 222
     15,    13,    16,        // 225
     13,     0,    16,        // 228
     12,    14,     2,        // 231
     12,    13,    14,        // 234
     13,     1,    14         // 237
}; // sphereTriangleIndices

int main(void)
{
  SDL_Window *window = SDL_CreateWindow("test", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, S3L_RESOLUTION_X, S3L_RESOLUTION_Y, SDL_WINDOW_SHOWN); 
  SDL_Renderer *renderer = SDL_CreateRenderer(window,-1,0);
  SDL_Texture *textureSDL = SDL_CreateTexture(renderer,SDL_PIXELFORMAT_RGBX8888, SDL_TEXTUREACCESS_STATIC, S3L_RESOLUTION_X, S3L_RESOLUTION_Y);
  SDL_Surface *screenSurface = SDL_GetWindowSurface(window);
  SDL_Event event;

  int running = 1;

  S3L_Model3D sphereModel;

  S3L_model3DInit(sphereVertices,SPHERE_VERTEX_COUNT,sphereTriangleIndices,
    SPHERE_TRIANGLE_COUNT,&sphereModel);

  S3L_model3DInit(cubeVertices,S3L_CUBE_VERTEX_COUNT,cubeTriangles,
    S3L_CUBE_TRIANGLE_COUNT,&cube);

  S3L_sceneInit(&sphereModel,1,&sphereScene);
  
  sphereScene.camera.transform.translation.z = -3000;

  int frame = 0;

TPE_Joint joints[100];
TPE_Connection connections[100];

switch (3)
{
  case 0:
    TPE_make2Line(joints,connections,1500,512);
    TPE_bodyInit(bodies,joints,2,connections,1,100);
    break;

  case 1:
    TPE_makeBox(joints,connections,1300,2000,3000,512);
    TPE_bodyInit(bodies,joints,8,connections,16,100);
    break;

  case 2:
    TPE_makeCenterRect(joints,connections,1300,1000,512);
    TPE_bodyInit(bodies,joints,5,connections,8,100);
    break;

  case 3:
    TPE_makeCenterBox(joints,connections,1000,1000,1000,512);
    joints[8].sizeDivided *= 3;
    joints[8].sizeDivided /= 2;
    TPE_bodyInit(bodies,joints,9,connections,18,100);
    break;

  case 4:
    TPE_makeTriangle(joints,connections,2000,512);
    TPE_bodyInit(bodies,joints,3,connections,3,100);
    break;

  default: break;
}

//TPE_makeBox(joints + 20,connections + 20,300,128);

//TPE_bodyInit(bodies,joints,1,connections,0,100);

//bodies[0].flags |= TPE_BODY_FLAG_SOFT;



TPE_worldInit(&world,bodies,1,environmentDistance);


TPE_bodyMove(world.bodies,TPE_vec3(-800,-300,0));
TPE_bodyMove(&world.bodies[1],TPE_vec3(400,100,1));

TPE_bodyStop(world.bodies);
TPE_bodyStop(world.bodies + 1);

//TPE_bodyRotate(world.bodies,TPE_vec3(0,0,200));




  //-------

  int time;

  while (running)
  {

    time = SDL_GetTicks();

    for (uint32_t i = 0; i < PIXELS_SIZE; ++i)
      pixels[i] = 0;

    S3L_newFrame();

TPE_Unit m = TPE_bodyAverageSpeed(world.bodies); //TPE_bodyNetSpeed(world.bodies);

printf("%d\n",m);


//TPE_bodiesResolveCollision(world.bodies,world.bodies + 1);


for (int i = 0; i < world.bodyCount; ++i)
{




TPE_bodyAccelerate(world.bodies + i,
TPE_vec3(0,-6,0));


}

TPE_worldStep(&world);

m /= 16;




    while (SDL_PollEvent(&event))
    {
      if (event.type == SDL_QUIT)
        running = 0;
      else if (event.type == SDL_KEYDOWN)
      {
        if (event.key.keysym.scancode == SDL_SCANCODE_Q || event.key.keysym.scancode == SDL_SCANCODE_ESCAPE)
          running = 0;
      }
    }

    const uint8_t *state = SDL_GetKeyboardState(NULL);

    S3L_Vec4 camF, camR;
 
#define SHIFT_STEP 50
#define ROT_STEP 5

   S3L_rotationToDirections(sphereScene.camera.transform.rotation,SHIFT_STEP,&camF,&camR,0);

TPE_Vec3 forw = TPE_vec3Minus( 
  bodies[0].joints[2].position,
  bodies[0].joints[0].position);

TPE_Vec3 righ = TPE_vec3Minus( 
  bodies[0].joints[0].position,
  bodies[0].joints[1].position);

TPE_Vec3 rrrr = TPE_orientationFromVecs(forw,righ);

cube.transform.rotation.x = rrrr.x;
cube.transform.rotation.y = rrrr.y;
cube.transform.rotation.z = rrrr.z;

TPE_Vec3 ppp = TPE_bodyGetCenter(&bodies[0]);

cube.transform.translation.x = ppp.x;
cube.transform.translation.y = ppp.y;
cube.transform.translation.z = ppp.z;


drawEnv(TPE_vec3(-100,-100,-100),100,5);

for (int i = 0; i < world.bodyCount; ++i)
  drawBody(&(world.bodies[i]),100 * i);


sphereScene.models = &cube;
S3L_newFrame();
S3L_drawScene(sphereScene);
sphereScene.models = &sphereModel;

draw3DLine(0,0,0,forw.x,forw.y,forw.z);
draw3DLine(0,0,0,righ.x,righ.y,righ.z);

    SDL_UpdateTexture(textureSDL,NULL,pixels,S3L_RESOLUTION_X * sizeof(uint32_t));



    if (state[SDL_SCANCODE_LSHIFT])
    {
      if (state[SDL_SCANCODE_UP])
        S3L_vec3Add(&sphereScene.camera.transform.translation,camF);
      else if (state[SDL_SCANCODE_DOWN])
        S3L_vec3Sub(&sphereScene.camera.transform.translation,camF);
      else if (state[SDL_SCANCODE_LEFT])
        S3L_vec3Sub(&sphereScene.camera.transform.translation,camR);
      else if (state[SDL_SCANCODE_RIGHT])
        S3L_vec3Add(&sphereScene.camera.transform.translation,camR);
    }
    else
    {
      if (state[SDL_SCANCODE_UP])
        sphereScene.camera.transform.rotation.x += ROT_STEP;
      else if (state[SDL_SCANCODE_DOWN])
        sphereScene.camera.transform.rotation.x -= ROT_STEP;
      else if (state[SDL_SCANCODE_LEFT])
        sphereScene.camera.transform.rotation.y += ROT_STEP;
      else if (state[SDL_SCANCODE_RIGHT])
        sphereScene.camera.transform.rotation.y -= ROT_STEP;
      else if (state[SDL_SCANCODE_K])
        sphereScene.camera.transform.rotation.z += ROT_STEP;
      else if (state[SDL_SCANCODE_L])
        sphereScene.camera.transform.rotation.z -= ROT_STEP;
    }

if (state[SDL_SCANCODE_M])
{
  TPE_bodyWake(world.bodies);

  TPE_bodySpin(bodies,TPE_vec3(50,40,100));

  TPE_bodyAccelerate(bodies,TPE_vec3(
   500,
   500,
   30));
}

#define SHIFT_STEP 50

    if (state[SDL_SCANCODE_P])
      sphereScene.camera.transform.translation.y += SHIFT_STEP;
    else if (state[SDL_SCANCODE_O])
      sphereScene.camera.transform.translation.y -= SHIFT_STEP;

#undef SHIFT_STEP

    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer,textureSDL,NULL,NULL);
    SDL_RenderPresent(renderer);

    time = time + MSPF - SDL_GetTicks();

    if (time > 1)
      usleep(time * 1000);

    frame++;
  }

  return 0;
}
