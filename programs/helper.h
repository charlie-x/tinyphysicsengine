/**
  Helper code for tinyphysicsengine example programs which handle things like
  SDL rendering, I/O, small3dlib rendering etc.
*/

#define TPE_LOG puts

#include "../tinyphysicsengine.h"
#include <SDL2/SDL.h>
#include <math.h>
#include <sys/time.h> // for measuring time

#ifndef RES_X
  #define RES_X 640
#endif

#ifndef RES_Y
  #define RES_Y 480
#endif

#ifndef FPS
  #define FPS 30
#endif

#ifndef CAMERA_STEP
  #define CAMERA_STEP 100
#endif

#ifndef CAMERA_ROT_STEP
  #define CAMERA_ROT_STEP 5
#endif

#define MSPF (1000 / (FPS))
#define S3L_RESOLUTION_X RES_X
#define S3L_RESOLUTION_Y RES_Y

#define S3L_PIXEL_FUNCTION s3l_drawPixel

#ifndef S3L_SORT
  #define S3L_SORT 0
#endif

#ifndef S3L_Z_BUFFER
  #define S3L_Z_BUFFER 1
#endif

#define S3L_NEAR_CROSS_STRATEGY 2
#define S3L_USE_WIDER_TYPES 1
#include "small3dlib.h"

#define PIXELS_SIZE (S3L_RESOLUTION_X * S3L_RESOLUTION_Y * 4)

S3L_Unit cubeVertices[] = { S3L_CUBE_VERTICES(TPE_FRACTIONS_PER_UNIT) };  
S3L_Index cubeTriangles[] = { S3L_CUBE_TRIANGLES };
S3L_Model3D cubeModel;

S3L_Unit triangleVertices[9];
S3L_Index triangleTriangles[] = {0, 1, 2, 0, 2, 1};
S3L_Model3D triangleModel;

S3L_Unit planeVerices[] = 
{
#define a S3L_FRACTIONS_PER_UNIT / 2
#define b -1 * S3L_FRACTIONS_PER_UNIT / 2
  a,0,a,
  a,0,b,
  b,0,a,
  b,0,b
#undef a
#undef b
};

S3L_Index planeTriangles[] = 
{ 0,2,1, 1,2,3 };
S3L_Model3D planeModel;

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

S3L_Model3D sphereModel;

#define MAX_BODIES 128
#define MAX_JOINTS 1024
#define MAX_CONNECTIONS 2048

TPE_Body tpe_bodies[MAX_BODIES];
TPE_Joint tpe_joints[MAX_JOINTS];
TPE_Connection tpe_connections[MAX_CONNECTIONS];

int helper_debugDrawOn = 0,
    helper_debugDrawOnCountdown = 0;

unsigned int 
  helper_jointsUsed = 0,
  helper_connectionsUsed = 0;

TPE_World tpe_world;

uint8_t sdl_pixels[PIXELS_SIZE];

SDL_Window *sdl_window;
SDL_Renderer *sdl_renderer;
SDL_Texture *sdl_texture;
SDL_Surface *sdl_screenSurface;

int helper_frameStartTime;
int helper_frameMsLeft;
int helper_running;
int helper_frame;
const uint8_t *sdl_keyboard;

S3L_Scene s3l_scene;

S3L_Vec4 helper_cameraForw, helper_cameraRight, helper_cameraUp;

unsigned long helper_getMicroSecs(void)
{
  struct timeval t;
  gettimeofday(&t,NULL);
  return 1000000 * t.tv_sec + t.tv_usec;
}

void _helper_bodyAdded(int joints, int conns, TPE_Unit mass)
{
  TPE_bodyInit(&tpe_bodies[tpe_world.bodyCount],
    &tpe_joints[helper_jointsUsed],joints,
    &tpe_connections[helper_connectionsUsed],conns,mass);
 
  helper_jointsUsed += joints;
  helper_connectionsUsed += conns;

  tpe_world.bodyCount++;
}

void helper_addBox(TPE_Unit w, TPE_Unit h, TPE_Unit d, TPE_Unit jointSize, TPE_Unit mass)
{
  TPE_makeBox(
    tpe_joints + helper_jointsUsed,
    tpe_connections + helper_connectionsUsed,w,h,d,jointSize);

  _helper_bodyAdded(8,16,mass);
}

void helper_add2Line(TPE_Unit w, TPE_Unit jointSize, TPE_Unit mass)
{
  TPE_make2Line(
    tpe_joints + helper_jointsUsed,
    tpe_connections + helper_connectionsUsed,w,jointSize);

  _helper_bodyAdded(2,1,mass);
}

void helper_addTriangle(TPE_Unit s, TPE_Unit d, TPE_Unit mass)
{
  TPE_makeTriangle(
    tpe_joints + helper_jointsUsed,
    tpe_connections + helper_connectionsUsed,s,d);

  _helper_bodyAdded(3,3,mass);
}

void helper_addRect(TPE_Unit w, TPE_Unit d, TPE_Unit jointSize, TPE_Unit mass)
{
  TPE_makeRect(
    tpe_joints + helper_jointsUsed,
    tpe_connections + helper_connectionsUsed,w,d,jointSize);

  _helper_bodyAdded(4,6,mass);
}

void helper_addBall(TPE_Unit s, TPE_Unit mass)
{
  tpe_joints[helper_jointsUsed] = TPE_joint(TPE_vec3(0,0,0),s);

  _helper_bodyAdded(1,0,mass);
}

void helper_printCamera(void)
{
  printf("camera: %ld %ld %ld (%ld %ld %ld)\n",
    s3l_scene.camera.transform.translation.x,
    s3l_scene.camera.transform.translation.y,
    s3l_scene.camera.transform.translation.z,
    s3l_scene.camera.transform.rotation.x,
    s3l_scene.camera.transform.rotation.y,
    s3l_scene.camera.transform.rotation.z);
}

void helper_printCPU(void)
{
  printf("CPU (%d FPS): %d \n",FPS,((MSPF - helper_frameMsLeft) * 100) / MSPF);
}

void helper_cameraFreeMovement(void)
{
  if (sdl_keyboard[SDL_SCANCODE_LSHIFT])
  {
    if (sdl_keyboard[SDL_SCANCODE_UP])
      S3L_vec3Add(&s3l_scene.camera.transform.translation,helper_cameraForw);
    else if (sdl_keyboard[SDL_SCANCODE_DOWN])
      S3L_vec3Sub(&s3l_scene.camera.transform.translation,helper_cameraForw);
    else if (sdl_keyboard[SDL_SCANCODE_LEFT])
      S3L_vec3Sub(&s3l_scene.camera.transform.translation,helper_cameraRight);
    else if (sdl_keyboard[SDL_SCANCODE_RIGHT])
      S3L_vec3Add(&s3l_scene.camera.transform.translation,helper_cameraRight);
  }
  else
  {
    if (sdl_keyboard[SDL_SCANCODE_UP])
      s3l_scene.camera.transform.rotation.x += CAMERA_ROT_STEP;
    else if (sdl_keyboard[SDL_SCANCODE_DOWN])
      s3l_scene.camera.transform.rotation.x -= CAMERA_ROT_STEP;
    else if (sdl_keyboard[SDL_SCANCODE_LEFT])
      s3l_scene.camera.transform.rotation.y += CAMERA_ROT_STEP;
    else if (sdl_keyboard[SDL_SCANCODE_RIGHT])
      s3l_scene.camera.transform.rotation.y -= CAMERA_ROT_STEP;
    else if (sdl_keyboard[SDL_SCANCODE_Q])
      s3l_scene.camera.transform.rotation.z += CAMERA_ROT_STEP;
    else if (sdl_keyboard[SDL_SCANCODE_E])
      s3l_scene.camera.transform.rotation.z -= CAMERA_ROT_STEP;
  }
}

void sdl_drawPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
  uint8_t *pixel = sdl_pixels + (y * S3L_RESOLUTION_X + x) * 4 + 1;

  *pixel = b;
  pixel++;
  *pixel = g;
  pixel++;
  *pixel = r;
}

void helper_drawLine2D(int x1, int y1, int x2, int y2, uint8_t r, uint8_t g,
  uint8_t b)
{
  // stupid algorithm

  x2 -= x1;
  y2 -= y1;

  int max = (x2 * x2 > y2 * y2) ? x2 : y2;

  if (max < 0)
    max *= -1;

  for (int i = 0; i < max; ++i)
    sdl_drawPixel(x1 + (x2 * i) / max,y1 + (y2 * i) / max,r,g,b);
}

void helper_drawLine3D(TPE_Vec3 p1, TPE_Vec3 p2, uint8_t rr, uint8_t gg,
  uint8_t bb)
{
  S3L_Vec4 a, b, c, d;

  a.x = p1.x; a.y = p1.y; a.z = p1.z; a.w = 0;
  b.x = p2.x; b.y = p2.y; b.z = p2.z; b.w = 0;

  S3L_project3DPointToScreen(a,s3l_scene.camera,&c);
  S3L_project3DPointToScreen(b,s3l_scene.camera,&d);
  
  if (c.x >= 0 && c.x < S3L_RESOLUTION_X && c.y >= 0 && c.y < S3L_RESOLUTION_Y && c.z > 0 &&
      d.x >= 0 && d.x < S3L_RESOLUTION_X && d.y >= 0 && d.y < S3L_RESOLUTION_Y && d.z > 0)
    helper_drawLine2D(c.x,c.y,d.x,d.y,rr,gg,bb);
}

void tpe_debugDrawPixel(uint16_t x, uint16_t y, uint8_t color)
{
  if (x < S3L_RESOLUTION_X - 2 && y < S3L_RESOLUTION_Y - 2)
  {
    uint8_t r, g, b;

    switch (color)
    {
      case 0:  r = 100; g = 255; b = 200; break;
      case 1:  r = 255; g = 100; b = 100; break;
      case 2:  r = 0; g = 50; b = 50; break;
      default: r = 0; g = 0; b = 0; break;
    }

    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        sdl_drawPixel(x + i,y + j,r,g,b);
  }
}

void helper_debugDraw(void)
{
  TPE_Vec3 camPos = 
    TPE_vec3(
      s3l_scene.camera.transform.translation.x,
      s3l_scene.camera.transform.translation.y,
      s3l_scene.camera.transform.translation.z);

  TPE_Vec3 camRot = 
    TPE_vec3(
      s3l_scene.camera.transform.rotation.x,
      s3l_scene.camera.transform.rotation.y,
      s3l_scene.camera.transform.rotation.z);

  TPE_worldDebugDraw(&tpe_world,tpe_debugDrawPixel,camPos,camRot,
    TPE_vec3(S3L_RESOLUTION_X,S3L_RESOLUTION_Y,s3l_scene.camera.focalLength),
    16,256);
}

uint8_t s3l_r = 0, s3l_g = 255, s3l_b = 0;
uint8_t s3l_rr = 0, s3l_gg = 255, s3l_bb = 0; 

int s3l_previousTriangleID = -1;

void s3l_drawPixel(S3L_PixelInfo *p)
{
  if (p->triangleIndex != s3l_previousTriangleID)
  {
    s3l_rr = s3l_r + ((p->triangleIndex * 5) % 128) * (((s3l_r < 128) * 2) - 1);
    s3l_gg = s3l_g + ((p->triangleIndex * 3) % 128) * (((s3l_g < 128) * 2) - 1);
    s3l_bb = s3l_b + ((p->triangleIndex * 7) % 128) * (((s3l_b < 128) * 2) - 1);

    s3l_previousTriangleID = p->triangleIndex;
  }

  sdl_drawPixel(p->x,p->y,s3l_rr,s3l_gg,s3l_bb);
}

void helper_set3dColor(uint8_t r, uint8_t g, uint8_t b)
{
  s3l_r = r;
  s3l_g = g;
  s3l_b = b;
}

void helper_drawModel(S3L_Model3D *model, TPE_Vec3 pos, TPE_Vec3 scale, 
  TPE_Vec3 rot)
{
  s3l_previousTriangleID = -1;

  model->transform.translation.x = pos.x;  
  model->transform.translation.y = pos.y;  
  model->transform.translation.z = pos.z;

  model->transform.scale.x = scale.x;
  model->transform.scale.y = scale.y;
  model->transform.scale.z = scale.z;

  model->transform.rotation.x = rot.x;  
  model->transform.rotation.y = rot.y;  
  model->transform.rotation.z = rot.z;  

  s3l_scene.models = model;

  S3L_drawScene(s3l_scene);
}

void helper_draw3dTriangle(TPE_Vec3 v1, TPE_Vec3 v2, TPE_Vec3 v3)
{
  triangleVertices[0] = v1.x; 
  triangleVertices[1] = v1.y; 
  triangleVertices[2] = v1.z; 
  triangleVertices[3] = v2.x; 
  triangleVertices[4] = v2.y; 
  triangleVertices[5] = v2.z; 
  triangleVertices[6] = v3.x; 
  triangleVertices[7] = v3.y; 
  triangleVertices[8] = v3.z;

  helper_drawModel(&triangleModel,TPE_vec3(0,0,0),
    TPE_vec3(S3L_FRACTIONS_PER_UNIT,S3L_FRACTIONS_PER_UNIT,S3L_FRACTIONS_PER_UNIT),
    TPE_vec3(0,0,0)); 
}

void helper_draw3dCube(TPE_Vec3 pos, TPE_Vec3 scale, TPE_Vec3 rot)
{
  cubeModel.config.backfaceCulling = 2;
  helper_drawModel(&cubeModel,pos,scale,rot);
}

void helper_draw3dCubeInside(TPE_Vec3 pos, TPE_Vec3 scale, TPE_Vec3 rot)
{
  cubeModel.config.backfaceCulling = 1;
  helper_drawModel(&cubeModel,pos,scale,rot);
}

void helper_draw3dPlane(TPE_Vec3 pos, TPE_Vec3 scale, TPE_Vec3 rot)
{
  helper_drawModel(&planeModel,pos,scale,rot);
}

void helper_draw3dSphere(TPE_Vec3 pos, TPE_Vec3 scale, TPE_Vec3 rot)
{
  helper_drawModel(&sphereModel,pos,scale,rot);
}

void helper_init(void)
{
  sdl_window = SDL_CreateWindow("program",SDL_WINDOWPOS_UNDEFINED,SDL_WINDOWPOS_UNDEFINED,S3L_RESOLUTION_X,S3L_RESOLUTION_Y,SDL_WINDOW_SHOWN); 
  sdl_renderer = SDL_CreateRenderer(sdl_window,-1,0);
  sdl_texture = SDL_CreateTexture(sdl_renderer,SDL_PIXELFORMAT_RGBX8888,SDL_TEXTUREACCESS_STATIC,S3L_RESOLUTION_X,S3L_RESOLUTION_Y);
  sdl_screenSurface = SDL_GetWindowSurface(sdl_window);

  sdl_keyboard = SDL_GetKeyboardState(NULL);

  helper_running = 1;
  helper_frame = 0;

  helper_frameMsLeft = 0;

  S3L_model3DInit(cubeVertices,S3L_CUBE_VERTEX_COUNT,cubeTriangles,
    S3L_CUBE_TRIANGLE_COUNT,&cubeModel);

  S3L_model3DInit(planeVerices,4,planeTriangles,2,&planeModel);

  S3L_model3DInit(sphereVertices,SPHERE_VERTEX_COUNT,sphereTriangleIndices,
    SPHERE_TRIANGLE_COUNT,&sphereModel);

  S3L_model3DInit(triangleVertices,3,triangleTriangles,2,&triangleModel);

  S3L_sceneInit(0,1,&s3l_scene);

  TPE_worldInit(&tpe_world,tpe_bodies,0,0);
}

void helper_frameStart(void)
{
  helper_frameStartTime = SDL_GetTicks();
    
  for (uint32_t i = 0; i < PIXELS_SIZE; ++i)
    sdl_pixels[i] = 0;

  S3L_newFrame();
  
  SDL_Event event;
  
  while (SDL_PollEvent(&event))
    if (
      (event.type == SDL_QUIT) ||
      ((event.type == SDL_KEYDOWN) && (event.key.keysym.scancode == SDL_SCANCODE_ESCAPE))
      )
      helper_running = 0;
  
  S3L_rotationToDirections(s3l_scene.camera.transform.rotation,
    CAMERA_STEP,&helper_cameraForw,&helper_cameraRight,&helper_cameraUp);

  sdl_keyboard = SDL_GetKeyboardState(NULL);

  if (helper_debugDrawOnCountdown == 0 && sdl_keyboard[SDL_SCANCODE_F1])
  {
    helper_debugDrawOn = !helper_debugDrawOn;
    helper_debugDrawOnCountdown = FPS / 4;
  }
  else if (helper_debugDrawOnCountdown > 0)
    helper_debugDrawOnCountdown--;
}

void helper_frameEnd(void)
{
  SDL_UpdateTexture(sdl_texture,NULL,sdl_pixels,S3L_RESOLUTION_X * sizeof(uint32_t));
  SDL_RenderClear(sdl_renderer);
  SDL_RenderCopy(sdl_renderer,sdl_texture,NULL,NULL);
  SDL_RenderPresent(sdl_renderer);

  helper_frame++;

  helper_frameMsLeft = helper_frameStartTime + MSPF - SDL_GetTicks();

  if (helper_frameMsLeft > 0)
    usleep(helper_frameMsLeft * 1000); // ofc this isn't accurate
}

void helper_end(void)
{
  // TODO
}
