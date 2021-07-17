#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdint.h>

#define S3L_RESOLUTION_X 640
#define S3L_RESOLUTION_Y 480
#define S3L_PIXEL_FUNCTION drawPixel

#include "small3dlib.h"

#include "tinyphysicsengine.h"

#define PIXELS_SIZE (S3L_RESOLUTION_X * S3L_RESOLUTION_Y * 4)

uint8_t pixels[PIXELS_SIZE];

void drawPixel(S3L_PixelInfo *p)
{
  uint32_t index = (p->y * S3L_RESOLUTION_X + p->x) * 4;
  pixels[index + 1] = p->triangleIndex * 16;
  pixels[index + 2] = 255 - p->triangleIndex * 16;
}

S3L_Unit cubeVertices[] = { S3L_CUBE_VERTICES(S3L_FRACTIONS_PER_UNIT) };
S3L_Index cubeTriangles[] = { S3L_CUBE_TRIANGLES };

int main()
{
  SDL_Window *window = SDL_CreateWindow("test", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, S3L_RESOLUTION_X, S3L_RESOLUTION_Y, SDL_WINDOW_SHOWN); 
  SDL_Renderer *renderer = SDL_CreateRenderer(window,-1,0);
  SDL_Texture *textureSDL = SDL_CreateTexture(renderer,SDL_PIXELFORMAT_RGBX8888, SDL_TEXTUREACCESS_STATIC, S3L_RESOLUTION_X, S3L_RESOLUTION_Y);
  SDL_Surface *screenSurface = SDL_GetWindowSurface(window);
  SDL_Event event;

  int running = 1;

  S3L_Model3D cubeModel;

  S3L_initModel3D(cubeVertices,S3L_CUBE_VERTEX_COUNT,cubeTriangles,S3L_CUBE_TRIANGLE_COUNT,&cubeModel);

 
  S3L_Scene scene;

  S3L_initScene(&cubeModel,1,&scene);

  scene.camera.transform.translation.z = -3 * S3L_FRACTIONS_PER_UNIT;


TPE_Body body;

TPE_bodyInit(&body);

S3L_Mat4 m;
cubeModel.customTransformMatrix = &m;



TPE_Vec4 p, v;

TPE_vec4Set(&p,512,512,0,0);
TPE_vec4Set(&v,30,0,0,0);



//TPE_vec4Set(&p,0,512,0,0);
//TPE_bodySetRotation(&body,p,10);

//TPE_vec4Set(&p,512,0,0,0);
//TPE_bodyAddRotation(&body,p,20);

TPE_bodyApplyVelocity(&body,p,v);

//TPE_bodySetRotation(&body,axis,5);
 
  TPE_Unit frame = 0;

  while (running)
  {

/*
TPE_PRINTF_VEC4(body.rotation.originalOrientation);
TPE_PRINTF_VEC4(body.rotation.axisVelocity);
printf("%d\n",body.rotation.currentAngle);
*/

TPE_bodyGetTransformMatrix(&body,m);
TPE_bodyStep(&body);





//S3L_logMat4(m);

/*
S3L_makeRotationMatrixZXY(128,0,0,&m);

S3L_logMat4(m);

break;
*/
    for (uint32_t i = 0; i < PIXELS_SIZE; ++i)
      pixels[i] = 0;

    S3L_drawScene(scene);

    SDL_UpdateTexture(textureSDL,NULL,pixels,S3L_RESOLUTION_X * sizeof(uint32_t));

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

    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer,textureSDL,NULL,NULL);
    SDL_RenderPresent(renderer);

    usleep(20000);

    frame++;
  }

  return 0;
}

