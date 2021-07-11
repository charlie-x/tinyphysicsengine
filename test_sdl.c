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

TPE_Vec4 a, b, r, r2, r3, axis;
TPE_Unit yaw, pitch, roll;

TPE_setVec4(&axis,0,0,512,0);

TPE_rotationToQuaternion(axis,TPE_FRACTIONS_PER_UNIT / 2,&r);

TPE_PRINTF_VEC4(r)

TPE_quaternionToEulerAngles(r,&yaw,&pitch,&roll);

printf("%d %d %d\n",yaw,pitch,roll);

/*
TPE_setVec4(&axis,512,0,0,0);
TPE_rotationToQuaternion(axis,-128,&r);

TPE_setVec4(&axis,0,512,0,0);
TPE_rotationToQuaternion(axis,-128,&r2);

TPE_quaternionMultiply(r,r2,&r3);

TPE_setVec4(&axis,512,0,0,0);
TPE_rotationToQuaternion(axis,-128,&r);
TPE_quaternionMultiply(r3,r,&r2);

TPE_PRINTF_VEC4(r2);

TPE_setVec4(&axis,0,0,512,0);
TPE_rotationToQuaternion(axis,128,&r);

TPE_PRINTF_VEC4(r);
*/

  SDL_Window *window = SDL_CreateWindow("test", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, S3L_RESOLUTION_X, S3L_RESOLUTION_Y, SDL_WINDOW_SHOWN); 
  SDL_Renderer *renderer = SDL_CreateRenderer(window,-1,0);
  SDL_Texture *textureSDL = SDL_CreateTexture(renderer,SDL_PIXELFORMAT_RGBX8888, SDL_TEXTUREACCESS_STATIC, S3L_RESOLUTION_X, S3L_RESOLUTION_Y);
  SDL_Surface *screenSurface = SDL_GetWindowSurface(window);
  SDL_Event event;

  int running = 1;

  S3L_Model3D cubeModel;

  S3L_initModel3D(cubeVertices,S3L_CUBE_VERTEX_COUNT,cubeTriangles,S3L_CUBE_TRIANGLE_COUNT,&cubeModel);

  cubeModel.transform.translation.z = 3 * S3L_FRACTIONS_PER_UNIT;
 
  S3L_Scene scene;

  S3L_initScene(&cubeModel,1,&scene);
 
  while (running)
  {
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
  }

  return 0;
}

