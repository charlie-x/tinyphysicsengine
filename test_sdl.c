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
TPE_Vec4 a, b, axis, r;
a.x = TPE_FRACTIONS_PER_UNIT / 2;
a.y = TPE_FRACTIONS_PER_UNIT;
a.z = TPE_FRACTIONS_PER_UNIT;
a.w = TPE_FRACTIONS_PER_UNIT;

b.x = 0;
b.y = TPE_FRACTIONS_PER_UNIT;
b.z = TPE_FRACTIONS_PER_UNIT / 2;
b.w = 0;

axis.x = TPE_FRACTIONS_PER_UNIT;
axis.y = TPE_FRACTIONS_PER_UNIT;
axis.z = 0;
axis.w = 0;

TPE_Unit angle = TPE_FRACTIONS_PER_UNIT / 2;

TPE_rotationToQuaternion(axis,angle,&r);
TPE_PRINTF_VEC4(r);

axis.x = 0;
axis.y = 0;
axis.z = 0;
axis.w = 0;
angle = 0;

TPE_quaternionToRotation(r,&axis,&angle);

TPE_PRINTF_VEC4(axis);
printf("%d\n",angle);

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

