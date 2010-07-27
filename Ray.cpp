// Ray.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <assert.h>
#include <concrt.h>
#include "ray_march.hpp"
#include "ray_trace.hpp"
#include "ray.hpp"
#include "camera.hpp"
#include "ray_math.hpp"
#include <celsus/file_watcher.hpp>

extern "C"
{
#include <lua.h>
#include <lauxlib.h>
#include <lualib.h>
};

#pragma comment(lib, "sdl.lib")
#pragma comment(lib, "winmm.lib")

SDL_Surface *g_font = NULL;
SDL_Surface *g_screen  = NULL;

struct CSDL_Rect : public SDL_Rect
{
	CSDL_Rect(Sint16 x, Sint16 y, Uint16 w, Uint16 h)
	{
		this->x = x; this->y = y;
		this->w = w; this->h = h;
	}
};

const int kCharWidth = 14;
const int kCharHeight = 24;

void draw_char(char ch, int x, int y)
{
	CSDL_Rect src(0, (ch - 32) * kCharHeight, kCharWidth, kCharHeight), dst(x, y, kCharWidth, kCharHeight);
	SDL_BlitSurface(g_font, &src, g_screen, &dst);
}

void draw_string(int x, int y, const char *fmt, ...)
{
	va_list arg;
	va_start(arg, fmt);

	const int len = _vscprintf(fmt, arg) + 1;
	char* buf = (char*)_alloca(len);
	vsprintf_s(buf, len, fmt, arg);

	while (*buf) {
		draw_char(*buf,x,y);
		x += kCharWidth;
		buf++;
	}
	va_end(arg);
}


int _tmain(int argc, _TCHAR* argv[])
{
	SDL_Init(SDL_INIT_EVERYTHING);

	const int width = GetSystemMetrics(SM_CXSCREEN) / 2;
	const int height = GetSystemMetrics(SM_CYSCREEN) / 2;

	g_screen = SDL_SetVideoMode(width, height, 32, SDL_DOUBLEBUF);

	SDL_Surface *temp = SDL_LoadBMP("font14x24.bmp");
	g_font = SDL_ConvertSurface(temp, g_screen->format, SDL_SWSURFACE);
	SDL_FreeSurface(temp);
	SDL_SetColorKey(g_font, SDL_SRCCOLORKEY, 0);
  SDL_EnableKeyRepeat(500, 50);

  RayBase *ray = new RayTracer();
  ray->_camera._pos = Vec3(0,4, 15);
  ray->_camera._up = Vec3(0,1,0);
  ray->_camera._dir = Vec3(0,0,-1);

	if (!ray->init(width, height))
		return 1;


	bool done = false;
  FileWatcher::instance().init();
	while (!done) {

    FileWatcher::instance().tick();

		SDL_Event event;
		bool redraw = false;
		if (SDL_PollEvent(&event)) {
			switch(event.type) {

			case SDL_QUIT:
				done = true;
				break;

			case SDL_KEYDOWN:
				switch (event.key.keysym.sym) {
				case SDLK_a: ray->_camera._pos.z -= 1; redraw = true; break;
				case SDLK_z: ray->_camera._pos.z += 1; redraw = true; break;
				case SDLK_UP: ray->_camera._pos.y += 1; redraw = true; break;
				case SDLK_DOWN: ray->_camera._pos.y -= 1; redraw = true; break;
				case SDLK_LEFT: ray->_camera._pos.x -= 1; redraw = true; break;
				case SDLK_RIGHT: ray->_camera._pos.x += 1; redraw = true; break;
				case SDLK_ESCAPE: done = true; break;
				}
				break;
			}
		}

		if (redraw || ray->force_update() ) {
			ray->_camera._dir = normalize(Vec3(0,0,-200) - ray->_camera._pos);

			// scale the view plane by the aspect ratio of the bitmap to get square pixels
			const float aspect = (float)width / height;
			const float size = 10;
			ray->_camera._u0 = -aspect * size;
			ray->_camera._u1 = +aspect * size;
			ray->_camera._v0 = -size;
			ray->_camera._v1 = +size;
			ray->_camera._dist = 100;

			ray->_camera.create_frame();

			SDL_LockSurface(g_screen);
			DWORD start = timeGetTime();
			ray->render(g_screen->pixels, g_screen->w, g_screen->h);
			DWORD elapsed = timeGetTime() - start;
			SDL_UnlockSurface(g_screen);
			draw_string(0, 0, "time: %.3fs", elapsed / 1000.0f);
			draw_string(0, kCharHeight, "cam pos: %.3f, %.3f, %.3f dir: %.3f, %.3f, %.3f", ray->_camera._pos.x, ray->_camera._pos.y, ray->_camera._pos.z, ray->_camera._dir.x, ray->_camera._dir.y, ray->_camera._dir.z);
			draw_string(0, 2 * kCharHeight, "a: %.3f, %.3f, %.3f", ray->_camera._a.x, ray->_camera._a.y, ray->_camera._a.z);
			draw_string(0, 3 * kCharHeight, "b: %.3f, %.3f, %.3f", ray->_camera._b.x, ray->_camera._b.y, ray->_camera._b.z);
			draw_string(0, 4 * kCharHeight, "c: %.3f, %.3f, %.3f", ray->_camera._c.x, ray->_camera._c.y, ray->_camera._c.z);
			SDL_Flip(g_screen);
		}
	}
  FileWatcher::instance().close();

	ray->close();

	SDL_FreeSurface(g_font);

	SDL_Quit();

	return 0;
}
