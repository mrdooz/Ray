// Ray.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <assert.h>

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

// we use a right-handed coordinate system (yes, this is going to confuse the hell out of me :)

//  +y
//  |  -z
//  | /
//  |/
//  +------ +x
//

struct Vec3
{
	Vec3() {}
	Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
	union {
		struct {
			float x, y, z;
		};
		float d[3];
	};

	float len() const;

	const float static kEps;

};

const float Vec3::kEps = 0.00001f;

float Vec3::len() const
{
	return sqrtf(x*x + y*y + z*z);
}

Vec3 operator+(const Vec3& a, const Vec3& b)
{
	return Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

Vec3 operator-(const Vec3& a, const Vec3& b)
{
	return Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

Vec3 operator-(const Vec3& a)
{
	return Vec3(-a.x, -a.y, -a.z);
}

Vec3 operator/(const Vec3& a, float s)
{
	return Vec3(a.x / s, a.y / s, a.z / s);
}

Vec3 operator*(float s, const Vec3& a)
{
	return Vec3(s * a.x, s * a.y, s * a.z);
}

Vec3 normalize(const Vec3& a)
{
	Vec3 res;
	const float len = a.len();
	if (len < Vec3::kEps)
		res.x = res.y = res.z = 0;
	else
		res = a / len;
	return res;
}

float dot(const Vec3& a, const Vec3& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(const Vec3& a, const Vec3& b)
{
	return Vec3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}


struct Frame
{
	Vec3 u, v, w;
};

// model according to "realistic ray-tracing", page 66
struct Camera
{

	void create_frame();

	void pixel_to_screen(int x, int y, int width, int height, float *a, float *b) const;
	void screen_to_world(float a, float b, Vec3 *world) const;
	void ray_to_world(const Vec3& s, Vec3 *o, Vec3 *d) const;

	void ray_from_pixel(int x, int y, int nx, int ny, Vec3 *o, Vec3 *d) const;

	Vec3 _pos;
	Vec3 _dir;
	Vec3 _up;
	float _dist;
	float _u0, _v0;
	float _u1, _v1;

	// transient values
	Frame _frame;
	Vec3 _a, _b, _c;
};

void Camera::pixel_to_screen(int x, int y, int nx, int ny, float *a, float *b) const
{
	// convert pixel coordinates (0..width) to screen coordinates (0..1)
	// we assume that pixel coordinates point to the middle of the pixel, hence the 0.5 offset.
	// also, y = 0 is the top most pixel-coordinate, so we must flip y
	*a = (x + 0.5f) / nx;
	*b = (ny - y - 0.5f) / ny;
}

void Camera::screen_to_world(float a, float b, Vec3 *s) const
{
	// convert from [0..1]^2 screen coords to world coords
	*s = _c + a * _a + b * _b;
}


void Camera::create_frame()
{
	_frame.w = -_dir;
	_frame.u = normalize(cross(_up, _frame.w));
	_frame.v = cross(_frame.w, _frame.u);

	_a = (_u1 - _u0) * _frame.u;
	_b = (_v1 - _v0) * _frame.v;
	_c = _pos + _u0 * _frame.u + _v0 * _frame.v + _dist * -_frame.w;
}

void Camera::ray_to_world(const Vec3& s, Vec3 *o, Vec3 *d) const
{
	*o = _pos;
	*d = normalize(s - _pos);
}

void Camera::ray_from_pixel(int x, int y, int nx, int ny, Vec3 *o, Vec3 *d) const
{
	float a, b;
	pixel_to_screen(x, y, nx, ny, &a, &b);
	Vec3 s;
	screen_to_world(a, b, &s);
	ray_to_world(s, o, d);
}

struct Object
{
	virtual ~Object() {}
	virtual bool intersect(const Vec3& o, const Vec3& d, float *t) const = 0;
	virtual Vec3 calc_normal(const Vec3& p) const = 0;
};

typedef std::vector<Object *> Objects;

struct Sphere : public Object
{
	Sphere(const Vec3& center, float radius) : c(center), r(radius) {}

	virtual bool intersect(const Vec3& o, const Vec3& d, float *t) const;
	virtual Vec3 calc_normal(const Vec3& p) const;

	Vec3 c;
	float r;
};

bool Sphere::intersect(const Vec3& o, const Vec3& d, float *t) const
{
	// d.d * t^2 + 2*d.d(o-c) * t + (o-c).(o-c) - r^2
	//   A * t^2 +          B * t + C;
	// t = -B +- sqrt(B^2 - 4AC) / 2A

	const float A = dot(d, d);
	const float B = 2 * dot(d, o-c);
	const float C = dot(o-c, o-c) - r * r;
	const float discriminant = B * B - 4 * A * C;

	if (discriminant < 0)
		return false;

	const float t0 = (-B + sqrtf(discriminant)) / (2 * A);
	const float t1 = (-B - sqrtf(discriminant)) / (2 * A);

	*t = t1 >= 0 ? t1 : t0;
	return true;
}

Vec3 Sphere::calc_normal(const Vec3& p) const
{
	return normalize(p - c);
}

struct Plane : public Object
{
	Plane(const Vec3& a, const Vec3& n) : a(a), n(n) {}
	virtual bool intersect(const Vec3& o, const Vec3& d, float *t) const;
	virtual Vec3 calc_normal(const Vec3& p) const;

	Vec3 a, n;
};

bool Plane::intersect(const Vec3& o, const Vec3& d, float *t) const
{
	// plane eq: (p-a).n = 0
	// p = o + t * d
	// (o + t * d - a).n =0
	// t = (a-o).n / d.n

	*t = dot(a-o, n) / (dot(d, n));
	return *t >= 0;
}

Vec3 Plane::calc_normal(const Vec3& p) const
{
	return n;
}


struct BGRA32
{
	uint8_t b;
	uint8_t g;
	uint8_t r;
	uint8_t a;
};

Object *find_intersection(const Objects& objects, const Vec3& o, const Vec3& d, float *t)
{
	int idx = -1;
	float tmp, min_t = FLT_MAX;
	for (int i = 0; i < (int)objects.size(); ++i) {
		if (objects[i]->intersect(o, d, &tmp) && tmp < min_t) {
			min_t = tmp;
			idx = i;
		}
	}
  *t = min_t;
	return idx == -1 ? NULL : objects[idx];
}

void render(const Camera& c, const Objects& objects, void *ptr, int width, int height)
{
	Vec3 o, d;
	Vec3 light_pos(0,100,-150);

	BGRA32 *p = (BGRA32 *)ptr;
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {

			float t;
			c.ray_from_pixel(x, y, width, height, &o, &d);

			if (Object *obj = find_intersection(objects, o, d, &t)) {
				Vec3 p0 = o + t * d;

        Vec3 probe = (light_pos - p0);
        const float light_dist = probe.len();
        normalize(probe);
        float tmp;
        Object *occulder = find_intersection(objects, p0 + 0.1f * probe, probe, &tmp);
        if (occulder && tmp > 0 && tmp < light_dist) {
          p->r = p->g = p->b = p->a = 0;
        } else {
          Vec3 l = normalize(light_pos - p0);
          Vec3 n = obj->calc_normal(p0);
          Vec3 v = normalize(c._pos - p0);
          Vec3 h = normalize(l + v);
          float spec = powf(dot(n, h), 32);
          float diffuse = dot(n, l);
          float col = min(1, max(0, diffuse + spec));
          p->r = p->g = p->b = p->a = (uint8_t)(255 * col);
        }
			} else {
				p->r = p->g = p->b = p->a = 0;
			}
			p++;
		}
	}

}

/*
bool load_scene(const char *filename, lua_State **ll)
{
	lua_State *l = *ll = lua_open();
	if (l == NULL)
		return false;
	luaL_openlibs(l);

	if (luaL_loadfile(l,  filename))
		return false;

	return true;
}
*/

struct CSDL_Rect : public SDL_Rect
{
	CSDL_Rect(Sint16 x, Sint16 y, Uint16 w, Uint16 h)
	{
		this->x = x; this->y = y;
		this->w = w; this->h = h;
	}
};

// Draw a single character.
// Characters are on top of each other in the font image, in ASCII order,
// so all this routine does is just set the coordinates for the character
// and use SDL to blit out.

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

	HDC dc = GetWindowDC(NULL);
	ReleaseDC(NULL, dc);
/*
	lua_State *l;
	if (!load_scene("scene1.lua", &l))
		return 1;
*/
	SDL_Init(SDL_INIT_EVERYTHING);

	const int width = GetSystemMetrics(SM_CXSCREEN) / 2;
	const int height = GetSystemMetrics(SM_CYSCREEN) / 2;

	g_screen = SDL_SetVideoMode(width, height, 32, SDL_DOUBLEBUF);

	Objects objects;

	objects.push_back(new Sphere(Vec3(-10, 0, -200), 10));
	objects.push_back(new Sphere(Vec3(+10, 5, -200), 10));
	objects.push_back(new Sphere(Vec3(0, 0, -240), 10));
	objects.push_back(new Plane(Vec3(0, -10, 0), Vec3(0,1,0)));

	SDL_Surface *temp = SDL_LoadBMP("font14x24.bmp");
	g_font = SDL_ConvertSurface(temp, g_screen->format, SDL_SWSURFACE);
	SDL_FreeSurface(temp);
	SDL_SetColorKey(g_font, SDL_SRCCOLORKEY, 0);


	Camera c;
	c._pos = Vec3(0,0, 0);
	c._up = Vec3(0,1,0);
	c._dir = Vec3(0,0,-1);

	bool done = false;
  bool first_time = true;
	while (!done) {

		SDL_Event event;
    bool redraw = false;
		if (SDL_PollEvent(&event)) {
			switch(event.type) {
			
			case SDL_QUIT:
				done = true;
				break;

			case SDL_KEYDOWN:
				switch (event.key.keysym.sym) {
				case SDLK_UP: c._pos.y += 1; redraw = true; break;
				case SDLK_DOWN: c._pos.y -= 1; redraw = true; break;
				case SDLK_LEFT: c._pos.x -= 1; redraw = true; break;
				case SDLK_RIGHT: c._pos.x += 1; redraw = true; break;
				case SDLK_ESCAPE: done = true; break;
				}
				break;
			}
		}

    if (redraw || first_time) {
      first_time = false;
      c._dir = normalize(Vec3(0,0,-200) - c._pos);

      // scale the view plane by the aspect ratio of the bitmap to get square pixels
      const float aspect = (float)width / height;
      const float size = 10;
      c._u0 = -aspect * size;
      c._u1 = +aspect * size;
      c._v0 = -size;
      c._v1 = +size;
      c._dist = 100;

      c.create_frame();

      SDL_LockSurface(g_screen);
			DWORD start = timeGetTime();
      render(c, objects, g_screen->pixels, g_screen->w, g_screen->h);
			DWORD elapsed = timeGetTime() - start;
      SDL_UnlockSurface(g_screen);
			draw_string(0, 0, "time: %.3fs", elapsed / 1000.0f);
			draw_string(0, kCharHeight, "cam pos: %.3f, %.3f, %.3f dir: %.3f, %.3f, %.3f", c._pos.x, c._pos.y, c._pos.z, c._dir.x, c._dir.y, c._dir.z);
			draw_string(0, 2 * kCharHeight, "a: %.3f, %.3f, %.3f", c._a.x, c._a.y, c._a.z);
			draw_string(0, 3 * kCharHeight, "b: %.3f, %.3f, %.3f", c._b.x, c._b.y, c._b.z);
			draw_string(0, 4 * kCharHeight, "c: %.3f, %.3f, %.3f", c._c.x, c._c.y, c._c.z);
      SDL_Flip(g_screen);
    }

	}

	for (int i = 0; i < (int)objects.size(); ++i)
		delete objects[i];
	objects.clear();

	SDL_FreeSurface(g_font);

	SDL_Quit();

	return 0;
}
