// Ray.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <vector>
#pragma comment(lib, "sdl.lib")

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
	_c = _u0 * _frame.u + _v0 * _frame.v + _dist * -_frame.w;
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

struct Sphere : public Object
{
	Sphere() {};
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

	const float t0 = -B + sqrtf(discriminant) / 2 * A;
	const float t1 = -B - sqrtf(discriminant) / 2 * A;

	*t = t1 >= 0 ? t1 : t0;
	return true;
}

Vec3 Sphere::calc_normal(const Vec3& p) const
{
	return normalize(p - c);
}

void create_ortho_ray(int x, int y, const Camera& cam, Vec3 *pos, Vec3 *dir)
{
	// orographic

}

void create_ray(int x, int y, Vec3 *pos, Vec3 *dir)
{
}


struct BGRA32
{
	uint8_t b;
	uint8_t g;
	uint8_t r;
	uint8_t a;
};

void render(const Camera& c, void *ptr, int width, int height)
{
	typedef std::vector<Object *> Objects;
	Objects objects;
	objects.push_back(new Sphere(Vec3(-10, 0, -200), 10));
	objects.push_back(new Sphere(Vec3(+10, 0, -200), 10));
	objects.push_back(new Sphere(Vec3(0, 0, -240), 10));

	Vec3 o, d;

	Vec3 light_pos(0,100,-200);

	float t;
	BGRA32 *p = (BGRA32 *)ptr;
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {

			c.ray_from_pixel(x, y, width, height, &o, &d);

			int idx = -1;
			float min_t = FLT_MAX;
			for (int i = 0; i < (int)objects.size(); ++i) {
				if (objects[i]->intersect(o, d, &t) && t < min_t) {
					min_t = t;
					idx = i;
				}
			}

			if (idx != -1) {
				Vec3 p0 = o + min_t * d;
				Vec3 l = normalize(light_pos - p0);
				Vec3 n = objects[idx]->calc_normal(p0);
				p->r = p->g = p->b = p->a = (uint8_t)(255 * dot(n, l));
			} else {
				p->r = p->g = p->b = p->a = 0;
			}

			p++;
		}
	}

	for (int i = 0; i < (int)objects.size(); ++i)
		delete objects[i];
	objects.clear();
}

int _tmain(int argc, _TCHAR* argv[])
{
	SDL_Init(SDL_INIT_EVERYTHING);

	const int width = GetSystemMetrics(SM_CXSCREEN) / 2;
	const int height = GetSystemMetrics(SM_CYSCREEN) / 2;

	SDL_Surface *screen = SDL_SetVideoMode(width, height, 32, SDL_DOUBLEBUF);

	Camera c;
	c._pos = Vec3(0,0,0);
	c._up = Vec3(0,1,0);

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

			case SDL_KEYUP:
				switch (event.key.keysym.sym) {
				case SDLK_UP: c._pos.y -= 1; redraw = true; break;
				case SDLK_DOWN: c._pos.y += 1; redraw = true; break;
				case SDLK_LEFT: c._pos.x -= 1; redraw = true; break;
				case SDLK_RIGHT: c._pos.x += 1; redraw = true; break;
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

      SDL_LockSurface(screen);
      render(c, screen->pixels, screen->w, screen->h);
      SDL_UnlockSurface(screen);

      SDL_Flip(screen);

    }

	}

	SDL_Quit();

	return 0;
}

