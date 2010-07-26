// Ray.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <assert.h>
#include <concrt.h>
#include "camera.hpp"
#include "ray_math.hpp"
#include "ray_trace.hpp"

extern "C"
{
#include <lua.h>
#include <lauxlib.h>
#include <lualib.h>
};

struct Object
{
	virtual ~Object() {}
	virtual bool intersect(const Vec3& o, const Vec3& d, float *t) const = 0;
	virtual Vec3 calc_normal(const Vec3& p) const = 0;
};

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

using namespace Concurrency;

// renders from start_y, h scanlines down
struct RayTracer::RenderJobData
{
  RenderJobData(int start_y, int num_lines, int width, int height, const Camera *camera, const Objects *objects) 
    : start_y(start_y), num_lines(num_lines), width(width), height(height), camera(camera), objects(objects), ptr(nullptr) {}
  int start_y;
  int num_lines;
  int width, height;
  const Camera *camera;
  const Objects *objects;
  void *ptr;
  event signal;
};

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

bool RayTracer::init(int width, int height)
{
	lua_State *l;
	if (!load_scene("scene1.lua", &l))
		return false;

	objects.push_back(new Sphere(Vec3(-10, 0, -200), 10));
	objects.push_back(new Sphere(Vec3(+10, 5, -200), 10));
	objects.push_back(new Sphere(Vec3(0, 0, -240), 10));
	objects.push_back(new Plane(Vec3(0, -10, 0), Vec3(0,1,0)));

  int ofs = 0;
  int num_jobs = height / 4;
  int lines = height / num_jobs;
  while (ofs <= height) {
    datas.push_back(new RenderJobData(ofs, min(height-ofs, lines), width, height, &_camera, &objects));
    events.push_back(&datas.back()->signal);
    ofs += lines;
  }

	return true;
}

void RayTracer::close()
{
	for (int i = 0; i < (int)objects.size(); ++i)
		delete objects[i];
	objects.clear();

  for (int i = 0; i < (int)datas.size(); ++i)
    delete datas[i];
  datas.clear();
}

static void __cdecl RenderJob(LPVOID param)
{
  RayTracer::RenderJobData *data = (RayTracer::RenderJobData *)param;
  data->signal.reset();

  Vec3 o, d;
  Vec3 light_pos(0,100,-150);

  const float kEps = 0.001f;

  BGRA32 *p = (BGRA32 *)data->ptr + data->start_y * data->width;
  for (int y = data->start_y; y < data->start_y + data->num_lines; ++y) {
    for (int x = 0; x < data->width; ++x) {

      float t;
      data->camera->ray_from_pixel(x, y, data->width, data->height, &o, &d);

      if (Object *obj = find_intersection(*data->objects, o, d, &t)) {
        Vec3 p0 = o + t * d;

        Vec3 probe = (light_pos - p0);
        const float light_dist = probe.len();
        normalize(probe);
        float tmp;
        Object *occulder = find_intersection(*data->objects, p0 + 0.1f * probe, probe, &tmp);
        if (occulder && tmp > 0 && tmp < light_dist) {
          p->r = p->g = p->b = p->a = 0;
        } else {
          Vec3 l = normalize(light_pos - p0);
          Vec3 n = obj->calc_normal(p0);
          Vec3 v = normalize(data->camera->_pos - p0);
          Vec3 h = normalize(l + v);
          float spec = powf(dot(n, h), 32);
          float diffuse = dot(n, l);
          float ambient = 0.2f;
          float col = min(1, max(0, diffuse + spec));
          p->r = p->g = p->b = p->a = (uint8_t)(255 * col);
        }
      } else {
        p->r = p->g = p->b = p->a = 0;
      }
      p++;
    }
  }

  data->signal.set();
}


void RayTracer::render(void *ptr, int width, int height)
{

  for (int i = 0; i < (int)datas.size(); ++i) {
    datas[i]->ptr = ptr;
    CurrentScheduler::ScheduleTask(RenderJob, datas[i]);
  }

  event::wait_for_multiple(&events[0], events.size(), true);

/*
	Vec3 o, d;
	Vec3 light_pos(0,100,-150);

	BGRA32 *p = (BGRA32 *)ptr;
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {

			float t;
			_camera.ray_from_pixel(x, y, width, height, &o, &d);

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
					Vec3 v = normalize(_camera._pos - p0);
					Vec3 h = normalize(l + v);
					float spec = powf(dot(n, h), 32);
					float diffuse = dot(n, l);
					float ambient = 0.2f;
					float col = min(1, max(0, diffuse + spec));
					p->r = p->g = p->b = p->a = (uint8_t)(255 * col);
				}
			} else {
				p->r = p->g = p->b = p->a = 0;
			}
			p++;
		}
	}
  */
}
