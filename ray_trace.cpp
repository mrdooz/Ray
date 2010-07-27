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
#include <celsus/string_utils.hpp>
#include <celsus/path_utils.hpp>
#include <celsus/Logger.hpp>
#include <celsus/file_watcher.hpp>
#include <celsus/fast_delegate.hpp>

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

struct LuaTable
{
  LuaTable(lua_State *state, const char *name);
  ~LuaTable();
  int get_int_field(const char *key);
  float get_float_field(const char *key);
  bool get_float_array(const char *key, float *arr, int count);

  bool iterate(std::function<bool ()> fn);

  string2 _name;
  lua_State * _state;
};

LuaTable::LuaTable(lua_State *state, const char *name)
  : _state(state)
  , _name(name)
{
  lua_getglobal(_state, _name);
}

LuaTable::~LuaTable()
{
  lua_pop(_state, 1);
}

bool LuaTable::iterate(std::function<bool ()> fn)
{
  // iterate over all the elements in a table, and for each element, call fn

  int i = 1;
  for (;;) {
    lua_pushinteger(_state, i + 1); // push index (key)
    lua_gettable(_state, -2);
    if (!lua_isnumber(_state, -1) || !lua_istable(_state, -1)) {
      break;
    }
    if (!fn())
      return false;
  }
  return true;
}

int LuaTable::get_int_field(const char *key)
{
  SCOPED_OBJ([this](){lua_pop(_state, 1); });	// pop the value off the stack
  // assumes the table with the field is pushed on the stack
  lua_pushstring(_state, key);
  lua_gettable(_state, -2);	// pop key, and push value on stack. the table is at -2
  if (!lua_isnumber(_state, -1)) {
    LOG_WARNING_LN("Error reading value for key: %s", key);
    throw std::runtime_error(__FUNCTION__);
  }
  return (int)((int64_t)lua_tonumber(_state, -1));
}

bool LuaTable::get_float_array(const char *key, float *arr, int count)
{
  SCOPED_OBJ([this](){lua_pop(_state, 1); });	// pop the value off the stack
  // assumes the table with the field is pushed on the stack
  lua_pushstring(_state, key);
  lua_gettable(_state, -2);	// pop key, and push value on stack
  if (!lua_istable(_state, -1)) {
    LOG_WARNING_LN("Error reading value for key: %s", key);
    return false;
  }

  for (int i = 0; i < count; ++i) {
    lua_pushinteger(_state, i + 1); // push index (key)
    lua_gettable(_state, -2);
    if (!lua_isnumber(_state, -1)) {
      return false;
    }
    arr[i] = (float)lua_tonumber(_state, -1);
    lua_pop(_state, 1); // pop value
  }
  return true;
}

float LuaTable::get_float_field(const char *key)
{
  SCOPED_OBJ([this](){lua_pop(_state, 1); });	// pop the value off the stack
  // assumes the table with the field is pushed on the stack
  lua_pushstring(_state, key);
  lua_gettable(_state, -2);	// pop key, and push value on stack
  if (!lua_isnumber(_state, -1)) {
    LOG_WARNING_LN("Error reading value for key: %s", key);
    throw std::runtime_error(__FUNCTION__);
  }
  return (float)lua_tonumber(_state, -1);
}


bool lua_init(lua_State **ll, const char *filename, bool run)
{
  lua_State *l = *ll = lua_open();
  if (l == NULL)
    return false;
  luaL_openlibs(l);

  if (luaL_loadfile(l,  filename)) {
    LOG_WARNING_LN(lua_tostring(l, -1));
    return false;
  }

  // update the package path to include the script's directory
  luaL_dostring(l, string2::fmt("package.path = '%s/?.lua;' .. package.path", Path::get_path(filename)));

  if (run) {
    if (lua_pcall(l, 0, 0, 0)) {
      LOG_WARNING_LN(lua_tostring(l, -1));
      return false;
    }
  }

  return true;
}

bool funky()
{
  return true;
}

bool RayTracer::load_scene(const string2& filename)
{
  lua_State *l = NULL;
  if (!lua_init(&l, filename, true))
    return false;

  // load camera settings
  LuaTable camera_table(l, "camera");

  _camera._dist = camera_table.get_float_field("dist");
  if (!camera_table.get_float_array("pos", &_camera._pos[0], 3)) 
    return false;

  float look_at[3];
  if (!camera_table.get_float_array("look-at", look_at, 3))
    return false;

  if (!camera_table.get_float_array("up", &_camera._up[0], 3))
    return false;

  LuaTable scene_table(l, "scene");
  scene_table.iterate(funky);


  _force_update = true;
  
	return true;
}

bool RayTracer::init(int width, int height)
{
  if (!FileWatcher::instance().add_file_changed("scene1.lua", fastdelegate::MakeDelegate(this, &RayTracer::load_scene), true))
    return false;

	_objects.push_back(new Sphere(Vec3(-10, 0, -200), 10));
	_objects.push_back(new Sphere(Vec3(+10, 5, -200), 10));
	_objects.push_back(new Sphere(Vec3(0, 0, -240), 10));
	_objects.push_back(new Plane(Vec3(0, -10, 0), Vec3(0,1,0)));

  int ofs = 0;
  int num_jobs = height / 4;
  int lines = height / num_jobs;
  while (ofs <= height) {
    _datas.push_back(new RenderJobData(ofs, min(height-ofs, lines), width, height, &_camera, &_objects));
    _events.push_back(&_datas.back()->signal);
    ofs += lines;
  }

	return true;
}

void RayTracer::close()
{
  container_delete(_objects);
  container_delete(_datas);
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
  for (int i = 0; i < (int)_datas.size(); ++i) {
    _datas[i]->ptr = ptr;
    CurrentScheduler::ScheduleTask(RenderJob, _datas[i]);
  }

  event::wait_for_multiple(&_events[0], _events.size(), true);
}
