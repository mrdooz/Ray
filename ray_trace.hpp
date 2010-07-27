#pragma once
#include "ray.hpp"
#include "camera.hpp"

struct Object;
typedef std::vector<Object *> Objects;


namespace Concurrency
{
  class event;
}

struct Camera;
struct lua_State;

struct RayTracer : public RayBase
{
	virtual bool init(int width, int height);
	virtual void render(void *ptr, int width, int height);
	virtual void close();
  bool load_scene(const string2& filename);

  struct RenderJobData;

  std::vector<RenderJobData *> _datas;
  std::vector<Concurrency::event *> _events;

	Objects _objects;
};
