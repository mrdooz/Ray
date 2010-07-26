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

struct RayTracer : public RayBase
{

	virtual bool init(int width, int height);
	virtual void render(void *ptr, int width, int height);
	virtual void close();

  struct RenderJobData;

  std::vector<RenderJobData *> datas;
  std::vector<Concurrency::event *> events;

	Objects objects;
};
