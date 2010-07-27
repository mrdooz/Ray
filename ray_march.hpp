#pragma once
#include "ray.hpp"


namespace Concurrency
{
	class event;
}

struct RayMarcher : public RayBase
{
	virtual bool init(int width, int height);
	virtual void render(const Camera& c, void *ptr, int width, int height);
	virtual void close();

  struct RenderJobData;

	std::vector<RenderJobData *> _datas;
	std::vector<Concurrency::event *> _events;
};
