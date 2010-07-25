#pragma once
#include "ray.hpp"

struct RenderJobData;

namespace Concurrency
{
	class event;
}

struct RayMarcher : public RayBase
{
	virtual bool init(int width, int height);
	virtual void render(const Camera& c, void *ptr, int width, int height);
	virtual void close();

	std::vector<RenderJobData *> datas;
	std::vector<Concurrency::event *> events;

};
