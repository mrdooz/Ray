#pragma once
#include "ray.hpp"

struct Object;
typedef std::vector<Object *> Objects;

struct Camera;

struct RayTracer : public RayBase
{

	virtual bool init(int width, int height);
	virtual void render(const Camera& c, void *ptr, int width, int height);
	virtual void close();

	Objects objects;
};
