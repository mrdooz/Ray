#pragma once
#include "camera.hpp"

struct RayBase
{
	virtual ~RayBase() {}

	virtual bool init(int width, int height) = 0;
	virtual void render(void *ptr, int width, int height) = 0;
	virtual void close() = 0;

  Camera _camera;
};