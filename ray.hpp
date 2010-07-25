#pragma once

struct Camera;

struct RayBase
{
	virtual ~RayBase() {}

	virtual bool init(int width, int height) = 0;
	virtual void render(const Camera& c, void *ptr, int width, int height) = 0;
	virtual void close() = 0;
};