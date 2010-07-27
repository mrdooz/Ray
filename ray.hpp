#pragma once
#include "camera.hpp"
#include <celsus/celsus.hpp>

struct BGRA32
{
  uint8_t b;
  uint8_t g;
  uint8_t r;
  uint8_t a;
};

struct RayBase
{
  RayBase() : _force_update(true) {}
	virtual ~RayBase() {}

	virtual bool init(int width, int height) = 0;
	virtual void render(void *ptr, int width, int height) = 0;
	virtual void close() = 0;

  bool force_update() { return exch_const(_force_update, false); }

  Camera _camera;
  bool _force_update;
};