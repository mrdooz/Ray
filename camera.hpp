#pragma once
#include "ray_math.hpp"

struct Frame
{
	Vec3 u, v, w;
};

// model according to "realistic ray-tracing", page 66
struct Camera
{

	void create_frame();

	void pixel_to_screen(int x, int y, int width, int height, float *a, float *b) const;
	void screen_to_world(float a, float b, Vec3 *world) const;
	void ray_to_world(const Vec3& s, Vec3 *o, Vec3 *d) const;

	void ray_from_pixel(int x, int y, int nx, int ny, Vec3 *o, Vec3 *d) const;

	Vec3 _pos;
	Vec3 _dir;
	Vec3 _up;
	float _dist;
	float _u0, _v0;
	float _u1, _v1;

	// transient values
	Frame _frame;
	Vec3 _a, _b, _c;
};
