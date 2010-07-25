#include "stdafx.h"
#include "camera.hpp"

void Camera::pixel_to_screen(int x, int y, int nx, int ny, float *a, float *b) const
{
	// convert pixel coordinates (0..width) to screen coordinates (0..1)
	// we assume that pixel coordinates point to the middle of the pixel, hence the 0.5 offset.
	// also, y = 0 is the top most pixel-coordinate, so we must flip y
	*a = (x + 0.5f) / nx;
	*b = (ny - y - 0.5f) / ny;
}

void Camera::screen_to_world(float a, float b, Vec3 *s) const
{
	// convert from [0..1]^2 screen coords to world coords
	*s = _c + a * _a + b * _b;
}


void Camera::create_frame()
{
	_frame.w = -_dir;
	_frame.u = normalize(cross(_up, _frame.w));
	_frame.v = cross(_frame.w, _frame.u);

	_a = (_u1 - _u0) * _frame.u;
	_b = (_v1 - _v0) * _frame.v;
	_c = _pos + _u0 * _frame.u + _v0 * _frame.v + _dist * -_frame.w;
}

void Camera::ray_to_world(const Vec3& s, Vec3 *o, Vec3 *d) const
{
	*o = _pos;
	*d = normalize(s - _pos);
}

void Camera::ray_from_pixel(int x, int y, int nx, int ny, Vec3 *o, Vec3 *d) const
{
	float a, b;
	pixel_to_screen(x, y, nx, ny, &a, &b);
	Vec3 s;
	screen_to_world(a, b, &s);
	ray_to_world(s, o, d);
}
