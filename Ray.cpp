// Ray.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <assert.h>
#include <concrt.h>

extern "C"
{
#include <lua.h>
#include <lauxlib.h>
#include <lualib.h>
};

#pragma comment(lib, "sdl.lib")
#pragma comment(lib, "winmm.lib")

SDL_Surface *g_font = NULL;
SDL_Surface *g_screen  = NULL;

// we use a right-handed coordinate system (yes, this is going to confuse the hell out of me :)

//  +y
//  |  -z
//  | /
//  |/
//  +------ +x
//

struct Vec3
{
	Vec3() {}
	Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
	union {
		struct {
			float x, y, z;
		};
		float d[3];
	};

	float len() const;

	const float static kEps;

};

struct Matrix3x3
{
	Matrix3x3() {}
	Matrix3x3(float m11, float m12, float m13, float m21, float m22, float m23, float m31, float m32, float m33)
		: m11(m11), m12(m12), m13(m13), m21(m21), m22(m22), m23(m23), m31(m31), m32(m32), m33(m33) {}

	union {
		struct {
			float m11, m12, m13;
			float m21, m22, m23;
			float m31, m32, m33;
		};
		float d[3*3];
	};
};

Vec3 operator*(const Vec3& v, const Matrix3x3& m)
{
	return Vec3(
		v.x * m.m11 + v.y * m.m12 + v.z * m.m13,
		v.x * m.m21 + v.y * m.m22 + v.z * m.m23,
		v.x * m.m31 + v.y * m.m32 + v.z * m.m33
		);
}

Matrix3x3 rotate_x(float q)
{
	const float sin_q = sinf(q);
	const float cos_q = cosf(q);
	return Matrix3x3(
		1, 0, 0,
		0, cos_q, sin_q,
		0, -sin_q, cos_q
		);
}

Matrix3x3 rotate_y(float q)
{
	const float sin_q = sinf(q);
	const float cos_q = cosf(q);
	return Matrix3x3(
		cos_q, 0, -sin_q,
		0, 1, 0,
		sin_q, 0, cos_q
		);
}

Matrix3x3 rotate_z(float q)
{
	const float sin_q = sinf(q);
	const float cos_q = cosf(q);
	return Matrix3x3(
		cos_q, sin_q, 0,
		-sin_q, cos_q, 0,
		0, 0, 1
		);
}

const float Vec3::kEps = 0.00001f;

float Vec3::len() const
{
	return sqrtf(x*x + y*y + z*z);
}

Vec3 operator+(const Vec3& a, const Vec3& b)
{
	return Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

Vec3 operator-(const Vec3& a, const Vec3& b)
{
	return Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

Vec3 operator-(const Vec3& a)
{
	return Vec3(-a.x, -a.y, -a.z);
}

Vec3 operator/(const Vec3& a, float s)
{
	return Vec3(a.x / s, a.y / s, a.z / s);
}

Vec3 operator*(float s, const Vec3& a)
{
	return Vec3(s * a.x, s * a.y, s * a.z);
}

Vec3 normalize(const Vec3& a)
{
	Vec3 res;
	const float len = a.len();
	if (len < Vec3::kEps)
		res.x = res.y = res.z = 0;
	else
		res = a / len;
	return res;
}

float dot(const Vec3& a, const Vec3& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(const Vec3& a, const Vec3& b)
{
	return Vec3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}


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

struct Object
{
	virtual ~Object() {}
	virtual bool intersect(const Vec3& o, const Vec3& d, float *t) const = 0;
	virtual Vec3 calc_normal(const Vec3& p) const = 0;
};

typedef std::vector<Object *> Objects;

struct Sphere : public Object
{
	Sphere(const Vec3& center, float radius) : c(center), r(radius) {}

	virtual bool intersect(const Vec3& o, const Vec3& d, float *t) const;
	virtual Vec3 calc_normal(const Vec3& p) const;

	Vec3 c;
	float r;
};

bool Sphere::intersect(const Vec3& o, const Vec3& d, float *t) const
{
	// d.d * t^2 + 2*d.d(o-c) * t + (o-c).(o-c) - r^2
	//   A * t^2 +          B * t + C;
	// t = -B +- sqrt(B^2 - 4AC) / 2A

	const float A = dot(d, d);
	const float B = 2 * dot(d, o-c);
	const float C = dot(o-c, o-c) - r * r;
	const float discriminant = B * B - 4 * A * C;

	if (discriminant < 0)
		return false;

	const float t0 = (-B + sqrtf(discriminant)) / (2 * A);
	const float t1 = (-B - sqrtf(discriminant)) / (2 * A);

	*t = t1 >= 0 ? t1 : t0;
	return true;
}

Vec3 Sphere::calc_normal(const Vec3& p) const
{
	return normalize(p - c);
}

struct Plane : public Object
{
	Plane(const Vec3& a, const Vec3& n) : a(a), n(n) {}
	virtual bool intersect(const Vec3& o, const Vec3& d, float *t) const;
	virtual Vec3 calc_normal(const Vec3& p) const;

	Vec3 a, n;
};

bool Plane::intersect(const Vec3& o, const Vec3& d, float *t) const
{
	// plane eq: (p-a).n = 0
	// p = o + t * d
	// (o + t * d - a).n =0
	// t = (a-o).n / d.n

	*t = dot(a-o, n) / (dot(d, n));
	return *t >= 0;
}

Vec3 Plane::calc_normal(const Vec3& p) const
{
	return n;
}


struct BGRA32
{
	uint8_t b;
	uint8_t g;
	uint8_t r;
	uint8_t a;
};

Object *find_intersection(const Objects& objects, const Vec3& o, const Vec3& d, float *t)
{
	int idx = -1;
	float tmp, min_t = FLT_MAX;
	for (int i = 0; i < (int)objects.size(); ++i) {
		if (objects[i]->intersect(o, d, &tmp) && tmp < min_t) {
			min_t = tmp;
			idx = i;
		}
	}
  *t = min_t;
	return idx == -1 ? NULL : objects[idx];
}

float fn(float x, float z)
{
	return sin(x) * sin(z);
}

Vec3 get_normal(const Vec3& p)
{
	const float eps = 0.01f;
	const Vec3 n = Vec3( fn(p.x-eps,p.z) - fn(p.x+eps,p.z),
		2.0f*eps,
		fn(p.x,p.z-eps) - fn(p.x,p.z+eps) );
	return normalize(n);
}

using namespace Concurrency;

// renders from start_y, h scanlines down
struct RenderJobData
{
  RenderJobData(int start_y, int num_lines, int width, int height, void *ptr, const Camera *camera) 
    : start_y(start_y), num_lines(num_lines), width(width), height(height), ptr(ptr), camera(camera) {}
  int start_y;
  int num_lines;
  int width, height;
  void *ptr;
  const Camera *camera;
  event signal;
};

// can be replaced by (int)floorf(x)
static __forceinline int mfloorf( float x )
{
  x = x - 0.5f;
  int t;
  _asm fld x
  _asm fistp t
  return t;
}

// can be replaced by return powf( 2.0f, f );
static __forceinline float m2xf(float f)
{
  _asm fld   dword ptr [f]
  _asm fld1
  _asm fld   st(1)
  _asm fprem
  _asm f2xm1
  _asm faddp st(1), st
  _asm fscale
  _asm fstp  st(1)
  _asm fstp  dword ptr [f]
  return f;
}

static float clamp01( float x )
{
  if( x<0.0f ) x=0.0f;
  if( x>1.0f ) x=1.0f;
  return x;
}

static float smoothstep( float x, float a, float b )
{
  if( x<a ) return 0.0f;
  if( x>b ) return 1.0f;
  x = (x-a)/(b-a);
  return x*x*(3.0f-2.0f*x);
}

static float coolfFunc3d2( int n )
{
  n = (n << 13) ^ n;
  n = (n * (n * n * 15731 + 789221) + 1376312589) & 0x7fffffff;
  return (float)n;
}

static __forceinline float smoothstep3( float x ) { return x*x*(3.0f-2.0f*x); }
static __forceinline float smoothstep5( float x ) { return x*x*x*(x*(x*6.0f-15.0f)+10.0f); }
static __forceinline float lerp( float x, float a, float b ) { return a+(b-a)*x; }

static float noise3f( const float x, const float y, const float z, int sem )
{
  const int ix = mfloorf( x );
  const int iy = mfloorf( y );
  const int iz = mfloorf( z );

  const float u = smoothstep3( x-(float)ix );
  const float v = smoothstep3( y-(float)iy );
  const float w = smoothstep3( z-(float)iz );

  const int n = ix + 57*iy + 113*iz + sem;

  const float res = lerp(w, lerp(v, lerp(u, coolfFunc3d2(n+(0+57*0+113*0)),
    coolfFunc3d2(n+(1+57*0+113*0))),
    lerp(u, coolfFunc3d2(n+(0+57*1+113*0)),
    coolfFunc3d2(n+(1+57*1+113*0)))),
    lerp(v, lerp(u, coolfFunc3d2(n+(0+57*0+113*1)),
    coolfFunc3d2(n+(1+57*0+113*1))),
    lerp(u, coolfFunc3d2(n+(0+57*1+113*1)),
    coolfFunc3d2(n+(1+57*1+113*1)))));
  return 1.0f - res*(1.0f/1073741824.0f);
}

static float fbm( float x, float y, float z )
{
  float v = 0.5000f*noise3f( x*1.0f, y*1.0f, z*1.0f, 0 ) + 
    0.2500f*noise3f( x*2.0f, y*2.0f, z*2.0f, 0 ) + 
    0.1250f*noise3f( x*4.0f, y*4.0f, z*4.0f, 0 ) +
    0.0625f*noise3f( x*8.0f, y*8.0f, z*8.0f, 0 );
  return v;
}

static float distToBox( float x, float y, float z, float a, float b, float c )
{
  float di = 0.0f;
  const float dx = fabsf(x)-a; if( dx>0.0f ) di+=dx*dx;
  const float dy = fabsf(y)-b; if( dy>0.0f ) di+=dy*dy;
  const float dz = fabsf(z)-c; if( dz>0.0f ) di+=dz*dz;
  return di;

}

static float columna( float x, float y, float z, float mindist, float offx )
{
  const float di0 = distToBox( x, y, z, 0.14f, 1.0f, 0.14f );

  if( di0 > (mindist*mindist) ) return mindist + 1.0f;

  const float y2=y-0.40f;
  const float y3=y-0.35f;
  const float y4=y-1.00f;

  const float di1 = distToBox( x, y , z, 0.10f, 1.00f, 0.10f );
  const float di2 = distToBox( x, y , z, 0.12f, 0.40f, 0.12f );
  const float di3 = distToBox( x, y , z, 0.05f, 0.35f, 0.14f );
  const float di4 = distToBox( x, y , z, 0.14f, 0.35f, 0.05f );
  const float di9 = distToBox( x, y4, z, 0.14f, 0.02f, 0.14f );
  const float di5 = distToBox( (x-y2)*0.7071f, (y2+x)*0.7071f, z,      0.10f*0.7071f, 0.10f*0.7071f, 0.12f );
  const float di6 = distToBox(              x, (y2+z)*0.7071f, (z-y2)*0.7071f, 0.12f, 0.10f*0.7071f, 0.1f*0.7071f );
  const float di7 = distToBox( (x-y3)*0.7071f, (y3+x)*0.7071f, z,      0.10f*0.7071f, 0.10f*0.7071f, 0.14f );
  const float di8 = distToBox(              x, (y3+z)*0.7071f, (z-y3)*0.7071f, 0.14f, 0.10f*0.7071f, 0.10f*0.7071f );

  float di = di1;
  if( di2<di ) di=di2;
  if( di3<di ) di=di3;
  if( di4<di ) di=di4;
  if( di5<di ) di=di5;
  if( di6<di ) di=di6;
  if( di7<di ) di=di7;
  if( di8<di ) di=di8;
  if( di9<di ) di=di9;

  const float fb = fbm(10.1f*x+offx,10.1f*y,10.1f*z);
  if( fb>0.0f )
    di = di + 0.00000003f*fb;

  return di;
}

static __forceinline unsigned int coolfFunc3d3( unsigned int n )
{
  n = (n << 13) ^ n;
  n = (n * (n * n * 15731 + 789221) + 1376312589) & 0x7fffffff;
  return n;
}


static float bicho( float x, float y, float z, float mindist )
{
  x = x-0.64f;
  y = y-0.50f;
  z = z-1.50f;

  float r2 = x*x + y*y + z*z;

  float sa = smoothstep(r2,0.0f,0.5f);
  float fax = 0.75f + 0.25f*sa;
  float fay = 0.80f + 0.20f*sa;
  x *= fax;
  y *= fay;
  z *= fax;
  r2 = x*x + y*y + z*z;


  // bighacks
#if 1
  if( r2>5.0f ) return mindist;
  if( y >0.5f ) return mindist;
  if( y>-0.20f && (x*x+z*z)>0.60f ) return mindist;
  if( r2>(1.70f+mindist)*(1.70f+mindist)  ) return mindist;
#endif

  const float r = sqrtf(r2);

  if( r<0.75f )
  {
    float a1 = 1.0f-smoothstep( r, 0.0f, 0.75 );
    a1 *= 0.60f;
    const float si1 = sinf(a1);
    const float co1 = cosf(a1);
    float nx = x;
    float ny = y;
    x = nx*co1 - ny*si1;
    y = nx*si1 + ny*co1;
  }


  float mindist2 = 100000.0f;
  const float p[3] = { x, y, z };

  const float rr = 0.05f+sqrtf(x*x+z*z);
  const float ca = (0.5f-0.045f*0.75f) -6.0f*rr*m2xf(-10.0f*rr);
  for( int j=1; j<7; j++ )
  {
    const float an = (6.2831f/7.0f) * (float)j;
    const float aa = an + 0.40f*rr*noise3f(4.0f*rr, 2.5f, an, 0 ) + 0.29f;
    const float rc = cosf(aa);
    const float rs = sinf(aa);
    const float q[3] = { p[0]*rc-p[2]*rs, p[1]+ca, p[0]*rs+p[2]*rc };
    const float dd = q[1]*q[1] + q[2]*q[2];
    if( q[0]>0.0f && q[0]<1.5f && dd<mindist2 ) mindist2=dd;
  }

  const float c = sqrtf(mindist2) - 0.045f;
  const float d = r-0.30f;

  const float a = clamp01( r*3.0f );
  return c*a + d*(1.0f-a);
}


static float techo2( float x, float y )
{
  y = 1.0f - y;
  if( x<0.1f || x>0.9f )  return y;
  x = x - 0.5f;
  return -(sqrtf(x*x+y*y)-0.4f);
}

float dist( float x, float y, float z, int *sid )
{
  float mindist;// = 1e20f;
  float dis;

  //-----------------------
  // floor
  //-----------------------
  dis = y;

  const float ax = 128.0f + (x+z)*6.0f;
  const float az = 128.0f + (x-z)*6.0f;
  const unsigned int ix = mfloorf(ax);
  const unsigned int iz = mfloorf(az);
  const int submat = coolfFunc3d3(ix+53*iz);
  const int ba = ( ((submat>>10)&7)>6 );
  const float peldx = fmodf( ax, 1.0f );
  const float peldz = fmodf( az, 1.0f );
  float peld = peldx; if( peldz>peld) peld=peldz;
  peld = smoothstep( peld, 0.975f, 1.0f );
  if( ba )peld = 1.0f;
  dis += 0.005f*peld;
  mindist = dis;
  if( peld>0.0000001f ) sid[0] = 2; else sid[0] = 0;
  sid[0] = sid[0]+(submat<<8);

  //-----------------------
  // roof
  //-----------------------
  const float fx = fmodf( x+128.0f, 1.0f );
  const float fz = fmodf( z+128.0f, 1.0f );

  if( y>1.0f )
  {
    dis  = techo2( fx, y );
    float disz = techo2( fz, y );
    if( disz>dis ) dis=disz;
    if( dis<mindist )
    {
      mindist = dis;
      sid[0] = 5;
    }
  }

  //-----------------------
  // columns
  //-----------------------
  const float fxc = fmodf( x+128.5f, 1.0f ) - 0.5f;
  const float fzc = fmodf( z+128.5f, 1.0f ) - 0.5f;
  dis = columna( fxc, y, fzc, mindist, 13.1f*(int)(x)+17.7f*(int)z );
  if( dis<(mindist*mindist) )
  {
    mindist = sqrtf(dis);
    sid[0] = 1;
  }

  //-----------------------
  // monster
  //-----------------------
  dis = bicho( x, y, z, mindist );
  if( dis<mindist )
  {
    mindist = dis;
    sid[0] = 4;
  }

  return mindist;
}

static void generateRay( Vec3 *ro, Vec3 *rd, int i, int j, int xres, int yres )
{
  // screen coords
  const float sx = -1.75f + 3.5f*(float)i/(float)xres;
  const float sy =  1.00f - 2.0f*(float)j/(float)yres;

  // bend rays (fish eye)
  const float r2 = sx*sx*0.32f + sy*sy;
  const float tt = (7.0f-sqrtf(37.5f-11.5f*r2))/(r2+1.0f);
  const float dx = sx*tt;
  const float dy = sy*tt;

  // rotate ray
  rd->x = dx*0.955336f + 0.29552f;
  rd->y = dy;
  rd->z = 0.955336f - dx*0.29552f;

  // normalize
  normalize( *rd );

  //
  ro->x = 0.195f;
  ro->y = 0.5f;
  ro->z = 0.0f;
}


// distance from unit sphere at (0,0,0)
float sphere_distance(const Vec3& p)
{
	return p.len() - 1;
}

template<class T>
T max3(const T& a, const T& b, const T& c)
{
	return max(a, max(b, c));
}

// unit cube at (0,0,0)
float cube_distance(const Vec3& p)
{
  return max3(fabs(p.x), fabs(p.y), fabs(p.z)) - 1;
}

float inline deg_to_rad(const float deg)
{
	return 3.1415926f * deg / 180.0f;
}

float distance(const Vec3& p)
{
  int a;
  return dist(p.x, p.y, p.z, &a);
  Vec3 t = p * rotate_y(p.y);
	return cube_distance(t);
}

void __cdecl RenderJob(LPVOID param)
{
	RenderJobData *data = (RenderJobData *)param;
	data->signal.reset();

	Vec3 o, d;
	Vec3 light_pos(0,100,0);

	const float kEps = 0.001f;

	BGRA32 *p = (BGRA32 *)data->ptr + data->start_y * data->width;
	for (int y = data->start_y; y < data->start_y + data->num_lines; ++y) {
		for (int x = 0; x < data->width; ++x) {

      
      generateRay(&o, &d, x, y, data->width, data->height);

			//data->camera->ray_from_pixel(x, y, data->width, data->height, &o, &d);

			// ray marching
			bool found = false;
			float min_t = 0.1f, max_t = 100;
			float t = min_t;
			while (t < max_t) {
				// find closest intersection
				Vec3 p0 = o + t * d;
				float closest = distance(p0);
				if (closest <= kEps) {
          p0 = o + (t + closest) * d;
					Vec3 n = normalize(Vec3( 
						distance(p0 + Vec3(kEps, 0, 0)) - distance(p0 - Vec3(kEps, 0, 0)),
						distance(p0 + Vec3(0, kEps, 0)) - distance(p0 - Vec3(0, kEps, 0)),
						distance(p0 + Vec3(0, 0, kEps)) - distance(p0 - Vec3(0, 0, kEps))));

					Vec3 l = normalize(light_pos - p0);
					float diffuse = dot(n, l);
					float ambient = 0.2f;
					float col = min(1, max(0, ambient + diffuse));
					p->r = p->g = p->b = p->a = (uint8_t)(255 * col);
					found = true;
					break;
				}
				t += closest;
			}
			if (!found)
				p->r = p->g = p->b = p->a = 0;

			p++;
		}
	}

	data->signal.set();
}

void __cdecl RenderJob2(LPVOID param)
{
  RenderJobData *data = (RenderJobData *)param;
  data->signal.reset();

  Vec3 o, d;
  Vec3 light_pos(0,100,0);

  BGRA32 *p = (BGRA32 *)data->ptr + data->start_y * data->width;
  for (int y = data->start_y; y < data->start_y + data->num_lines; ++y) {
    for (int x = 0; x < data->width; ++x) {

      data->camera->ray_from_pixel(x, y, data->width, data->height, &o, &d);

      bool found = false;
      float min_t = 0, max_t = 1000, dt = 5;
      for (float t = min_t; t < max_t; t += dt) {
        Vec3 p0 = o + t * d;
        if (p0.y < fn(p0.x, p0.z)) {
          Vec3 p0 = o + (t - 0.5f * dt) * d;
          Vec3 l = normalize(light_pos - p0);
          Vec3 n = get_normal(p0);
          float diffuse = dot(n, l);
          float col = min(1, max(0, diffuse));
          p->r = p->g = p->b = p->a = (uint8_t)(255 * col);
          found = true;
          break;
        }
      }
      if (!found)
        p->r = p->g = p->b = p->a = 0;

      p++;
    }
  }

  data->signal.set();
}

void raycast(const Camera& c, const Objects& objects, void *ptr, int width, int height)
{
	Vec3 o, d;
	Vec3 light_pos(0,100,-150);

	BGRA32 *p = (BGRA32 *)ptr;
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {

			c.ray_from_pixel(x, y, width, height, &o, &d);

			bool found = false;
			float min_t = 0, max_t = 1000, dt = 5;
			for (float t = min_t; t < max_t; t += dt) {
				Vec3 p0 = o + t * d;
				if (p0.y < fn(p0.x, p0.z)) {
					Vec3 p0 = o + (t - 0.5f * dt) * d;
					Vec3 l = normalize(light_pos - p0);
					Vec3 n = get_normal(p0);
					float diffuse = dot(n, l);
					float col = min(1, max(0, diffuse));
					p->r = p->g = p->b = p->a = (uint8_t)(255 * col);
					found = true;
					break;
				}
			}
			if (!found)
				p->r = p->g = p->b = p->a = 0;

			p++;
		}
	}

}

void raytrace(const Camera& c, const Objects& objects, void *ptr, int width, int height)
{
	Vec3 o, d;
	Vec3 light_pos(0,100,-150);

	BGRA32 *p = (BGRA32 *)ptr;
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {

			float t;
			c.ray_from_pixel(x, y, width, height, &o, &d);

			if (Object *obj = find_intersection(objects, o, d, &t)) {
				Vec3 p0 = o + t * d;

        Vec3 probe = (light_pos - p0);
        const float light_dist = probe.len();
        normalize(probe);
        float tmp;
        Object *occulder = find_intersection(objects, p0 + 0.1f * probe, probe, &tmp);
        if (occulder && tmp > 0 && tmp < light_dist) {
          p->r = p->g = p->b = p->a = 0;
        } else {
          Vec3 l = normalize(light_pos - p0);
          Vec3 n = obj->calc_normal(p0);
          Vec3 v = normalize(c._pos - p0);
          Vec3 h = normalize(l + v);
          float spec = powf(dot(n, h), 32);
          float diffuse = dot(n, l);
					float ambient = 0.2f;
          float col = min(1, max(0, diffuse + spec));
          p->r = p->g = p->b = p->a = (uint8_t)(255 * col);
        }
			} else {
				p->r = p->g = p->b = p->a = 0;
			}
			p++;
		}
	}

}



struct CSDL_Rect : public SDL_Rect
{
	CSDL_Rect(Sint16 x, Sint16 y, Uint16 w, Uint16 h)
	{
		this->x = x; this->y = y;
		this->w = w; this->h = h;
	}
};

const int kCharWidth = 14;
const int kCharHeight = 24;
void draw_char(char ch, int x, int y)
{
	CSDL_Rect src(0, (ch - 32) * kCharHeight, kCharWidth, kCharHeight), dst(x, y, kCharWidth, kCharHeight);
	SDL_BlitSurface(g_font, &src, g_screen, &dst);
}

void draw_string(int x, int y, const char *fmt, ...)
{
	va_list arg;
	va_start(arg, fmt);

	const int len = _vscprintf(fmt, arg) + 1;
	char* buf = (char*)_alloca(len);
	vsprintf_s(buf, len, fmt, arg);

	while (*buf) {
		draw_char(*buf,x,y);
		x += kCharWidth;
		buf++;
	}
	va_end(arg);
}

int run_raycasting()
{
  SDL_Init(SDL_INIT_EVERYTHING);

  const int width = GetSystemMetrics(SM_CXSCREEN) / 2;
  const int height = GetSystemMetrics(SM_CYSCREEN) / 2;

  g_screen = SDL_SetVideoMode(width, height, 32, SDL_DOUBLEBUF);

  SDL_Surface *temp = SDL_LoadBMP("font14x24.bmp");
  g_font = SDL_ConvertSurface(temp, g_screen->format, SDL_SWSURFACE);
  SDL_FreeSurface(temp);
  SDL_SetColorKey(g_font, SDL_SRCCOLORKEY, 0);

  Camera c;
  c._pos = Vec3(0,4, 15);
  c._up = Vec3(0,1,0);
  c._dir = Vec3(0,0,-1);

  std::vector<RenderJobData *> datas;
  std::vector<event *> events;
  int ofs = 0;
  int num_jobs = height / 4;
  int lines = height / num_jobs;
  while (ofs <= height) {
    datas.push_back(new RenderJobData(ofs, min(height-ofs, lines), width, height, NULL, &c));
    events.push_back(&datas.back()->signal);
    ofs += lines;
  }

  bool done = false;
  bool first_time = true;
  while (!done) {

    SDL_Event event;
    bool redraw = false;
    if (SDL_PollEvent(&event)) {
      switch(event.type) {

      case SDL_QUIT:
        done = true;
        break;

      case SDL_KEYDOWN:
        switch (event.key.keysym.sym) {
        case SDLK_a: c._pos.z -= 1; redraw = true; break;
        case SDLK_z: c._pos.z += 1; redraw = true; break;
        case SDLK_UP: c._pos.y += 1; redraw = true; break;
        case SDLK_DOWN: c._pos.y -= 1; redraw = true; break;
        case SDLK_LEFT: c._pos.x -= 1; redraw = true; break;
        case SDLK_RIGHT: c._pos.x += 1; redraw = true; break;
        case SDLK_ESCAPE: done = true; break;
        }
        break;
      }
    }

    if (redraw || first_time) {
      first_time = false;
      c._dir = normalize(Vec3(0,0,-200) - c._pos);
      c._dir = normalize(Vec3(0,0,0) - c._pos);

      // scale the view plane by the aspect ratio of the bitmap to get square pixels
      const float aspect = (float)width / height;
      const float size = 10;
      c._u0 = -aspect * size;
      c._u1 = +aspect * size;
      c._v0 = -size;
      c._v1 = +size;
      c._dist = 100;

      c.create_frame();

      SDL_LockSurface(g_screen);

      DWORD start = timeGetTime();

      for (int i = 0; i < (int)datas.size(); ++i) {
        datas[i]->ptr = g_screen->pixels;
        CurrentScheduler::ScheduleTask(RenderJob, datas[i]);
      }

      event::wait_for_multiple(&events[0], events.size(), true);

      //raycast(c, objects, g_screen->pixels, g_screen->w, g_screen->h);
      DWORD elapsed = timeGetTime() - start;
      SDL_UnlockSurface(g_screen);
      draw_string(0, 0, "time: %.3fs", elapsed / 1000.0f);
      draw_string(0, kCharHeight, "cam pos: %.3f, %.3f, %.3f dir: %.3f, %.3f, %.3f", c._pos.x, c._pos.y, c._pos.z, c._dir.x, c._dir.y, c._dir.z);
      draw_string(0, 2 * kCharHeight, "a: %.3f, %.3f, %.3f", c._a.x, c._a.y, c._a.z);
      draw_string(0, 3 * kCharHeight, "b: %.3f, %.3f, %.3f", c._b.x, c._b.y, c._b.z);
      draw_string(0, 4 * kCharHeight, "c: %.3f, %.3f, %.3f", c._c.x, c._c.y, c._c.z);
      SDL_Flip(g_screen);
    }

  }

  for (int i = 0; i < (int)datas.size(); ++i)
    delete datas[i];
  datas.clear();

  SDL_FreeSurface(g_font);
  SDL_Quit();
  return 0;
}

bool load_scene(const char *filename, lua_State **ll)
{
  lua_State *l = *ll = lua_open();
  if (l == NULL)
    return false;
  luaL_openlibs(l);

  if (luaL_loadfile(l,  filename))
    return false;

  return true;
}

int run_raytracing()
{
  lua_State *l;
  if (!load_scene("scene1.lua", &l))
    return 1;

  SDL_Init(SDL_INIT_EVERYTHING);

  const int width = GetSystemMetrics(SM_CXSCREEN) / 2;
  const int height = GetSystemMetrics(SM_CYSCREEN) / 2;

  g_screen = SDL_SetVideoMode(width, height, 32, SDL_DOUBLEBUF);

  Objects objects;

  objects.push_back(new Sphere(Vec3(-10, 0, -200), 10));
  objects.push_back(new Sphere(Vec3(+10, 5, -200), 10));
  objects.push_back(new Sphere(Vec3(0, 0, -240), 10));
  objects.push_back(new Plane(Vec3(0, -10, 0), Vec3(0,1,0)));

  SDL_Surface *temp = SDL_LoadBMP("font14x24.bmp");
  g_font = SDL_ConvertSurface(temp, g_screen->format, SDL_SWSURFACE);
  SDL_FreeSurface(temp);
  SDL_SetColorKey(g_font, SDL_SRCCOLORKEY, 0);


  Camera c;
  c._pos = Vec3(0,4, 15);
  c._up = Vec3(0,1,0);
  c._dir = Vec3(0,0,-1);

  bool done = false;
  bool first_time = true;
  while (!done) {

    SDL_Event event;
    bool redraw = false;
    if (SDL_PollEvent(&event)) {
      switch(event.type) {

      case SDL_QUIT:
        done = true;
        break;

      case SDL_KEYDOWN:
        switch (event.key.keysym.sym) {
        case SDLK_a: c._pos.z -= 1; redraw = true; break;
        case SDLK_z: c._pos.z += 1; redraw = true; break;
        case SDLK_UP: c._pos.y += 1; redraw = true; break;
        case SDLK_DOWN: c._pos.y -= 1; redraw = true; break;
        case SDLK_LEFT: c._pos.x -= 1; redraw = true; break;
        case SDLK_RIGHT: c._pos.x += 1; redraw = true; break;
        case SDLK_ESCAPE: done = true; break;
        }
        break;
      }
    }

    if (redraw || first_time) {
      first_time = false;
      c._dir = normalize(Vec3(0,0,-200) - c._pos);

      // scale the view plane by the aspect ratio of the bitmap to get square pixels
      const float aspect = (float)width / height;
      const float size = 10;
      c._u0 = -aspect * size;
      c._u1 = +aspect * size;
      c._v0 = -size;
      c._v1 = +size;
      c._dist = 100;

      c.create_frame();

      SDL_LockSurface(g_screen);
      DWORD start = timeGetTime();
      raytrace(c, objects, g_screen->pixels, g_screen->w, g_screen->h);
      DWORD elapsed = timeGetTime() - start;
      SDL_UnlockSurface(g_screen);
      draw_string(0, 0, "time: %.3fs", elapsed / 1000.0f);
      draw_string(0, kCharHeight, "cam pos: %.3f, %.3f, %.3f dir: %.3f, %.3f, %.3f", c._pos.x, c._pos.y, c._pos.z, c._dir.x, c._dir.y, c._dir.z);
      draw_string(0, 2 * kCharHeight, "a: %.3f, %.3f, %.3f", c._a.x, c._a.y, c._a.z);
      draw_string(0, 3 * kCharHeight, "b: %.3f, %.3f, %.3f", c._b.x, c._b.y, c._b.z);
      draw_string(0, 4 * kCharHeight, "c: %.3f, %.3f, %.3f", c._c.x, c._c.y, c._c.z);
      SDL_Flip(g_screen);
    }
  }

  for (int i = 0; i < (int)objects.size(); ++i)
    delete objects[i];
  objects.clear();

  SDL_FreeSurface(g_font);

  SDL_Quit();

  return 0;
}


int _tmain(int argc, _TCHAR* argv[])
{
  return run_raytracing();
}
