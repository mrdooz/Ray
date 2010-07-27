#include "stdafx.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <assert.h>
#include <concrt.h>
#include "ray_math.hpp"
#include "ray_march.hpp"
#include "camera.hpp"


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
struct RayMarcher::RenderJobData
{
  RenderJobData(int start_y, int num_lines, int width, int height, const Camera *camera) 
    : start_y(start_y), num_lines(num_lines), width(width), height(height), camera(camera), ptr(nullptr) {}
  int start_y;
  int num_lines;
  int width, height;
  const Camera *camera;
  void *ptr;
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

static void __cdecl RenderJob(LPVOID param)
{
	RayMarcher::RenderJobData *data = (RayMarcher::RenderJobData *)param;
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

static void __cdecl RenderJob2(LPVOID param)
{
  RayMarcher::RenderJobData *data = (RayMarcher::RenderJobData *)param;
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

void raycast(const Camera& c, void *ptr, int width, int height)
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


bool RayMarcher::init(int width, int height)
{
	int ofs = 0;
	int num_jobs = height / 4;
	int lines = height / num_jobs;
	while (ofs <= height) {
		_datas.push_back(new RayMarcher::RenderJobData(ofs, min(height-ofs, lines), width, height, &_camera));
		_events.push_back(&_datas.back()->signal);
		ofs += lines;
	}

	return true;
}

void RayMarcher::render(const Camera& c, void *ptr, int width, int height)
{
	for (int i = 0; i < (int)_datas.size(); ++i) {
		_datas[i]->ptr = ptr;
		CurrentScheduler::ScheduleTask(RenderJob, _datas[i]);
	}

	event::wait_for_multiple(&_events[0], _events.size(), true);
}

void RayMarcher::close()
{
	for (int i = 0; i < (int)_datas.size(); ++i)
		delete _datas[i];
	_datas.clear();
}

