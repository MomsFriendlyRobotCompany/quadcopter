
#pragma once

#include <algorithm>
#include <math.h>
#include "units.hpp"
#include "common.hpp"

using std::min;
using std::max;

// namespace Units {
// constexpr float deg2rad = M_PI/180.0;
// constexpr float rad2deg = 180.0/M_PI;
// };

// struct llh_t {float lat, lon, h;};
// struct ecef_t {float x, y, z;};

// FIXME: This is a MESS ... what do I want to do?

// class WGS84 {
//   public:
namespace WGS84 {
  constexpr float INV_FLATTENING     = 298.257223563;
  constexpr float FLATTENING         = 1.0 / INV_FLATTENING;       // f
  constexpr float SEMI_MAJOR_AXIS_M  = 6378137.0;                  // a, m
  constexpr float SEMI_MAJOR_AXIS_KM = SEMI_MAJOR_AXIS_M / 1000.0; // a, Km
  constexpr float SEMI_MINOR_AXIS_M  = SEMI_MAJOR_AXIS_M - SEMI_MAJOR_AXIS_M * FLATTENING; // b, m
  // constexpr float STD_PRESSURE_PA    = 101325.0;                   // Pa
  constexpr float SPIN_RATE_RPS      = 7.2921150e-5;               // rad / sec
  constexpr float G0                 = 9.7803253359; // Gravity [m/sec^2]

  // float rf = 298.257223563;
  // const float f = 1.0/FLATTENING;
  // const float a = SEMI_MAJOR_AXIS_M;
  // const float b = a - a * f;
  constexpr float RADIUS = (2.0*SEMI_MAJOR_AXIS_M + SEMI_MINOR_AXIS_M) / 3.0;
  constexpr float ROTATIONAL_PERIOD = 23*3600 + 56*60 + 4.09053;

  const float f = FLATTENING;
  const float a = SEMI_MAJOR_AXIS_M;
  const float b = SEMI_MINOR_AXIS_M;
  const float r = RADIUS;
  const float e = sqrt(1.0 - ( (b*b) / (a*a) ));

  float rate = SPIN_RATE_RPS;  // Rotation rate of Earth [rad/s]
  float sf = 1.2383e-3;       // Schuller frequency

  // float gravity(float lat) {
  //   /*
  //   Based off the Oxford reference for the gravity formula at sealevel.

  //   lat: latitude in degrees
  //   */
  //   lat *= deg2rad; // pi/180
  //   float sinflat = sinf(lat);
  //   float sinf2lat = sinf(2.0f*lat);
  //   return 9.78f*(1.0f + 0.0053024f*sinflat*sinflat - 0.0000058f*sinf2lat*sinf2lat);
  // }

  float gravity(float lat_deg) {
    /*
    Based off the Oxford reference for the gravity formula at sealevel.
    https://www.oxfordreference.com/view/10.1093/oi/authority.20110803100007626

    Also the WGS84 has a newer model, but it is more computationally
    intensive and only differs from this one by 0.68 um/s^2
    https://en.wikipedia.org/wiki/Gravity_of_Earth

    lat: latitude [decimal deg], North is positive and South is negative
    */
    float lat = lat_deg*Units::deg2rad;
    return G0*(1.0 + 0.0053024*pow(sin(lat),2) - 0.0000058*pow(sin(2.0*lat),2));
  }

  ecef_t llh2ecef(const llh_t& llh) {
    // matlab: https://www.mathworks.com/help/aeroblks/llatoecefposition.html
    // this works, matches: https://www.oc.nps.edu/oc2902w/coord/llhxyz.htm
    // float lat = llh.lat;
    // float lon = llh.lon;
    const float H = llh.h;

    const float mu = llh.lat*Units::deg2rad;
    const float i = llh.lon*Units::deg2rad;
    const float r = SEMI_MAJOR_AXIS_M; //6378137.0;
    // float f = 1.0/298.257223563;
    const float ls = atan(pow(1.0-f,2) * tan(mu));
    const float rs = sqrt(r*r / (1.0 + (1.0/pow(1.0-f,2) - 1.0)*pow(sin(ls),2)));

    const float x = rs*cos(ls)*cos(i) + H*cos(mu)*cos(i);
    const float y = rs*cos(ls)*sin(i) + H*cos(mu)*sin(i);
    const float z = rs*sin(ls)+H*sin(mu);

    return ecef_t{x,y,z};
  }

  llh_t ecef2llh(const ecef_t& ecef) {
    const float a = SEMI_MAJOR_AXIS_M;
    const float a2 = a*a;
    const float b2 = SEMI_MINOR_AXIS_M;
    // const float e2 =
    return llh_t{0,0,0};
  }

  float haversine(const llh_t& a, const llh_t& b) {
    // Returns the haversine (or great circle) distance between
    // 2 sets of GPS coordinates. This appears to work really well.
    //
    // a: (lat, lon) in decimal deg
    // b: (lat, lon) in decimal deg
    const float alat = a.lat*Units::deg2rad;
    const float blat = b.lat*Units::deg2rad;
    const float alon = a.lon*Units::deg2rad;
    const float blon = b.lon*Units::deg2rad;
    const float dlat = (blat - alat);
    const float dlon = (blon - alon);
    const float m = pow(sin(dlat*0.5),2) + cos(alat) * cos(blat) * pow(sin(dlon*0.5),2);
    return r*2.0*asin(min(1.0f, sqrt(m)));
  }

  /*
  Returns the geocentric radius based on WGS84
  lat: latitude in deg
  */
  float radius(float lat) {
    lat *= Units::deg2rad;
    const float a = SEMI_MAJOR_AXIS_M;
    const float b = a - a * FLATTENING;
    const float num = pow(a * a * cos(lat), 2) + pow(b * b * sin(lat), 2);
    const float den = pow(a * cos(lat), 2) + pow(b * sin(lat), 2);
    return sqrt(num / den);
  }
};
