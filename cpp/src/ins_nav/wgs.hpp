
// #pragma once

// constexpr float rad2deg = M_PI / 180.0f;
// constexpr float deg2rad = 180.0f / M_PI;

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