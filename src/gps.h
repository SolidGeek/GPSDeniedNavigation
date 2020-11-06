#define _USE_MATH_DEFINES
 
#include <cmath>

#ifndef GPS_H_
#define GPS_H_

const double radius_earth = 6371e3; // Radius of the earch in meters

typedef struct{
    double latitude;
    double longitude;
} geo_point_t;

double distance_between_points( geo_point_t p1, geo_point_t p2 );

geo_point_t get_point_ahead( geo_point_t p1, double distance, double angle );

#endif