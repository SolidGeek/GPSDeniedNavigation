#include "gps.h"

// This uses the "haversine" formula to calculate the great-circle distance between two points
// https://www.movable-type.co.uk/scripts/latlong.html
double distance_between_points( geo_point_t p1, geo_point_t p2 ) {

    // Convert angles to radians
    double lat1 = p1.latitude * M_PI/180.0;
    double lat2 = p2.latitude * M_PI/180.0;

    double delta_lat  = (p2.latitude - p1.latitude) * M_PI/180.0;
    double delta_long = (p2.longitude - p1.longitude) * M_PI/180.0;

    double a = sin(delta_lat/2) * sin(delta_lat/2) + cos(lat1) * cos(lat2) * sin(delta_long/2) * sin(delta_long/2);
    double c = 2 * atan2( sqrt(a), sqrt(1-a) );

    return radius_earth * c;

}

// Calculate new position from current point, a distance and the heading
// http://mathforum.org/library/drmath/view/52049.html
geo_point_t get_point_ahead( geo_point_t p1, double distance, double angle ){

    geo_point_t p2; 

    // Angular distance in radians
    double alpha = distance / radius_earth; 

    // Convert angles to radians
    double heading = angle * M_PI/180.0;
    double lat1  = p1.latitude * M_PI/180.0;
    double long1 = p1.longitude * M_PI/180.0;

    // Calculate new position from moved distance
    double lat2  = asin( cos(heading) * cos(lat1) * sin(alpha) + sin(lat1) * cos(alpha) );
    double long2 = long1 + atan2( (sin(heading) * sin(alpha) * cos(lat1)) , (cos(alpha) - sin(lat1) * sin(lat2)) );

    p2.latitude = lat2 * 180.0/M_PI;
    p2.longitude = long2 * 180.0/M_PI;

    return p2;
} 