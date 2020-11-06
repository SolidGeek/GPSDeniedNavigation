
#include <iostream>
#include "src/gps.h"

geo_point_t skywatch = {
    56.880452, 9.838556
};

int main( int argc, char** argv )
{

    double distance = 100.0; // Meters
    double angle = -90.0; // Degress

    geo_point_t newpos = get_point_ahead( skywatch, distance, angle );

    printf("Position: %.6f, %.6f \n", newpos.latitude, newpos.longitude);

    printf("Distance: %.6f meters \n", distance_between_points(skywatch, newpos) );

}



