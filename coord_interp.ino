#include "coord_interp.h"

// Find y for x: interpolate between values in lookup table.
// c: lookup table
// n: size of table
double interp( coord_t* c, double x, int n )
{
    int i;

    for( i = 0; i < n-1; i++ )
    {
        if ( c[i].x <= x && c[i+1].x >= x )
        {
            double diffx = x - c[i].x;
            double diffn = c[i+1].x - c[i].x;

            return c[i].y + ( c[i+1].y - c[i].y ) * diffx / diffn; 
        }
    }

    return 0; // Not in Range
}
