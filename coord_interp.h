// https://stackoverflow.com/questions/7091294/how-to-build-a-lookup-table-in-c-sdcc-compiler-with-linear-interpolation

#ifndef _COORD_INTERP_H
#define _COORD_INTERP_H

typedef struct { double x; double y; } coord_t;

double interp( coord_t* c, double x, int n );

#endif
