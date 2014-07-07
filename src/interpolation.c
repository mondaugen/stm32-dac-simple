#include <math.h> 

/* Cubic interpolation of a table of doubles. Index must be valid table index. */
double cubic_interp_double(double index, double *table, size_t length)
{
    double y0, y1, y2;
    size_t x0, x1, x2;
    x1 = (size_t)floor(index);
    x0 = x1 + length - 1;
    while(x0 >= length) { x0 -= length; }
    x2 = x1 + length + 1;
    while(x2 >= length) { x2 -= length; }
    y0 = table[x0];
    y1 = table[x1];
    y2 = table[x2];
    return (index - x1) * (index - x2) / ((x0 - x1) * (x0 - x2)) * y0
        + (index - x0) * (index - x2) / ((x1 - x0) * (x1 - x2)) * y1
        + (index - x0) * (index - x1) / ((x2 - x0) * (x2 - x1)) * y2;
}
