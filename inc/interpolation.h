#ifndef INTERPOLATION_H
#define INTERPOLATION_H 

/* Cubic interpolation of a table of doubles. Index must be valid table index. */
double cubic_interp_double(double index, double *table, size_t length);

#endif /* INTERPOLATION_H */
