#ifndef ANN_H_INCLUDED
#define ANN_H_INCLUDED


#define ANN_INPUTS 4
#define ANN_HIDEN_LAYERS 1
#define ANN_HIDEN_UNITS 16
#define ANN_OUTPUTS 8

void ann_forward_propagation(double * outputs, double const * inputs, double const * weights);

#endif // ANN_H_INCLUDED
