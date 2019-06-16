#include <stdlib.h>
#include <math.h>

#include "ann.h"

double sigmoid(double a)
{
    if (a < -45.0) return 0.0;
    if (a > 45.0) return 1.0;
    return 1.0 / (1.0 + exp(-a));
}

static double hiddens[ANN_HIDEN_LAYERS][ANN_HIDEN_UNITS];


void ann_forward_propagation(double * outputs, double const * inputs, double const * weights)
{
    // first hidden layer
    size_t layer = 0;
    for(size_t j=0;j<ANN_HIDEN_UNITS;++j)
    {
        double sum = 1.0 * *weights++; //bias
        for(size_t i=0;i<ANN_INPUTS;++i)
        {
            sum += inputs[i] * *weights++;
        }
        hiddens[layer][j] = sigmoid(sum);
    }
    // ouputs
    for(size_t j=0;j<ANN_OUTPUTS;++j)
    {
        double sum = 1.0 * *weights++; //bias
        for(size_t i=0;i<ANN_HIDEN_UNITS;++i)
        {
            sum += hiddens[layer][i] * *weights++;
        }
        outputs[j] = sigmoid(sum);
    }
}
