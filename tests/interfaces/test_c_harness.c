#include <stdio.h>
#include <math.h>

// Forward declaration of the generated function
void evaluate_closed_loop(const float* x, const float* u, float* out_dx);

int main() {
    // 4 states: DoubleIntegrator (pos, speed) + FilteredController (integral, filter)
    float x[4] = {1.0f, 0.0f, 0.0f, 0.0f}; 
    float u[1] = {0.0f}; // Dummy input for diagram reference (even if dim=0)
    float dx[4];
    
    evaluate_closed_loop(x, u, dx);
    
    printf("State  x: [");
    for (int i = 0; i < 4; i++) {
        printf("%f ", x[i]);
    }
    printf("]\n");

    printf("Deriv dx: [");
    for (int i = 0; i < 4; i++) {
        printf("%f ", dx[i]);
    }
    printf("]\n");

    return 0;
}
