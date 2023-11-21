

```cpp
/// Low pass filter cut frequency for derivative calculation.
float filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";
// Examples for _filter:
// f_cut = 10 Hz -> _filter = 15.9155e-3
// f_cut = 15 Hz -> _filter = 10.6103e-3
// f_cut = 20 Hz -> _filter =  7.9577e-3
// f_cut = 25 Hz -> _filter =  6.3662e-3
// f_cut = 30 Hz -> _filter =  5.3052e-3
```

```c
#include <stdio.h>

// First-order low-pass filter structure
typedef struct {
    double alpha;  // Smoothing factor
    double y_prev; // Previous output
} LowPassFilter;

// Initialize the low-pass filter
void initLowPassFilter(LowPassFilter *filter, double cutoffFrequency, double samplingFrequency) {
    double tau = 1.0 / (2.0 * 3.141592653589793 * cutoffFrequency); // Time constant
    filter->alpha = 1.0 / (1.0 + tau * samplingFrequency);
    filter->y_prev = 0.0;
}

// Apply the low-pass filter to a new input sample
double filterSample(LowPassFilter *filter, double input) {
    // Update the filter
    double output = filter->alpha * input + (1.0 - filter->alpha) * filter->y_prev;

    // Save the current output for the next iteration
    filter->y_prev = output;

    return output;
}

int main() {
    // Example usage
    double cutoffFrequency = 5.0;  // Adjust this to your desired cutoff frequency
    double samplingFrequency = 100.0;  // Adjust this to your actual sampling frequency

    LowPassFilter filter;
    initLowPassFilter(&filter, cutoffFrequency, samplingFrequency);

    // Simulate the filter on a sample signal
    double inputSignal[] = {1.0, 2.0, 3.0, 4.0, 5.0};  // Replace this with your actual signal
    int numSamples = sizeof(inputSignal) / sizeof(inputSignal[0]);

    printf("Input signal: ");
    for (int i = 0; i < numSamples; i++) {
        double filteredValue = filterSample(&filter, inputSignal[i]);
        printf("%.2f  ", filteredValue);
    }

    return 0;
}
```

This code implements a simple low-pass filter with a first-order difference equation. The initLowPassFilter function initializes the filter with the specified cutoff frequency and sampling frequency, and the filterSample function applies the filter to a new input sample.

## Vectors

```cpp
struct vec_t {
    float x,y,z;
};

union uvec_t {
    float a[3];
    vec_t v;
};

uvec_t m;
m.a[0] = 5.0;
m.v.y = 2.0;
```


```cpp
struct vec_t {
    float x,y,z;
    float operator[](size_t i) {
        return (i == 0) ? x : (i == 1) ? y : z;
    }
};
```