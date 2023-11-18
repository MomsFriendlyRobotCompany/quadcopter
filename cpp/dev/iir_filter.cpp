
#include <stdio.h>
#include <cstring>

#define FILTER_SIZE 3 // Size of the filter

template<uint32_t N>
class IIRFilter {
  float a[N], b[N], y[N], x[N];
  public:
  IIRFilter(float num[N], float den[N], float init_value) {
    memcpy(b, num, N);
    memcpy(a, den, N);
    memset(x,0,N*sizeof(float));
    memset(y,init_value,N*sizeof(float));
  }

  // Function to apply IIR filter
  float update(float in) {
    memmove(x,&x[1],N-1);
    x[0] = in;
    float out = b[0]*x[0];
    for (uint32_t i=1; i<N; ++i) {
      out += b[i]*x[i] - a[i]*y[i];
    }
    memmove(y, &y[1], N-1);
    y[0] = out;
    return out;
  }
};

int main() {
  // Define the filter coefficients
  float b[FILTER_SIZE] = {1.0, -0.5, 0.2}; // Numerator coefficients
  float a[FILTER_SIZE] = {1.0, -0.7, 0.1}; // Denominator coefficients

  // Input signal
  float input[] = {1.0, 2.0, 3.0, 4.0, 5.0};

  // Output signal
  float output[5];

  // Apply the IIR filter
  // iirFilter(input, output, 5, b, a, N);
  IIRFilter<FILTER_SIZE> iir(b,a,1.0f);

  for (int i=0; i<5; ++i) output[i] = iir.update(input[i]);

  // Display the input and output signals
  printf("Input signal: ");
  for (int i = 0; i < 5; i++) {
    printf("%.2f  ", input[i]);
  }

  printf("\nOutput signal: ");
  for (int i = 0; i < 5; i++) {
    printf("%.2f  ", output[i]);
  }

  return 0;
}
