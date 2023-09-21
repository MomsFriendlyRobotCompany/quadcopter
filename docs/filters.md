

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