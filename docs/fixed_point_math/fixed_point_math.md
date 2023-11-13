
- [ref](https://vanhunteradams.com/FixedPoint/FixedPoint.html)


Probably use this instead? It appears good and has all of the function (cos, sin, etc ...)

https://github.com/MikeLankamp/fpm/tree/master

```c
// === the fixed point macros ========================================
typedef int32_t fixpt ;
#define multfixpt(a,b) ((fixpt)((((int64_t)(a))*((int64_t)(b)))>>15))
// maybe convert to constexpr
// constexpr fixpt multfixpt(fixpt a, fixpt b) { return (fixpt)(((int64_t)a*(int64_t)b)>>15); }
#define float2fixpt(a) ((fixpt)((a)*32768.0))
#define fixpt2float(a) ((float)(a)/32768.0)
#define absfixpt(a) abs(a)
#define int2fixpt(a) ((fixpt)(a << 15))
#define fixpt2int(a) ((int32_t)(a >> 15))
#define char2fixpt(a) (fixpt)(((fixpt)(a)) << 15)
#define divfix(a,b) (fixpt)( (((int64_t)(a)) << 15) / (b))
```