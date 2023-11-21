
https://github.com/PX4/PX4-Autopilot/tree/main

```cpp
// Rotation matrix can be easily constructed from acceleration and mag field vectors
// 'k' is Earth Z axis (Down) unit vector in body frame
Vector3f k = -_accel;
k.normalize();

// 'i' is Earth X axis (North) unit vector in body frame, orthogonal with 'k'
Vector3f i = (_mag - k * (_mag * k));
i.normalize();

// 'j' is Earth Y axis (East) unit vector in body frame, orthogonal with 'k' and 'i'
Vector3f j = k % i;

// Fill rotation matrix
Dcmf R;
R.row(0) = i;
R.row(1) = j;
R.row(2) = k;

// Convert to quaternion
_q = R;

// Compensate for magnetic declination
Quatf decl_rotation = Eulerf(0.0f, 0.0f, _mag_decl);
_q = _q * decl_rotation;

_q.normalize();
```