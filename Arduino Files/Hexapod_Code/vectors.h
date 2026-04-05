class Vector2 {
public:
  float x;
  float y;
  // constructor
  Vector2() {
    x = 0;
    y = 0;
  }

  Vector2(float newX, float newY) {
    x = newX;
    y = newY;
  }

  String toString() {
    String xs = String(x);
    String ys = String(y);

    String ret = "(" + xs + "," + ys + ")";
    return ret;
  }

  float magnitude() const {
      return sqrt(x * x + y * y);
  }

  // Function to normalize the vector
  void normalize() {
      float mag = magnitude();
      if (mag > 0) {
          x /= mag;
          y /= mag;
      }
  }

  Vector2 operator+(Vector2 val) {
    // Implement the multiply operator
    Vector2 result;
    result.x = x + val.x;
    result.y = y + val.y;
    return result;
  }

  Vector2 operator*(float val) {
    // Implement the multiply operator
    Vector2 result;
    result.x = x * val;
    result.y = y * val;
    return result;
  }

  Vector2 operator*(Vector2 val) {
    // Implement the multiply operator
    Vector2 result;
    result.x = x * val.x;
    result.y = y * val.y;
    return result;
  }

  Vector2 bodyToLeg(float angle)
  {
    float angleRad = angle * (PI / 180);

    float xleg = x * cos(angleRad) + y * sin(angleRad);
    float yleg = -1 * x * sin(angleRad) + y * cos(angleRad);

    return Vector2 (abs(xleg), yleg);

  }
};

class Vector3 {
public:
  float x;
  float y;
  float z;
  // constructor
  Vector3() {
    x = 0;
    y = 0;
    z = 0;
  }

  Vector3(float newX, float newY, float newZ) {
    x = newX;
    y = newY;
    z = newZ;
  }

  bool operator!=(Vector3 val) {
    return (x != val.x || y != val.y || z != val.z);
  }

  bool operator==(Vector3 val) {
    return (x == val.x && y == val.y && z == val.z);
  }

  Vector3 operator*(float val) {
    return Vector3(x*val, y*val, z*val);
  }

  Vector3 operator*(Vector3 val) {
    return Vector3(x*val.x, y*val.y, z*val.z);
  }

  Vector3 operator/(Vector3 val) {
    return Vector3(x/val.x, y/val.y, z/val.z);
  }

  Vector3 operator/(float val) {
    return Vector3(x/val, y/val, z/val);
  }

  Vector3 operator+(Vector3 val) {
    return Vector3(x+val.x, y+val.y, z+val.z);
  }

  Vector3 operator-(Vector3 val) {
    return Vector3(x-val.x, y-val.y, z-val.z);
  }

  String toString() {
    String xs = String(x);
    String ys = String(y);
    String zs = String(z);

    String ret = "(" + xs + "," + ys + "," + zs + ")";
    return ret;
  }
  
  Vector3 bodyToLeg(float angle)
  {
    float a = angle * PI / 180.0f;

    float xleg = x * cos(a) - y * sin(a);
    float yleg = x * sin(a) + y * cos(a);

    return Vector3 (abs(xleg), yleg, z);

  }

  float distanceTo(Vector3 v){
    double dx = v.x - x;
    double dy = v.y - y;
    double dz = v.z - z;
    return sqrt(dx*dx + dy*dy + dz*dz);
  }
};


