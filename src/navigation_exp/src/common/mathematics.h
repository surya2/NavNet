#pragma once

#include <cmath>

#define M_PI 3.14159265358979323846

struct Vertex  // multipurpose for both 3D point and vector def
{
    float x, y, z;
    Vertex() : x(-1), y(-1), z(-1) {}
    Vertex(float x, float y, float z) : x(x), y(y), z(z) {}

    bool operator==(const Vertex &other) const
    {
        return std::fabs(x - other.x) <= 0.5 && std::fabs(y - other.y) <= 0.5 && std::fabs(z - other.z) <= 0.5;
    }

    Vertex operator-(const Vertex &other) const
    {
        return Vertex(x - other.x, y - other.y, z - other.z);
    }
    Vertex operator+(const Vertex &other) const
    {
        return Vertex(x + other.x, y + other.y, z + other.z);
    }
    Vertex operator*(float scalar) const
    {
        return Vertex(x * scalar, y * scalar, z * scalar);
    }
    Vertex operator/(float scalar) const
    {
        return Vertex(x / scalar, y / scalar, z / scalar);
    }
    Vertex &operator+=(const Vertex &other)
    {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }
    Vertex &operator-=(const Vertex &other)
    {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }
    float magnitude() const
    {
        return std::sqrt(x * x + y * y + z * z);
    }
    float dot(const Vertex &other) const
    {
        return x * other.x + y * other.y + z * other.z;
    }
    Vertex cross(const Vertex &other) const
    {
        return Vertex(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
    }
    float operator^(const Vertex &other) const // angle between this vector and other vector in degrees
    {
        return std::acos(this->dot(other)/(this->magnitude() * other.magnitude())) * (180.0f / M_PI);
    }
    Vertex operator~(){ // get unit vector
        float mag = this->magnitude();
        if (mag == 0) return Vertex(0, 0, 0);
        return Vertex(x/mag, y/mag, z/mag);

    }
};

template <uint8_t W, uint8_t H>
struct Matrix {
    
    float data[W * H] = {};

    Matrix() = default;

    Matrix(float *src) {
        assign<W, H>(src);
    }

    Matrix(std::initializer_list<float> list)
    {
        uint8_t i = 0;
        for (float v : list)
            if (i < W * H) data[i++] = v;
    }

    float &operator()(uint8_t x, uint8_t y)
    {
        return data[y * W + x];
    }

    const float &operator()(uint8_t x, uint8_t y) const
    {
        return data[y * W + x];
    }

    template <uint8_t W2, uint8_t H2>
    void assign(float *src)
    {
        static_assert(W == W2 && H == H2, "Matrix assign: dimensions must be identical");
        for (int i = 0; i < W * H; i++){
            data[i] = src[i];
        }
    }

    template <uint8_t W2, uint8_t H2>
    Matrix<W, H> operator+(const Matrix<W2, H2> &o) const
    {
        static_assert(W == W2 && H == H2, "Matrix +: dimensions must be identical");
        float out[W * H];
        for (int i = 0; i < W * H; i++){
            out[i] = data[i] + o.data[i];
        }
        return Matrix<W, H>(out);
    }

    template <uint8_t W2, uint8_t H2>
    Matrix<W, H> operator-(const Matrix<W2, H2> &o) const
    {
        static_assert(W == W2 && H == H2, "Matrix -: dimensions must be identical");
        float out[W * H];
        for (int i = 0; i < W * H; i++){
            out[i] = data[i] - o.data[i];
        }
        return Matrix<W, H>(out);
    }

    template <uint8_t W2, uint8_t H2>
    Matrix<W, H> operator/(const Matrix<W2, H2> &o) const
    {
        static_assert(W == W2 && H == H2, "Matrix /: dimensions must be identical");
        float out[W * H];
        for (int i = 0; i < W * H; i++){
            out[i] = data[i] / o.data[i];
        }
        return Matrix<W, H>(out);
    }

    template <uint8_t W2, uint8_t H2>
    Matrix<W2, H> operator*(const Matrix<W2, H2> &o) const
    {
        static_assert(W == H2, "Matrix *: columns of left (W) must equal rows of right (H2)");
        float out[W2 * H];
        for (int y = 0; y < H; y++){
            for (int x = 0; x < W2; x++)
            {
                float sum = 0.0f;
                for (int k = 0; k < W; k++){
                    sum += (*this)(k, y) * o(x, k);
                }
                out[y * W2 + x] = sum;
            }
        }
        return Matrix<W2, H>(out);
    }

    Vertex operator+(const Vertex &v) const
    {
        static_assert(W == 1 && H == 3, "Matrix + Vertex: matrix must be 1 wide x 3 tall");
        return Vertex(data[0] + v.x, data[1] + v.y, data[2] + v.z);
    }
    Vertex operator-(const Vertex &v) const
    {
        static_assert(W == 1 && H == 3, "Matrix - Vertex: matrix must be 1 wide x 3 tall");
        return Vertex(data[0] - v.x, data[1] - v.y, data[2] - v.z);
    }
    Vertex operator/(const Vertex &v) const   // element-wise
    {
        static_assert(W == 1 && H == 3, "Matrix / Vertex: matrix must be 1 wide x 3 tall");
        return Vertex(data[0] / v.x, data[1] / v.y, data[2] / v.z);
    }

    auto operator*(const Vertex &v) const
    {
        static_assert(W == 3, "Matrix * Vertex: matrix must be 3 wide (a Vertex is 3x1)");
        const float in[3] = {v.x, v.y, v.z};
        float out[H];
        for (int y = 0; y < H; y++){
            out[y] = (*this)(0, y) * in[0] + (*this)(1, y) * in[1] + (*this)(2, y) * in[2];
        }
        if constexpr (H == 3){
            return Vertex(out[0], out[1], out[2]);
        }
        else return Matrix<1, H>(out);
    }

    Matrix<W, H> operator*(float s) const
    {
        float out[W * H];
        for (int i = 0; i < W * H; i++){
            out[i] = data[i] * s;
        }
        return Matrix<W, H>(out);
    }
    Matrix<W, H> operator/(float s) const
    {
        float out[W * H];
        for (int i = 0; i < W * H; i++){
            out[i] = data[i] / s;
        }
        return Matrix<W, H>(out);
    }

    Matrix<H, W> transposed() const
    {
        float out[W * H];
        for (int y = 0; y < H; y++){
            for (int x = 0; x < W; x++){
                out[x * H + y] = (*this)(x, y);
            }
        }
        return Matrix<H, W>(out);
    }
};

template <uint8_t W, uint8_t H>
Matrix<W, H> get_identity() {
    Matrix<W, H> identity;
    for (uint8_t i = 0; i < std::min(W, H); ++i) {
        identity(i, i) = 1.0f;
    }
    return identity;
}

inline Matrix<1, 3> to_matrix(const Vertex &v)   // column vector
{
    float d[3] = {v.x, v.y, v.z};
    return Matrix<1, 3>(d);
}
inline Matrix<3, 1> transpose(const Vertex &v)   // row vector
{
    float d[3] = {v.x, v.y, v.z};
    return Matrix<3, 1>(d);
}
inline Vertex to_vertex(const Matrix<1, 3> &m)
{
    return Vertex(m.data[0], m.data[1], m.data[2]);
}
template <uint8_t W, uint8_t H>
Matrix<W, H> operator*(float s, const Matrix<W, H> &m)
{
    return m * s;
}

struct Motion {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;

    Motion() : accel_x(0), accel_y(0), accel_z(0), gyro_x(0), gyro_y(0), gyro_z(0) {}

    Vertex get_gravity_vector() const {
        float mag = std::sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
        if (mag == 0) return Vertex(0, 0, 0);
        return Vertex(accel_x / mag, accel_y / mag, accel_z / mag);
    }

    float get_pitch_abt_x_axis() const { // in degrees
        return std::atan2(-accel_x, std::sqrt(accel_y * accel_y + accel_z * accel_z)) * (180.0f / M_PI);
    }

    float get_roll_abt_y_axis() const { // in degrees
        return std::atan2(accel_y, accel_z) * (180.0f / M_PI);
    }
};

inline float rad_to_deg(float rad){
    return (rad*180)/M_PI;
}
inline float deg_to_rad(float deg){
    return (deg*M_PI)/180;
}