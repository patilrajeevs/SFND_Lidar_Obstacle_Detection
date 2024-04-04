#include <cmath>
#include <vector>
#include <Eigen/Dense>

class Plane{
    // Line with equation of type : 
    // Ax + By + Cz + D = 0;
    float A, B, C, D, denominator;
public:
    // Constructor to create line from two points.
    // Line equation : (y2 - x1)X + (x2 - y)
    Plane(const float & x1, const float & y1, const float & z1, 
         const float & x2, const float & y2, const float & z2,
         const float & x3, const float & y3, const float & z3){

            Eigen::Vector3f v_12, v_13, v;
            v_12 << x2-x1, y2-y1, z2-z1;
            v_13 << x3-x1, y3-y1, z3-z1;
            v = v_12.cross(v_13);

            A = v[0];
            B = v[1];
            C = v[2];
            D = - (A*x1 + B*y1 + C*z1);

            denominator = sqrt(A*A + B*B + C*C);
    }

    // Calculate distance between this->line and point
    float DistanceToPoint(const float & x, const float & y, const float & z){
        return fabs(A*x + B*y + C*z + D) / denominator;
    }

};