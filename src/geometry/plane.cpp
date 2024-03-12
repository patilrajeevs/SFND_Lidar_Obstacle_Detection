#include <cmath>
#include <vector>

class Plane{
    // Line with equation of type : 
    // Ax + By + Cz + D = 0;
    float A, B, C, D;
public:
    // Constructor to create line from two points.
    // Line equation : (y2 - x1)X + (x2 - y)
    Plane(const float & x1, const float & y1, const float & z1, 
         const float & x2, const float & y2, const float & z2,
         const float & x3, const float & y3, const float & z3){

            std::vector <float> v_12 { x2-x1, y2-y1, z2-z1};
            std::vector <float> v_13 { x3-x1, y3-y1, z3-z1};
              //y2-y1 * z3-z1   - z2-z1  * y3-y1
            A = v_12[1]*v_13[2] - v_12[2]*v_13[1];
              //z2-z1 * x3-x1   - x2-x1  * z3-z1
            B = v_12[2]*v_13[0] - v_12[0]*v_13[2];
              //x2-x1 * y3-y1   - y2-y1  * x3-x1
            C = v_12[0]*v_13[1] - v_12[1]*v_13[0];

            D = -1 *(A*x1 + B*y1 + C*z1);
    }

    // Calculate distance between this->line and point
    float DistanceToPoint(const float & x, const float & y, const float & z){
        return fabs(A*x + B*y + C*z + D) / sqrt(x*x + y*y + z*z);
    }

};