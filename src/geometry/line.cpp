#include <cmath>

class Line{
    // Line with equation of type : 
    // Ax + By + C = 0;
    float A, B, C;
public:
    // Constructor to create line from two points.
    // Line equation : (y2 - x1)X + (x2 - y)
    Line(const float & x1,const float & y1, const float & x2, const float & y2){
        A = y1 - y2;
        B = x2 - x1;
        C = x1*y2 - x2*y1;
    }

    // Calculate distance between this->line and point
    float DistanceToPoint(const float & x, const float & y){
        return fabs(A*x + B*y + C) / sqrt(x*x + y*y);
    }

    // TODO : 
    // SLOPE, INTERCEPT FOR OF LINE 

};
