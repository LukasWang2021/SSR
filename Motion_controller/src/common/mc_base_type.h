#ifndef MC_BASE_TYPE_H
#define MC_BASE_TYPE_H

#include <assert.h>

#define                 PI      3.1415926535897932384626433832795
#define       NUM_OF_JOINT      9

typedef double  MotionTime;

namespace fst_mc
{
	
enum MotionType
{
    MOTION_NONE   = 0,
    MOTION_JOINT  = 1,
    MOTION_LINE   = 2,
    MOTION_CIRCLE = 3,
};

class Joint
{
  public:
    double   j1;
    double   j2;
    double   j3;
    double   j4;
    double   j5;
    double   j6;
    double   j7;
    double   j8;
    double   j9;

    double& operator[](size_t index) {assert(index < NUM_OF_JOINT); return *(&j1 + index);}
    const double& operator[](size_t index) const {assert(index < NUM_OF_JOINT); return *(&j1 + index);}
};

class Point
{
	public:
    double x;
    double y;
    double z;
    
    double& operator[](size_t index) {assert(index < 3); return *(&x + index);}
    const double& operator[](size_t index) const {assert(index < 3); return *(&x + index);}
};

class Quaternion
{
	public:
    double x;
    double y;
    double z;
    double w;
    
    double& operator[](size_t index) {assert(index < 4); return *(&x + index);}
    const double& operator[](size_t index) const {assert(index < 4); return *(&x + index);}
};

class Euler
{
	public:
    double a;
    double b;
    double c;
    
    double& operator[](size_t index) {assert(index < 3); return *(&a + index);}
    const double& operator[](size_t index) const {assert(index < 3); return *(&a + index);}
};

class Pose
{
	public:
    Point       position;
    Quaternion  orientation;
    
    double& operator[](size_t index) {assert(index < 7); return *(&position.x + index);}
    const double& operator[](size_t index) const {assert(index < 7); return *(&position.x + index);}
};

class PoseEuler
{
	public:
    Point position;
    Euler orientation;
    
    double& operator[](size_t index) {assert(index < 6); return *(&position.x + index);}
    const double& operator[](size_t index) const {assert(index < 6); return *(&position.x + index);}
};

// total size of TrajectoryItem is 16 * 8 = 128 bytes
class TrajectoryItem
{
  public:
	double c0;
	double c1;
	double c2;
	double c3;
	double c4;
	double c5;
	
	double torque;
	double inertia;
	double gravity;
	double reserve[7];
	
	double& operator[](size_t index) {assert(index < 6); return *(&c0 + index);}
    const double& operator[](size_t index) const {assert(index < 6); return *(&c0 + index);}
};

// total size of TrajectorySegment is 128 * 9 + 8 = 1160 bytes
struct TrajectorySegment
{
	MotionTime      duration;
	TrajectoryItem  coeff[NUM_OF_JOINT];
};


}

#endif

