#ifndef _MOTION_CONTROL_BASE_TYPE_H
#define _MOTION_CONTROL_BASE_TYPE_H

#include <stddef.h>
#include <assert.h>

#define     NUM_OF_JOINT    9
#define     PI              3.1415926535897932384626433832795
#define     MINIMUM_E3      0.001
#define     MINIMUM_E6      0.000001
#define     MINIMUM_E9      0.000000001
#define     MINIMUM_E12     0.000000000001

#define MAX_PATH_SIZE   2048

typedef unsigned long long int ErrorCode;
typedef unsigned int Tick;
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

enum MotionFrame
{
	JOINT,
	BASE,
	WORLD,
	USER,
	TOOL,
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

/*
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
*/

}

#endif

