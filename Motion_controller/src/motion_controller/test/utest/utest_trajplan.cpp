/*************************************************************************
	> File Name: utest_trajplan.cpp
	> Author: 
	> Mail: 
	> Created Time: 2017年09月19日 星期二 15时15分53秒
 ************************************************************************/

#include<iostream>

#include<gtest/gtest.h>
#include<trajplan/TrajPlan.h>

#ifndef NUM_OF_AXIS
#define NUM_OF_AXIS   6
#endif

using namespace fst_controller;


class TransformTest : public ::testing::Test
{
    protected:
    
    virtual void SetUp() {
        planner_ = new TrajPlan();
    }
    
    virtual void TearDown() {
        delete planner_;
    }

    double ComputeMatrixVariance(double (&x)[4][4], double (&y)[4][4])
    {
        double error = 0.0;

        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                error += pow(x[i][j] - y[i][j], 2);

        error = sqrt(error);

        if (error > 0.000001) {
            cout << "error > 0.000001, error=" << error << endl;
            cout << "Matrix_1:" << endl;

            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    cout << x[i][j] << ",  ";
                }
                cout << endl;
            }

            cout << "Matrix_2:" << endl;

            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    cout << y[i][j] << ",  ";
                }
                cout << endl;
            }
        }
        
        return error;
    }

    TrajPlan *planner_;
};

TEST_F(TransformTest, QuaternionAndEuler)
{
    Pose        pose, pose_result;
    PoseEuler   pose_e, pose_t;

    double mat1[4][4] = {0.0};
    double mat2[4][4] = {0.0};

    pose_e.position.x = (rand() - RAND_MAX / 2) / (0.0 + RAND_MAX / 2) * 400;
    pose_e.position.y = (rand() - RAND_MAX / 2) / (0.0 + RAND_MAX / 2) * 400;
    pose_e.position.z = (rand() - RAND_MAX / 2) / (0.0 + RAND_MAX / 2) * 400;
    pose_e.orientation.a = -0.7;
    pose_e.orientation.b = 0.5;
    pose_e.orientation.c = 0.8;

    ASSERT_EQ(0, planner_->Euler2Quatern(pose_e, pose));
    ASSERT_EQ(0, planner_->Quatern2Euler(pose, pose_t));
    ASSERT_EQ(0, planner_->PoseEuler2Matrix(pose_e, mat1));
    ASSERT_EQ(0, planner_->PoseEuler2Matrix(pose_t, mat2));
    ASSERT_GT(0.000001, ComputeMatrixVariance(mat1, mat2));

    pose_e.position.x = (rand() - RAND_MAX / 2) / (0.0 + RAND_MAX / 2) * 400;
    pose_e.position.y = (rand() - RAND_MAX / 2) / (0.0 + RAND_MAX / 2) * 400;
    pose_e.position.z = (rand() - RAND_MAX / 2) / (0.0 + RAND_MAX / 2) * 400;
    pose_e.orientation.a = 1.3;
    pose_e.orientation.b = 2.5;
    pose_e.orientation.c = -3.1;

    ASSERT_EQ(0, planner_->Euler2Quatern(pose_e, pose));
    ASSERT_EQ(0, planner_->Quatern2Euler(pose, pose_t));
    ASSERT_EQ(0, planner_->PoseEuler2Matrix(pose_e, mat1));
    ASSERT_EQ(0, planner_->PoseEuler2Matrix(pose_t, mat2));
    ASSERT_GT(0.000001, ComputeMatrixVariance(mat1, mat2));

    pose_e.position.x = (rand() - RAND_MAX / 2) / (0.0 + RAND_MAX / 2) * 400;
    pose_e.position.y = (rand() - RAND_MAX / 2) / (0.0 + RAND_MAX / 2) * 400;
    pose_e.position.z = (rand() - RAND_MAX / 2) / (0.0 + RAND_MAX / 2) * 400;
    pose_e.orientation.a = 0.7;
    pose_e.orientation.b = 5.5;
    pose_e.orientation.c = 0.0;

    ASSERT_EQ(0, planner_->Euler2Quatern(pose_e, pose));
    ASSERT_EQ(0, planner_->PoseEuler2Matrix(pose_e, mat1));
    ASSERT_EQ(0, planner_->PoseQuatern2Matrix(pose, mat2));
    ASSERT_GT(0.000001, ComputeMatrixVariance(mat1, mat2));

    pose.position.x = (rand() - RAND_MAX / 2) / (0.0 + RAND_MAX / 2) * 400;
    pose.position.y = (rand() - RAND_MAX / 2) / (0.0 + RAND_MAX / 2) * 400;
    pose.position.z = (rand() - RAND_MAX / 2) / (0.0 + RAND_MAX / 2) * 400;
    pose.orientation.x = 0.0;
    pose.orientation.y = 1.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;
    
    ASSERT_EQ(0, planner_->Quatern2Euler(pose, pose_e));
    ASSERT_EQ(0, planner_->Euler2Quatern(pose_e, pose_result));
    ASSERT_EQ(0, planner_->PoseQuatern2Matrix(pose, mat1));
    ASSERT_EQ(0, planner_->PoseQuatern2Matrix(pose_result, mat2));
    ASSERT_GT(0.000001, ComputeMatrixVariance(mat1, mat2));

    pose.position.x = (rand() - RAND_MAX / 2) / (0.0 + RAND_MAX / 2) * 400;
    pose.position.y = (rand() - RAND_MAX / 2) / (0.0 + RAND_MAX / 2) * 400;
    pose.position.z = (rand() - RAND_MAX / 2) / (0.0 + RAND_MAX / 2) * 400;
    pose.orientation.x = 0.2;
    pose.orientation.y = 0.5;
    pose.orientation.z = 0.7;
    pose.orientation.w = 0.5;

    ASSERT_EQ(2001, planner_->Quatern2Euler(pose, pose_e));
}

TEST_F(TransformTest, CarpetTest)
{
    double a, b, c;
    double step = 0.1 * PI;

    Pose        pose;
    PoseEuler   pose_e;
    PoseEuler   pose_t;

    double error = 0.0;
    double mat1[4][4] = {0.0};
    double mat2[4][4] = {0.0};

    srand((unsigned)time(NULL));

    pose_e.position.x = (rand() - RAND_MAX / 2) / (0.0 + RAND_MAX / 2) * 400;
    pose_e.position.y = (rand() - RAND_MAX / 2) / (0.0 + RAND_MAX / 2) * 400;
    pose_e.position.z = (rand() - RAND_MAX / 2) / (0.0 + RAND_MAX / 2) * 400;

    for (a = -PI; a < PI; a += step) {
        for (b = -PI; b < PI; b += step) {
            for (c = -PI; c < PI; c += step) {
                error = 0.0;

                pose_e.orientation.a = a;
                pose_e.orientation.b = b;
                pose_e.orientation.c = c;

                ASSERT_EQ(0, planner_->Euler2Quatern(pose_e, pose));
                ASSERT_EQ(0, planner_->Quatern2Euler(pose, pose_t));
                ASSERT_EQ(0, planner_->PoseEuler2Matrix(pose_e, mat1));
                ASSERT_EQ(0, planner_->PoseEuler2Matrix(pose_t, mat2));
                ASSERT_GT(0.000001, ComputeMatrixVariance(mat1, mat2));
            }
        }
    }   
}

class IKandFKTest : public ::testing::Test
{
    protected:
    
    virtual void SetUp() {
        planner_ = new TrajPlan();
    }
    
    virtual void TearDown() {
        delete planner_;
    }

    double ComputeJointVariance(const Joint &j1, const Joint &j2)
    {
        double error = 0.0;

        for (int i = 0; i < NUM_OF_AXIS; ++i) {
            error += pow(*((double*)&j1 + i) - *((double*)&j2 + i), 2);
        }

        error = sqrt(error);
        
        if (error > 0.00001) {
            printf("\n");
            printf("joint1 = %1.6f, %1.6f, %1.6f, %1.6f, %1.6f, %1.6f\n",
                    j1.j1, j1.j2, j1.j3, j1.j4, j1.j5, j1.j6);
            printf("joint2 = %1.6f, %1.6f, %1.6f, %1.6f, %1.6f, %1.6f\n",
                    j2.j1, j2.j2, j2.j3, j2.j4, j2.j5, j2.j6);
        }

        return error;
    }

    double ComputePoseVariance(const Pose &p1, const Pose &p2)
    {
        double error = 0.0;

        for (int i = 0; i < sizeof(Pose) / sizeof(double); ++i) {
            error += pow(*((double*)&p1 + i) - *((double*)&p2 + i), 2);
        }

        error = sqrt(error);

        if (error > 0.00001) {
            printf("\npose = (x,y,z),(x,y,z,w)}\n");
            printf("pose1 = %3.6f, %3.6f, %3.6f, %1.6f, %1.6f, %1.6f, %1.6f\n",
                   p1.position.x, p1.position.y, p1.position.z,
                   p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w);
            printf("pose2 = %3.6f, %3.6f, %3.6f, %1.6f, %1.6f, %1.6f, %1.6f\n",
                   p2.position.x, p2.position.y, p2.position.z,
                   p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w);
        }

        return error;
    }

    TrajPlan *planner_;
};


TEST_F(IKandFKTest, IKAndFK)
{
    Joint       jnt, ref, res;
    PoseEuler   pose;

    double tmp1[] = {2.96, 1.3, 3.22, 3.31, 1.84, 6.28};
    double tmp2[] = {-2.96,-2.26,-1.16,-3.31,-1.84,-6.28};
    double tmp3[] = {-3.96,2.5,-1.6,3.4,1.95,0};

    memcpy(&jnt, tmp1, sizeof(jnt));
    memcpy(&ref, tmp1, sizeof(jnt));
    ASSERT_EQ(0, planner_->ForwardKinematics(jnt, pose));
    ASSERT_EQ(0, planner_->InverseKinematics(pose, ref, res));
    ASSERT_GT(0.000001, ComputeJointVariance(jnt, res));

    memcpy(&jnt, tmp2, sizeof(jnt));
    memcpy(&ref, tmp2, sizeof(jnt));
    ASSERT_EQ(0, planner_->ForwardKinematics(jnt, pose));
    ASSERT_EQ(0, planner_->InverseKinematics(pose, ref, res));
    ASSERT_GT(0.000001, ComputeJointVariance(jnt, res));

    memcpy(&jnt, tmp3, sizeof(jnt));
    memcpy(&ref, tmp3, sizeof(jnt));
    ASSERT_EQ(0, planner_->ForwardKinematics(jnt, pose));
    ASSERT_EQ(1002, planner_->InverseKinematics(pose, ref, res));
}

TEST_F(IKandFKTest, CarpetTest)
{
    Joint       jnt, res;
    PoseEuler   pose;
    int         cnt = 0;
    
    double tmp_floor[] = {-2.6, -2.0, -1.0, -3.0, -1.5, -3.0};
    double tmp_ceil[]  = {2.6, 1.2, 3.0, 3.0, 1.5, 3.0};

    for (jnt.j1 = tmp_floor[0]; jnt.j1 < tmp_ceil[0]; jnt.j1 += 0.3) {
        for (jnt.j2 = tmp_floor[1]; jnt.j2 < tmp_ceil[1]; jnt.j2 += 0.2) {
            for (jnt.j3 = tmp_floor[2]; jnt.j3 < tmp_ceil[2]; jnt.j3 += 0.2) {
                for (jnt.j4 = tmp_floor[3]; jnt.j4 < tmp_ceil[3]; jnt.j4 += 0.4) {
                    for (jnt.j5 = tmp_floor[4]; jnt.j5 < tmp_ceil[4]; jnt.j5 += 0.2) {
                        for (jnt.j6 = tmp_floor[5]; jnt.j6 < tmp_ceil[5]; jnt.j6 += 0.4) {
                            printf("\r%1.2f\t%1.2f\t%1.2f\t%1.2f\t%1.2f\t%1.2f      ",
                                  jnt.j1, jnt.j2, jnt.j3, jnt.j4, jnt.j5, jnt.j6);

                            ASSERT_EQ(0, planner_->ForwardKinematics(jnt, pose));
                            ASSERT_EQ(0, planner_->InverseKinematics(pose, jnt, res));
                            ASSERT_GT(0.00001, ComputeJointVariance(jnt, res));

                            cnt++;
                        }
                    }
                }
            }
        }
    }

    printf("\ntotal test cycle cnt: %d\n", cnt);
}

class PlanningTest : public ::testing::Test
{
    protected:
    
    virtual void SetUp() {
        motion1 = NULL;
        motion2 = NULL;
        motion3 = NULL;
        motion4 = NULL;
        
        planner_ = new TrajPlan();
    }
    
    virtual void TearDown() {
        printf("dddd\n");
        if (planner_)   delete planner_;
        if (motion1)    delete motion1;
        if (motion2)    delete motion2;
        if (motion3)    delete motion3;
        if (motion4)    delete motion4;
        printf("d23213ddd\n");
    }

    TrajPlan    *planner_;
    MoveCommand *motion1, *motion2, *motion3, *motion4;
};

TEST_F(PlanningTest, MoveJ)
{
    double tmp1[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double tmp2[] = {0.0, 0.0, 0.0, 0.0, -1.57, 0.0};
    double tmp3[] = {-2.6, -2.0, -1.0, -3.0, -1.5, -3.0};
    double tmp4[] = {2.6, 1.2, 3.0, 3.0, 1.5, 3.0};
    double tmp5[] = {3.5, 2.7, 1.4, 5.8, -4.6, 2.1};
    double tmp6[] = {0.0, 1.0, -1.0, 2.1, 1.57, 0.3};
    double tmp7[] = {0.8, 0.2, 0.3, -0.5, 0.7, -1.0};
    double tmp8[] = {-3.96, 2.5, -1.6, 3.4, 1.95, 0.0};
    
    double tmp11[] = {400.0, 400.0, 400.0, 0.0, 0.0, 3.1416};
    double tmp12[] = {400.0, -400.0, 500.0, 0.0, 0.0, 3.1416};
    double tmp13[] = {380.0, 0.0, 617.5, 0.0, 0.0, 3.1416};
    double tmp14[] = {400.0, 240.0, 650.0, 0.0, 0.0, 3.1416};
    double tmp15[] = {400.0, 360.0, 350.0, 0.0, 0.0, 3.1416};
    double tmp16[] = {400.0, -140.0, 600.0, 0.0, 0.0, 3.1416};
    double tmp17[] = {400.0, -240.0, 400.0, 0.0, 0.0, 3.1416};
    double tmp18[] = {400.0, 100.0, 650.0, 0.0, 0.0, 3.1416};
    double tmp19[] = {400.0, 300.0, 400.0, 0.0, 0.0, 3.1416};


    MotionTarget    t1, t2, t3;
    Joint           joint;
    PoseEuler       pose;

    // (0.0, 0.0, 0.0, 0.0, 0.0, 0.0) -> (0.0, 0.0, 0.0, 0.0, -1.57, 0.0)
    memcpy(&joint, tmp2, sizeof(joint));

    t1.type             = MOTION_JOINT;
    t1.cnt              = 0;
    t1.percent_velocity = 100;
    t1.joint_target     = joint;
    t2.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 1);
    ASSERT_TRUE(NULL != motion1);

    motion1->prev = NULL;
    motion1->next = NULL;

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());

    // (0.0, 0.0, 0.0, 0.0, -1.57, 0.0) -> (-2.6, -2.0, -1.0, -3.0, -1.5, -3.0)
    memcpy(&joint, tmp3, sizeof(joint));

    t1.type             = MOTION_JOINT;
    t1.cnt              = 0;
    t1.percent_velocity = 70;
    t1.joint_target     = joint;
    t2.type             = MOTION_NONE;

    motion2 = new MoveCommand(planner_, t1, t2, 2);
    ASSERT_TRUE(NULL != motion2);

    motion2->prev = motion1;
    motion2->next = NULL;
    motion1->next = motion2;

    ASSERT_EQ(0, motion2->PlanTraj());

    // (-2.6, -2.0, -1.0, -3.0, -1.5, -3.0) -> (2.6, 1.2, 3.0, 3.0, 1.5, 3.0)
    memcpy(&joint, tmp4, sizeof(joint));

    t1.type             = MOTION_JOINT;
    t1.cnt              = 0;
    t1.percent_velocity = 40;
    t1.joint_target     = joint;
    t2.type             = MOTION_NONE;

    motion3 = new MoveCommand(planner_, t1, t2, 3);
    ASSERT_TRUE(NULL != motion3);

    motion3->prev = motion2;
    motion3->next = NULL;
    motion2->next = motion3;

    ASSERT_EQ(0, motion3->PlanTraj());

    // (2.6, 1.2, 3.0, 3.0, 1.5, 3.0) -> (3.5, 2.7, 1.4, 5.8, -4.6, 2.1)
    memcpy(&joint, tmp5, sizeof(joint));

    t1.type             = MOTION_JOINT;
    t1.cnt              = 0;
    t1.percent_velocity = 10;
    t1.joint_target     = joint;
    t2.type             = MOTION_NONE;

    motion4 = new MoveCommand(planner_, t1, t2, 4);
    ASSERT_TRUE(NULL != motion4);

    motion4->prev = motion3;
    motion4->next = NULL;
    motion3->next = motion4;

    ASSERT_EQ(0, motion4->PlanTraj());

    delete motion1; motion1 = NULL;
    delete motion2; motion2 = NULL;
    delete motion3; motion3 = NULL;
    delete motion4; motion4 = NULL;

    // (0.0,0.0,0.0,0.0,0.0,0.0) -> (0.0,0.0,0.0,0.0,-1.57,0.0) -> (-2.6,-2.0,-1.0,-3.0,-1.5,-3.0) -> (2.6,1.2,3.0,3.0,1.5,3.0)
    memcpy(&joint, tmp2, sizeof(joint));
    t1.type             = MOTION_JOINT;
    t1.cnt              = 100;
    t1.percent_velocity = 100;
    t1.joint_target     = joint;

    memcpy(&joint, tmp3, sizeof(joint));
    t2.type             = MOTION_JOINT;
    t2.cnt              = 100;
    t2.percent_velocity = 50;
    t2.joint_target     = joint;

    memcpy(&joint, tmp4, sizeof(joint));
    t3.type             = MOTION_JOINT;
    t3.cnt              = 100;
    t3.percent_velocity = 10;
    t3.joint_target     = joint;

    motion1 = new MoveCommand(planner_, t1, t2, 5);
    motion2 = new MoveCommand(planner_, t2, t3, 6);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);
    
    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = NULL;

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());

    delete motion1; motion1 = NULL;
    delete motion2; motion2 = NULL;

    // (0.0,0.0,0.0,0.0,0.0,0.0) -> (2.6,1.2,3.0,3.0,1.5,3.0) -> (-2.6,-2.0,-1.0,-3.0,-1.5,-3.0) -> (0.0,0.0,0.0,0.0,-1.57,0.0)
    memcpy(&joint, tmp4, sizeof(joint));
    t1.type             = MOTION_JOINT;
    t1.cnt              = 50;
    t1.percent_velocity = 100;
    t1.joint_target     = joint;

    memcpy(&joint, tmp3, sizeof(joint));
    t2.type             = MOTION_JOINT;
    t2.cnt              = 50;
    t2.percent_velocity = 50;
    t2.joint_target     = joint;

    memcpy(&joint, tmp2, sizeof(joint));
    t3.type             = MOTION_JOINT;
    t3.cnt              = 50;
    t3.percent_velocity = 10;
    t3.joint_target     = joint;

    motion1 = new MoveCommand(planner_, t1, t2, 7);
    motion2 = new MoveCommand(planner_, t2, t3, 8);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);
    
    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = NULL;

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());

    delete motion1; motion1 = NULL;
    delete motion2; motion2 = NULL;

    // (0.0,0.0,0.0,0.0,0.0,0.0) -> (0.0,1.0,-1,2.1,1.57,0.3) -> (0.8,0.2,0.3,-0.5,0.7,-1.0) -> (-3.96,2.5,-1.6,3.4,1.95,0.0)
    memcpy(&joint, tmp6, sizeof(joint));
    t1.type             = MOTION_JOINT;
    t1.cnt              = 10;
    t1.percent_velocity = 100;
    t1.joint_target     = joint;

    memcpy(&joint, tmp7, sizeof(joint));
    t2.type             = MOTION_JOINT;
    t2.cnt              = 10;
    t2.percent_velocity = 50;
    t2.joint_target     = joint;

    memcpy(&joint, tmp8, sizeof(joint));
    t3.type             = MOTION_JOINT;
    t3.cnt              = 10;
    t3.percent_velocity = 10;
    t3.joint_target     = joint;

    motion1 = new MoveCommand(planner_, t1, t2, 9);
    motion2 = new MoveCommand(planner_, t2, t3, 10);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);
    
    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = NULL;

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());

    delete motion1; motion1 = NULL;
    delete motion2; motion2 = NULL;

    // (0.0,0.0,0.0,0.0,0.0,0.0) -> (0.0,0.0,0.0,0.0,-1.57,0.0) -> [400.0,400.0,400.0,0.0,0.0,3.1416]
    memcpy(&joint, tmp2, sizeof(joint));
    t1.type             = MOTION_JOINT;
    t1.cnt              = 100;
    t1.percent_velocity = 100;
    t1.joint_target     = joint;

    memcpy(&pose, tmp11, sizeof(pose));
    t2.type             = MOTION_LINE;
    t2.cnt              = 0;
    t2.linear_velocity  = 500;
    t2.pose_target      = pose;

    t3.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 11);
    motion2 = new MoveCommand(planner_, t2, t3, 12);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = NULL;

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());

    delete motion1; motion1 = NULL;
    delete motion2; motion2 = NULL;

    // (0.0,0.0,0.0,0.0,0.0,0.0) -> (0.0,0.0,0.0,0.0,-1.57,0.0) -> [400.0,-400.0,500.0,0.0,0.0,3.1416]
    memcpy(&joint, tmp2, sizeof(joint));
    t1.type             = MOTION_JOINT;
    t1.cnt              = 100;
    t1.percent_velocity = 50;
    t1.joint_target     = joint;

    memcpy(&pose, tmp12, sizeof(pose));
    t2.type             = MOTION_LINE;
    t2.cnt              = 0;
    t2.linear_velocity  = 500;
    t2.pose_target      = pose;

    t3.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 11);
    motion2 = new MoveCommand(planner_, t2, t3, 12);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = NULL;

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());

    delete motion1; motion1 = NULL;
    delete motion2; motion2 = NULL;

    // (0.0,0.0,0.0,0.0,0.0,0.0) -> (0.0,0.0,0.0,0.0,-1.57,0.0) -> [380.0,0.0,617.5,0.0,0.0,3.1416]
    memcpy(&joint, tmp2, sizeof(joint));
    t1.type             = MOTION_JOINT;
    t1.cnt              = 10;
    t1.percent_velocity = 50;
    t1.joint_target     = joint;

    memcpy(&pose, tmp13, sizeof(pose));
    t2.type             = MOTION_LINE;
    t2.cnt              = 0;
    t2.linear_velocity  = 500;
    t2.pose_target      = pose;

    t3.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 11);
    motion2 = new MoveCommand(planner_, t2, t3, 12);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = NULL;

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());

    delete motion1; motion1 = NULL;
    delete motion2; motion2 = NULL;

    // (0.0,0.0,0.0,0.0,0.0,0.0) -> (0.0,0.0,0.0,0.0,-1.57,0.0) -> [400.0,240.0,650,0.0,0.0,3.1416]/[400.0,360.0,350.0,0.0,0.0,3.1416] 
    memcpy(&joint, tmp2, sizeof(joint));
    t1.type             = MOTION_JOINT;
    t1.cnt              = 100;
    t1.percent_velocity = 50;
    t1.joint_target     = joint;

    memcpy(&pose, tmp14, sizeof(pose));
    t2.type             = MOTION_CIRCLE;
    t2.cnt              = 0;
    t2.linear_velocity  = 500;
    t2.circle_target.pose1  = pose;
    memcpy(&pose, tmp15, sizeof(pose));
    t2.circle_target.pose2  = pose;

    t3.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 11);
    motion2 = new MoveCommand(planner_, t2, t3, 12);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = NULL;

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());

    delete motion1; motion1 = NULL;
    delete motion2; motion2 = NULL;

    // (0.0,0.0,0.0,0.0,0.0,0.0) -> (0.0,0.0,0.0,0.0,-1.57,0.0) -> [400.0,-240.0,600.0,0.0,0.0,3.1416]/[400.0,0.0,300.0,0.0,0.0,3.1416] 
    memcpy(&joint, tmp2, sizeof(joint));
    t1.type             = MOTION_JOINT;
    t1.cnt              = 50;
    t1.percent_velocity = 50;
    t1.joint_target     = joint;

    memcpy(&pose, tmp16, sizeof(pose));
    t2.type             = MOTION_CIRCLE;
    t2.cnt              = 0;
    t2.linear_velocity  = 500;
    t2.circle_target.pose1  = pose;
    memcpy(&pose, tmp17, sizeof(pose));
    t2.circle_target.pose2  = pose;

    t3.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 11);
    motion2 = new MoveCommand(planner_, t2, t3, 12);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = NULL;

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());

    delete motion1; motion1 = NULL;
    delete motion2; motion2 = NULL;

    // (0.0,0.0,0.0,0.0,0.0,0.0) -> (0.0,0.0,0.0,0.0,-1.57,0.0) -> [400.0,100.0,650.0,0.0,0.0,3.1416]/[400.0,300.0,400.0,0.0,0.0,3.1416] 
    memcpy(&joint, tmp2, sizeof(joint));
    t1.type             = MOTION_JOINT;
    t1.cnt              = 50;
    t1.percent_velocity = 50;
    t1.joint_target     = joint;

    memcpy(&pose, tmp18, sizeof(pose));
    t2.type             = MOTION_CIRCLE;
    t2.cnt              = 0;
    t2.linear_velocity  = 500;
    t2.circle_target.pose1  = pose;
    memcpy(&pose, tmp19, sizeof(pose));
    t2.circle_target.pose2  = pose;

    t3.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 11);
    motion2 = new MoveCommand(planner_, t2, t3, 12);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = NULL;

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());

    delete motion1; motion1 = NULL;
    delete motion2; motion2 = NULL;
}

TEST_F(PlanningTest, MoveL)
{
    double tmp1[] = {0.0, 0.0, 0.0, 0.0, -1.5708, 0.0};
    double tmp2[] = {0.2, -0.5, 0.0, 0.0, -1.0, 0.0};

    double tmp11[] = {400.0, 400.0, 400.0, 0.0, 0.0, 3.1416};
    double tmp12[] = {400.0, -400.0, 500.0, 0.0, 0.0, 3.1416};
    double tmp13[] = {380.0, 0.0, 617.5, 0.0, 0.0, 3.1416};
    double tmp14[] = {250.0, -250.0, 450.0, 0.2, 0.2, 2.8};
    double tmp15[] = {400.0, -240.0, 600.0, 0.0, 0.0, 3.1416};
    double tmp16[] = {400.0, 0.0, 300.0, 0.0, 0.0, 3.1416};
    double tmp17[] = {400.0, -240.0, 450.0, 0.0, 0.0, 3.1416};
    double tmp18[] = {400.0, -100.0, 700.0, 0.0, 0.0, 3.1416};
    double tmp19[] = {400.0, -500.0, 350.0, 0.0, 0.0, 3.01416};


    MotionTarget    t1, t2, t3;
    Joint           joint;
    PoseEuler       pose;


    // (0.0, 0.0, 0.0, 0.0, -1.5708, 0.0) -> [400.0, 400.0, 400.0, 0.0, 0.0, 3.1416]
    memcpy(&pose, tmp11, sizeof(pose));

    t1.type             = MOTION_LINE;
    t1.cnt              = 0;
    t1.linear_velocity  = 1000;
    t1.pose_target      = pose;

    t2.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 1);
    ASSERT_TRUE(NULL != motion1);

    motion1->prev = NULL;
    motion1->next = NULL;

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());
    
    // [400.0, 400.0, 400.0, 0.0, 0.0, 3.1416] -> [400.0,-400.0,500.0,0.0,0.0,3.1416]
    memcpy(&pose, tmp12, sizeof(pose));

    t1.type             = MOTION_LINE;
    t1.cnt              = 0;
    t1.linear_velocity  = 500;
    t1.pose_target      = pose;

    t2.type             = MOTION_NONE;

    motion2 = new MoveCommand(planner_, t1, t2, 2);
    ASSERT_TRUE(NULL != motion2);

    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = NULL;

    ASSERT_EQ(0, motion2->PlanTraj());

    // [400.0,-400.0,500.0,0.0,0.0,3.1416] -> [380.0,0.0,617.5,0.0,0.0,3.1416]
    memcpy(&pose, tmp13, sizeof(pose));

    t1.type             = MOTION_LINE;
    t1.cnt              = 0;
    t1.linear_velocity  = 100;
    t1.pose_target      = pose;

    t2.type             = MOTION_NONE;

    motion3 = new MoveCommand(planner_, t1, t2, 3);
    ASSERT_TRUE(NULL != motion3);

    motion2->next = motion3;
    motion3->prev = motion2;
    motion3->next = NULL;

    ASSERT_EQ(0, motion3->PlanTraj());

    delete motion1;     motion1 = NULL;
    delete motion2;     motion2 = NULL;
    delete motion3;     motion3 = NULL;

    // [380.0,0.0,617.5,0.0,0.0,3.1416] -> [400.0, 400.0, 400.0, 0.0, 0.0, 3.1416] -> (0.0, 0.0, 0.0, 0.0, -1.5708, 0.0)
    memcpy(&pose, tmp11, sizeof(pose));

    t1.type             = MOTION_LINE;
    t1.cnt              = 100;
    t1.linear_velocity  = 800;
    t1.pose_target      = pose;
    
    memcpy(&joint, tmp1, sizeof(joint));
    t2.type             = MOTION_JOINT;
    t2.cnt              = 0;
    t2.percent_velocity = 50;
    t2.joint_target     = joint;

    t3.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 4);
    motion2 = new MoveCommand(planner_, t2, t3, 5);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);

    motion1->setInitialJoint(joint);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = NULL;

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());
    
    delete motion1;     motion1 = NULL;
    delete motion2;     motion2 = NULL;

    // [380.0,0.0,617.5,0.0,0.0,3.1416] -> [400.0, -400.0, 500.0, 0.0, 0.0, 3.1416] -> (0.2, -0.5, 0.0, 0.0, -1.0, 0.0)
    memcpy(&pose, tmp12, sizeof(pose));

    t1.type             = MOTION_LINE;
    t1.cnt              = 50;
    t1.linear_velocity  = 800;
    t1.pose_target      = pose;
    
    memcpy(&joint, tmp2, sizeof(joint));
    t2.type             = MOTION_JOINT;
    t2.cnt              = 0;
    t2.percent_velocity = 50;
    t2.joint_target     = joint;

    t3.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 6);
    motion2 = new MoveCommand(planner_, t2, t3, 7);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = NULL;

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());
    
    delete motion1;     motion1 = NULL;
    delete motion2;     motion2 = NULL;

    // [380.0,0.0,617.5,0.0,0.0,3.1416] -> [250.0, -250.0, 450.0, 0.2, 0.2, 2.8] -> (0.0, 0.0, 0.0, 0.0, -1.5708, 0.0)
    memcpy(&pose, tmp14, sizeof(pose));

    t1.type             = MOTION_LINE;
    t1.cnt              = 100;
    t1.linear_velocity  = 800;
    t1.pose_target      = pose;
    
    memcpy(&joint, tmp1, sizeof(joint));
    t2.type             = MOTION_JOINT;
    t2.cnt              = 0;
    t2.percent_velocity = 50;
    t2.joint_target     = joint;

    t3.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 8);
    motion2 = new MoveCommand(planner_, t2, t3, 9);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);

    motion1->setInitialJoint(joint);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = NULL;

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());
    
    delete motion1;     motion1 = NULL;
    delete motion2;     motion2 = NULL;

    // [380.0,0.0,617.5,0.0,0.0,3.1416] -> [400.0, 400.0, 400.0, 0.0, 0.0, 3.1416] -> [400.0, -400.0, 500.0, 0.0, 0.0, 3.1416]
    memcpy(&pose, tmp11, sizeof(pose));

    t1.type             = MOTION_LINE;
    t1.cnt              = 100;
    t1.linear_velocity  = 800;
    t1.pose_target      = pose;
    
    memcpy(&pose, tmp12, sizeof(pose));

    t2.type             = MOTION_LINE;
    t2.cnt              = 0;
    t2.linear_velocity  = 800;
    t2.pose_target      = pose;

    t3.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 10);
    motion2 = new MoveCommand(planner_, t2, t3, 11);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = NULL;

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());
    
    delete motion1;     motion1 = NULL;
    delete motion2;     motion2 = NULL;

    // [380.0,0.0,617.5,0.0,0.0,3.1416] -> [400.0, -400.0, 500.0, 0.0, 0.0, 3.1416] -> [400.0, 400.0, 400.0, 0.0, 0.0, 3.1416]
    memcpy(&pose, tmp12, sizeof(pose));

    t1.type             = MOTION_LINE;
    t1.cnt              = 50;
    t1.linear_velocity  = 800;
    t1.pose_target      = pose;
    
    memcpy(&pose, tmp11, sizeof(pose));

    t2.type             = MOTION_LINE;
    t2.cnt              = 0;
    t2.linear_velocity  = 800;
    t2.pose_target      = pose;

    t3.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 12);
    motion2 = new MoveCommand(planner_, t2, t3, 13);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = NULL;

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());
    
    delete motion1;     motion1 = NULL;
    delete motion2;     motion2 = NULL;

    // [380.0,0.0,617.5,0.0,0.0,3.1416] -> [250.0, -250.0, 450.0, 0.2, 0.2, 2.8] -> [400.0, 400.0, 400.0, 0.0, 0.0, 3.1416]
    memcpy(&pose, tmp14, sizeof(pose));

    t1.type             = MOTION_LINE;
    t1.cnt              = 10;
    t1.linear_velocity  = 800;
    t1.pose_target      = pose;
    
    memcpy(&pose, tmp11, sizeof(pose));

    t2.type             = MOTION_LINE;
    t2.cnt              = 0;
    t2.linear_velocity  = 800;
    t2.pose_target      = pose;

    t3.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 14);
    motion2 = new MoveCommand(planner_, t2, t3, 15);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = NULL;

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());
    
    delete motion1;     motion1 = NULL;
    delete motion2;     motion2 = NULL;

    // [380.0,0.0,617.5,0.0,0.0,3.1416] -> [400.0, -400.0, 500.0, 0.0, 0.0, 3.1416] -> [400.0, -240.0, 600.0, 0.0, 0.0, 3.1416]/[400.0, 0.0, 300.0, 0.0, 0.0, 3.1416]
    memcpy(&pose, tmp12, sizeof(pose));

    t1.type             = MOTION_LINE;
    t1.cnt              = 100;
    t1.linear_velocity  = 500;
    t1.pose_target      = pose;
    
    memcpy(&pose, tmp15, sizeof(pose));

    t2.type             = MOTION_CIRCLE;
    t2.cnt              = 0;
    t2.linear_velocity  = 500;
    t2.circle_target.pose1  = pose;

    memcpy(&pose, tmp16, sizeof(pose));
    t2.circle_target.pose2  = pose;

    t3.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 16);
    motion2 = new MoveCommand(planner_, t2, t3, 17);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = NULL;

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());
    
    delete motion1;     motion1 = NULL;
    delete motion2;     motion2 = NULL;

    // [380.0,0.0,617.5,0.0,0.0,3.1416] -> [400.0, -400.0, 500.0, 0.0, 0.0, 3.1416] -> [400.0, -240.0, 450.0, 0.0, 0.0, 3.1416]/[400.0, -100.0, 700.0, 0.0, 0.0, 3.1416]
    memcpy(&pose, tmp12, sizeof(pose));

    t1.type             = MOTION_LINE;
    t1.cnt              = 50;
    t1.linear_velocity  = 500;
    t1.pose_target      = pose;
    
    memcpy(&pose, tmp17, sizeof(pose));

    t2.type             = MOTION_CIRCLE;
    t2.cnt              = 0;
    t2.linear_velocity  = 500;
    t2.circle_target.pose1  = pose;

    memcpy(&pose, tmp18, sizeof(pose));
    t2.circle_target.pose2  = pose;

    t3.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 16);
    motion2 = new MoveCommand(planner_, t2, t3, 17);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = NULL;

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());
    
    delete motion1;     motion1 = NULL;
    delete motion2;     motion2 = NULL;

    // [380.0,0.0,617.5,0.0,0.0,3.1416] -> [400.0, -400.0, 500.0, 0.0, 0.0, 3.1416] -> [400.0, -240.0, 450.0, 0.0, 0.0, 3.1416]/[400.0, -100.0, 700.0, 0.0, 0.0, 3.1416]
    memcpy(&pose, tmp19, sizeof(pose));

    t1.type             = MOTION_LINE;
    t1.cnt              = 10;
    t1.linear_velocity  = 500;
    t1.pose_target      = pose;
    
    memcpy(&pose, tmp17, sizeof(pose));

    t2.type             = MOTION_CIRCLE;
    t2.cnt              = 0;
    t2.linear_velocity  = 500;
    t2.circle_target.pose1  = pose;

    memcpy(&pose, tmp18, sizeof(pose));
    t2.circle_target.pose2  = pose;

    t3.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 16);
    motion2 = new MoveCommand(planner_, t2, t3, 17);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = NULL;

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());
    
    delete motion1;     motion1 = NULL;
    delete motion2;     motion2 = NULL;
}

TEST_F(PlanningTest, MoveC)
{
    double tmp1[] = {0.0, 0.0, 0.0, 0.0, -1.5708, 0.0};
    double tmp2[] = {0.0, 1.0, -1.0, 2.1, 1.5708, 0.3};
    double tmp3[] = {0.8, 0.2, 0.3, -0.5, 0.7, -1.0};

    double tmp11[] = {380.0, 0.0, 617.5, 0.0, 0.0, 3.1416};
    double tmp12[] = {400.0, 240.0, 650.0, 0.0, 0.0, 3.1416};
    double tmp13[] = {400.0, 360.0, 350.0, 0.0, 0.0, 3.1416};
    double tmp14[] = {400.0, -400.0, 500.0, 0.0, 0.0, 3.1416};
    double tmp15[] = {400.0, -240.0, 600.0, 0.0, 0.0, 3.1416};
    double tmp16[] = {400.0, 0.0, 300.0, 0.0, 0.0, 3.1416};
    double tmp17[] = {400.0, -100.0, 600.0, 0.0, 0.0, 3.1416};
    double tmp18[] = {400.0, 400.0, 400.0, 0.0, 0.0, 3.1416};
    double tmp19[] = {400.0, -240.0, 450.0, 0.0, 0.0, 3.1416};
    double tmp20[] = {400.0, 100.0, 650.0, 0.0, 0.0, 3.1416};
    double tmp21[] = {400.0, 300.0, 400.0, 0.0, 0.0, 3.1416};


    MotionTarget    t1, t2, t3, t4;
    Joint           joint;
    PoseEuler       pose;


    // [380.0, 0.0, 617.5, 0.0, 0.0, 3.1416] -> [400.0, 240.0, 650.0, 0.0, 0.0, 3.1416]/[400.0, 360.0, 350.0, 0.0, 0.0, 3.1416]
    t1.type             = MOTION_CIRCLE;
    t1.cnt              = 0;
    t1.linear_velocity  = 800;
    
    memcpy(&pose, tmp12, sizeof(pose));
    t1.circle_target.pose1  = pose;
    
    memcpy(&pose, tmp13, sizeof(pose));
    t1.circle_target.pose2  = pose;

    t2.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 1);
    ASSERT_TRUE(NULL != motion1);

    motion1->prev = NULL;
    motion1->next = NULL;

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());

    delete motion1;     motion1 = NULL;
    
    // [400.0, -400.0, 500.0, 0.0, 0.0, 3.1416] -> [400.0, -240.0, 600.0, 0.0, 0.0, 3.1416]/[400.0, 0.0, 300.0, 0.0, 0.0, 3.1416]
    memcpy(&pose, tmp14, sizeof(pose));

    t1.type             = MOTION_LINE;
    t1.cnt              = 0;
    t1.linear_velocity  = 800;
    t1.pose_target      = pose;

    t2.type             = MOTION_CIRCLE;
    t2.cnt              = 0;
    t2.linear_velocity  = 800;

    memcpy(&pose, tmp15, sizeof(pose));
    t2.circle_target.pose1  = pose;

    memcpy(&pose, tmp16, sizeof(pose));
    t2.circle_target.pose2  = pose;

    t3.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 2);
    ASSERT_TRUE(NULL != motion1);

    motion1->prev = NULL;
    motion1->next = NULL;

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());

    motion2 = new MoveCommand(planner_, t2, t3, 3);

    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = NULL;

    ASSERT_EQ(0, motion2->PlanTraj());

    delete motion1;     motion1 = NULL;
    delete motion2;     motion2 = NULL;

    // [400.0, -400.0, 500.0, 0.0, 0.0, 3.1416] -> [400.0, -240.0, 600.0, 0.0, 0.0, 3.1416]/[400.0, -100.0, 600.0, 0.0, 0.0, 3.1416]
    memcpy(&pose, tmp14, sizeof(pose));

    t1.type             = MOTION_LINE;
    t1.cnt              = 0;
    t1.linear_velocity  = 800;
    t1.pose_target      = pose;

    t2.type             = MOTION_CIRCLE;
    t2.cnt              = 0;
    t2.linear_velocity  = 800;

    memcpy(&pose, tmp15, sizeof(pose));
    t2.circle_target.pose1  = pose;

    memcpy(&pose, tmp17, sizeof(pose));
    t2.circle_target.pose2  = pose;

    t3.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 2);
    ASSERT_TRUE(NULL != motion1);

    motion1->prev = NULL;
    motion1->next = NULL;

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());

    motion2 = new MoveCommand(planner_, t2, t3, 3);

    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = NULL;

    ASSERT_EQ(0, motion2->PlanTraj());

    delete motion1;     motion1 = NULL;
    delete motion2;     motion2 = NULL;

    // [380.0, 0.0, 617.5, 0.0, 0.0, 3.1416] -> [400.0, 240.0, 650.0, 0.0, 0.0, 3.1416]/[400.0, 360.0, 350.0, 0.0, 0.0, 3.1416] -> (0.0, 0.0, 0.0, 0.0, -1.5708, 0.0)
    t1.type             = MOTION_CIRCLE;
    t1.cnt              = 100;
    t1.linear_velocity  = 800;
    
    memcpy(&pose, tmp12, sizeof(pose));
    t1.circle_target.pose1  = pose;
    
    memcpy(&pose, tmp13, sizeof(pose));
    t1.circle_target.pose2  = pose;

    t2.type             = MOTION_JOINT;
    t2.cnt              = 0;
    t2.percent_velocity = 50;

    memcpy(&joint, tmp1, sizeof(joint));
    t2.joint_target     = joint;

    t3.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 4);
    motion2 = new MoveCommand(planner_, t2, t3, 5);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = NULL;

    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());

    delete motion1;     motion1 = NULL;
    delete motion2;     motion2 = NULL;

    // [400.0, -400.0, 500.0, 0.0, 0.0, 3.1416] -> [400.0, -240.0, 600.0, 0.0, 0.0, 3.1416]/[400.0, 0.0, 300.0, 0.0, 0.0, 3.1416] -> (0.0, 1.0, -1.0, 2.1, 1.5708, 0.3)
    memcpy(&pose, tmp14, sizeof(pose));

    t1.type             = MOTION_LINE;
    t1.cnt              = 0;
    t1.linear_velocity  = 800;
    t1.pose_target      = pose;

    t2.type             = MOTION_CIRCLE;
    t2.cnt              = 50;
    t2.linear_velocity  = 800;

    memcpy(&pose, tmp15, sizeof(pose));
    t2.circle_target.pose1  = pose;

    memcpy(&pose, tmp16, sizeof(pose));
    t2.circle_target.pose2  = pose;

    memcpy(&joint, tmp2, sizeof(joint));
    t3.type             = MOTION_JOINT;
    t3.cnt              = 0;
    t3.percent_velocity = 50;
    t3.joint_target     = joint;

    t4.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 6);
    motion2 = new MoveCommand(planner_, t2, t3, 7);
    motion3 = new MoveCommand(planner_, t3, t4, 8);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);
    ASSERT_TRUE(NULL != motion3);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = motion3;
    motion3->prev = motion2;
    motion3->next = NULL;

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());
    ASSERT_EQ(0, motion3->PlanTraj());

    delete motion1;     motion1 = NULL;
    delete motion2;     motion2 = NULL;
    delete motion3;     motion3 = NULL;

    // [400.0, -400.0, 500.0, 0.0, 0.0, 3.1416] -> [400.0, -240.0, 600.0, 0.0, 0.0, 3.1416]/[400.0, -100.0, 600.0, 0.0, 0.0, 3.1416] -> (0.8, 0.2, 0.3, -0.5, 0.7, -1.0)
    memcpy(&pose, tmp14, sizeof(pose));

    t1.type             = MOTION_LINE;
    t1.cnt              = 0;
    t1.linear_velocity  = 800;
    t1.pose_target      = pose;

    t2.type             = MOTION_CIRCLE;
    t2.cnt              = 50;
    t2.linear_velocity  = 800;

    memcpy(&pose, tmp15, sizeof(pose));
    t2.circle_target.pose1  = pose;

    memcpy(&pose, tmp17, sizeof(pose));
    t2.circle_target.pose2  = pose;

    memcpy(&joint, tmp3, sizeof(joint));
    t3.type             = MOTION_JOINT;
    t3.cnt              = 0;
    t3.percent_velocity = 50;
    t3.joint_target     = joint;

    t4.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 9);
    motion2 = new MoveCommand(planner_, t2, t3, 10);
    motion3 = new MoveCommand(planner_, t3, t4, 11);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);
    ASSERT_TRUE(NULL != motion3);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = motion3;
    motion3->prev = motion2;
    motion3->next = NULL;

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());
    ASSERT_EQ(0, motion3->PlanTraj());

    delete motion1;     motion1 = NULL;
    delete motion2;     motion2 = NULL;
    delete motion3;     motion3 = NULL;

    // [380.0, 0.0, 617.5, 0.0, 0.0, 3.1416] -> [400.0, 240.0, 650.0, 0.0, 0.0, 3.1416]/[400.0, 360.0, 350.0, 0.0, 0.0, 3.1416] -> [400.0, -400.0, 500.0, 0.0, 0.0, 3.1416]
    t1.type             = MOTION_CIRCLE;
    t1.cnt              = 100;
    t1.linear_velocity  = 800;
    
    memcpy(&pose, tmp12, sizeof(pose));
    t1.circle_target.pose1  = pose;
    
    memcpy(&pose, tmp13, sizeof(pose));
    t1.circle_target.pose2  = pose;

    t2.type             = MOTION_LINE;
    t2.cnt              = 0;
    t2.linear_velocity  = 800;

    memcpy(&pose, tmp14, sizeof(pose));
    t2.pose_target      = pose;

    t3.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 12);
    motion2 = new MoveCommand(planner_, t2, t3, 13);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = NULL;

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());

    delete motion1;     motion1 = NULL;
    delete motion2;     motion2 = NULL;

    // [400.0, -400.0, 500.0, 0.0, 0.0, 3.1416] -> [400.0, -240.0, 600.0, 0.0, 0.0, 3.1416]/[400.0, 0.0, 300.0, 0.0, 0.0, 3.1416] -> [380.0, 0.0, 617.5, 0.0, 0.0, 3.1416]
    memcpy(&pose, tmp14, sizeof(pose));

    t1.type             = MOTION_LINE;
    t1.cnt              = 0;
    t1.linear_velocity  = 800;
    t1.pose_target      = pose;

    t2.type             = MOTION_CIRCLE;
    t2.cnt              = 50;
    t2.linear_velocity  = 800;

    memcpy(&pose, tmp15, sizeof(pose));
    t2.circle_target.pose1  = pose;

    memcpy(&pose, tmp16, sizeof(pose));
    t2.circle_target.pose2  = pose;

    t3.type             = MOTION_LINE;
    t3.cnt              = 0;
    t3.linear_velocity  = 800;

    memcpy(&pose, tmp11, sizeof(pose));
    t3.pose_target      = pose;

    t4.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 6);
    motion2 = new MoveCommand(planner_, t2, t3, 7);
    motion3 = new MoveCommand(planner_, t3, t4, 8);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);
    ASSERT_TRUE(NULL != motion3);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = motion3;
    motion3->prev = motion2;
    motion3->next = NULL;

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());
    ASSERT_EQ(0, motion3->PlanTraj());

    delete motion1;     motion1 = NULL;
    delete motion2;     motion2 = NULL;
    delete motion3;     motion3 = NULL;

    // [400.0, -400.0, 500.0, 0.0, 0.0, 3.1416] -> [400.0, -240.0, 600.0, 0.0, 0.0, 3.1416]/[400.0, -100.0, 600.0, 0.0, 0.0, 3.1416] -> [400.0, 400.0, 400.0, 0.0, 0.0, 3.1416]
    memcpy(&pose, tmp14, sizeof(pose));

    t1.type             = MOTION_LINE;
    t1.cnt              = 0;
    t1.linear_velocity  = 800;
    t1.pose_target      = pose;

    t2.type             = MOTION_CIRCLE;
    t2.cnt              = 10;
    t2.linear_velocity  = 800;

    memcpy(&pose, tmp15, sizeof(pose));
    t2.circle_target.pose1  = pose;

    memcpy(&pose, tmp17, sizeof(pose));
    t2.circle_target.pose2  = pose;

    t3.type             = MOTION_LINE;
    t3.cnt              = 0;
    t3.linear_velocity  = 800;

    memcpy(&pose, tmp18, sizeof(pose));
    t3.pose_target      = pose;

    t4.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 9);
    motion2 = new MoveCommand(planner_, t2, t3, 10);
    motion3 = new MoveCommand(planner_, t3, t4, 11);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);
    ASSERT_TRUE(NULL != motion3);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = motion3;
    motion3->prev = motion2;
    motion3->next = NULL;

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());
    ASSERT_EQ(0, motion3->PlanTraj());

    delete motion1;     motion1 = NULL;
    delete motion2;     motion2 = NULL;
    delete motion3;     motion3 = NULL;


    // [400.0, -400.0, 500.0, 0.0, 0.0, 3.1416] -> [400.0, -240.0, 450.0, 0.0, 0.0, 3.1416]/[400.0, -100.0, 600.0, 0.0, 0.0, 3.1416] -> [400.0, 100.0, 650.0, 0.0, 0.0, 3.1416]/[400.0, 300.0, 400.0, 0.0, 0.0, 3.1416]
    memcpy(&pose, tmp14, sizeof(pose));

    t1.type             = MOTION_LINE;
    t1.cnt              = 0;
    t1.linear_velocity  = 800;
    t1.pose_target      = pose;

    t2.type             = MOTION_CIRCLE;
    t2.cnt              = 100;
    t2.linear_velocity  = 800;

    memcpy(&pose, tmp19, sizeof(pose));
    t2.circle_target.pose1  = pose;

    memcpy(&pose, tmp17, sizeof(pose));
    t2.circle_target.pose2  = pose;

    t3.type             = MOTION_CIRCLE;
    t3.cnt              = 0;
    t3.linear_velocity  = 800;

    memcpy(&pose, tmp20, sizeof(pose));
    t3.circle_target.pose1  = pose;

    memcpy(&pose, tmp21, sizeof(pose));
    t3.circle_target.pose2  = pose;

    t4.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 12);
    motion2 = new MoveCommand(planner_, t2, t3, 13);
    motion3 = new MoveCommand(planner_, t3, t4, 14);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);
    ASSERT_TRUE(NULL != motion3);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = motion3;
    motion3->prev = motion2;
    motion3->next = NULL;

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());
    ASSERT_EQ(0, motion3->PlanTraj());

    delete motion1;     motion1 = NULL;
    delete motion2;     motion2 = NULL;
    delete motion3;     motion3 = NULL;


    // [400.0, -400.0, 500.0, 0.0, 0.0, 3.1416] -> [400.0, -240.0, 450.0, 0.0, 0.0, 3.1416]/[400.0, -100.0, 600.0, 0.0, 0.0, 3.1416] -> [400.0, 100.0, 650.0, 0.0, 0.0, 3.1416]/[400.0, 300.0, 400.0, 0.0, 0.0, 3.1416]
    memcpy(&pose, tmp21, sizeof(pose));

    t1.type             = MOTION_LINE;
    t1.cnt              = 0;
    t1.linear_velocity  = 800;
    t1.pose_target      = pose;

    t2.type             = MOTION_CIRCLE;
    t2.cnt              = 50;
    t2.linear_velocity  = 800;

    memcpy(&pose, tmp20, sizeof(pose));
    t2.circle_target.pose1  = pose;

    memcpy(&pose, tmp17, sizeof(pose));
    t2.circle_target.pose2  = pose;

    t3.type             = MOTION_CIRCLE;
    t3.cnt              = 0;
    t3.linear_velocity  = 800;

    memcpy(&pose, tmp19, sizeof(pose));
    t3.circle_target.pose1  = pose;

    memcpy(&pose, tmp14, sizeof(pose));
    t3.circle_target.pose2  = pose;

    t4.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 15);
    motion2 = new MoveCommand(planner_, t2, t3, 16);
    motion3 = new MoveCommand(planner_, t3, t4, 17);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);
    ASSERT_TRUE(NULL != motion3);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = motion3;
    motion3->prev = motion2;
    motion3->next = NULL;

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());
    ASSERT_EQ(0, motion3->PlanTraj());

    delete motion1;     motion1 = NULL;
    delete motion2;     motion2 = NULL;
    delete motion3;     motion3 = NULL;
}

TEST_F(PlanningTest, PickPoint)
{
    double tmp1[] = {0.0, 0.0, 0.0, 0.0, -1.5708, 0.0};
    double tmp2[] = {400.0, 400.0, 400.0, 0.0, 0.0, 3.1416};

    MotionTarget    t1, t2;
    PoseEuler       pose;
    Joint           joint;
    
    memcpy(&pose, tmp2, sizeof(pose));

    t1.type             = MOTION_LINE;
    t1.cnt              = 0;
    t1.linear_velocity  = 500;
    t1.pose_target      = pose;

    t2.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 1);
    ASSERT_TRUE(NULL != motion1);

    motion1->prev = NULL;
    motion1->next = NULL;

    memcpy(&joint, tmp1, sizeof(joint));
    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());

    vector<PathPoint> p;

    motion1->setPickedlength(300);
    motion1->PickPoint(p);

    ASSERT_EQ(100, p.size());
    ASSERT_EQ(300, p.front().stamp);
    ASSERT_EQ(400, motion1->getPickedlength());

    motion1->setPickedlength(0);
    p.clear();
    motion1->PickPoint(p);
    ASSERT_EQ(0, p.front().stamp);
    p.clear();
    motion1->PickPoint(p);
    ASSERT_EQ(100, p.front().stamp);
    p.clear();
    motion1->PickPoint(p);
    ASSERT_EQ(200, p.front().stamp);
    p.clear();
    motion1->PickPoint(p);
    ASSERT_EQ(300, p.front().stamp);
    p.clear();
    motion1->PickPoint(p);
    ASSERT_EQ(400, p.front().stamp);
    

    motion1->setPickedlength(0);
    p.clear();
    motion1->PickPoint(p, 50);
    ASSERT_EQ(0, p.front().stamp);
    p.clear();
    motion1->PickPoint(p, 150);
    ASSERT_EQ(50, p.front().stamp);
    p.clear();
    motion1->PickPoint(p, 250);
    ASSERT_EQ(200, p.front().stamp);
    p.clear();
    motion1->PickPoint(p, 350);
    ASSERT_EQ(450, p.front().stamp);
    p.clear();
    motion1->PickPoint(p, 60);
    ASSERT_EQ(800, p.front().stamp);

    delete motion1;     motion1 = NULL;
}

TEST_F(PlanningTest, PauseAndResume_1)
{
    double tmp1[] = {0.0, 0.0, 0.0, 0.0, -1.5708, 0.0};
    double tmp2[] = {400.0, 400.0, 400.0, 0.0, 0.0, 3.1416};
    double tmp3[] = {400.0, -400.0, 500.0, 0.0, 0.0, 3.1416};

    MotionTarget    t1, t2, t3, t4;
    PoseEuler       pose;
    Joint           joint;
    
    memcpy(&pose, tmp2, sizeof(pose));

    t1.type             = MOTION_LINE;
    t1.cnt              = 50;
    t1.linear_velocity  = 500;
    t1.pose_target      = pose;

    memcpy(&pose, tmp3, sizeof(pose));

    t2.type             = MOTION_LINE;
    t2.cnt              = 50;
    t2.linear_velocity  = 500;
    t2.pose_target      = pose;

    memcpy(&joint, tmp1, sizeof(joint));

    t3.type             = MOTION_JOINT;
    t3.cnt              = 0;
    t3.percent_velocity = 50;
    t3.joint_target     = joint;

    t4.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 1);
    motion2 = new MoveCommand(planner_, t2, t3, 2);
    motion3 = new MoveCommand(planner_, t3, t4, 3);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);
    ASSERT_TRUE(NULL != motion3);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = motion3;
    motion3->prev = motion2;
    motion3->next = NULL;

    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());
    ASSERT_EQ(0, motion3->PlanTraj());

    vector<PathPoint>   pv;
    vector<JointPoint>  jv;
    JointPoint          jp, jpend;

    motion1->PickPoint(pv, 1000);
    ASSERT_EQ(1000, pv.size());

    jv.resize(1000);
    
    for (int i = 0; i < 1000; ++i) {
        ASSERT_EQ(0, planner_->InverseKinematics(pv[i].pose, joint, jv[i].joint));
        jv[i].stamp     = pv[i].stamp;
        jv[i].level     = pv[i].level;
        jv[i].source    = pv[i].source;

        joint   = jv[i].joint;
    }

    vector<JointPoint>  jvout;

    jv.erase(jv.begin(), jv.begin() + 600);
    
    ASSERT_EQ(400, jv.size());
    ASSERT_EQ(0, planner_->Pause(jv, jvout, jpend));

    joint = jvout.back().joint;
    jvout.clear();

    motion1->setPickedlength(jpend.stamp + 1);
    ASSERT_EQ(0, motion1->Resume(joint, jpend.joint, jvout));

    motion1->PickPoint(pv);
    ASSERT_EQ(jvout.back().stamp + 1, pv.front().stamp);

    delete motion1;     motion1 = NULL;
    delete motion2;     motion2 = NULL;
    delete motion3;     motion3 = NULL;
}

TEST_F(PlanningTest, PauseAndResume_2)
{
    double tmp1[] = {0.0, 0.0, 0.0, 0.0, -1.5708, 0.0};
    double tmp2[] = {280.0, 200.0, 400.0, 0.0, 0.0, 3.1416};
    double tmp3[] = {400.0, -400.0, 500.0, 0.0, 0.0, 3.1416};

    MotionTarget    t1, t2, t3, t4;
    PoseEuler       pose;
    Joint           joint;
    
    memcpy(&pose, tmp2, sizeof(pose));

    t1.type             = MOTION_LINE;
    t1.cnt              = 50;
    t1.linear_velocity  = 500;
    t1.pose_target      = pose;

    memcpy(&pose, tmp3, sizeof(pose));

    t2.type             = MOTION_LINE;
    t2.cnt              = 50;
    t2.linear_velocity  = 500;
    t2.pose_target      = pose;

    memcpy(&joint, tmp1, sizeof(joint));

    t3.type             = MOTION_JOINT;
    t3.cnt              = 0;
    t3.percent_velocity = 50;
    t3.joint_target     = joint;

    t4.type             = MOTION_NONE;

    motion1 = new MoveCommand(planner_, t1, t2, 1);
    motion2 = new MoveCommand(planner_, t2, t3, 2);
    motion3 = new MoveCommand(planner_, t3, t4, 3);
    ASSERT_TRUE(NULL != motion1);
    ASSERT_TRUE(NULL != motion2);
    ASSERT_TRUE(NULL != motion3);

    motion1->prev = NULL;
    motion1->next = motion2;
    motion2->prev = motion1;
    motion2->next = motion3;
    motion3->prev = motion2;
    motion3->next = NULL;

    motion1->setInitialJoint(joint);

    ASSERT_EQ(0, motion1->PlanTraj());
    ASSERT_EQ(0, motion2->PlanTraj());
    ASSERT_EQ(0, motion3->PlanTraj());

    vector<PathPoint>   pv;
    vector<JointPoint>  jv;
    JointPoint          jp, jpend;
    Joint               jout;

    printf("traj len=%d\n", motion1->getTrajLength());
    motion1->PickPoint(pv);
    int cnt = 0;

    while(pv.size() > 0) {
        printf("loop:%d\n", cnt++);
        for (int i=0; i < pv.size(); ++i) {
            ASSERT_EQ(0, planner_->InverseKinematics(pv[i].pose, joint, jout));
            joint = jout;
            printf("%.6f, %.6f, %.6f, %.6f, %.6f, %.6f\n",
                   joint.j1, joint.j2, joint.j3, joint.j4, joint.j5, joint.j6);
        }

        pv.clear();
        motion1->PickPoint(pv);
    }

    printf("here\n");

    motion2->PickPoint(pv, 1000);
    
    ASSERT_EQ(1000, pv.size());
    
    jv.resize(1000);
    
    for (int i = 0; i < 1000; ++i) {
        ASSERT_EQ(0, planner_->InverseKinematics(pv[i].pose, joint, jv[i].joint));
        jv[i].stamp     = pv[i].stamp;
        jv[i].level     = pv[i].level;
        jv[i].source    = pv[i].source;

        joint   = jv[i].joint;
    }

    vector<JointPoint>  jvout;

    jv.erase(jv.begin(), jv.begin() + 600);
    
    ASSERT_EQ(400, jv.size());
    ASSERT_EQ(0, planner_->Pause(jv, jvout, jpend));

    joint = jvout.back().joint;
    jvout.clear();

    motion2->setPickedlength(jpend.stamp + 1);
    ASSERT_EQ(0, motion2->Resume(joint, jpend.joint, jvout));

    motion2->PickPoint(pv);
    ASSERT_EQ(jvout.back().stamp + 1, pv.front().stamp);

    delete motion1;     motion1 = NULL;
    delete motion2;     motion2 = NULL;
    delete motion3;     motion3 = NULL;
}



