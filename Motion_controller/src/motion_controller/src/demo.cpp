#include <motion_controller/motion_controller_arm_group.h>
using std::string;
using std::cout;
using std::endl;
using namespace fst_controller;

class A
{
  public:
    void test()
    {
        cout << "Test:" << endl;
        TrajPlan *plan = new TrajPlan();
        MotionTarget t1,t2;
        MoveCommand *cmd1 = new MoveCommand(plan, t1, t2, 15);
        MoveCommand *cmd2 = new MoveCommand(plan, t1, t2, 15);
        MoveCommand *cmd3 = new MoveCommand(plan, t1, t2, 15);
        cout << "construct MoveCommand Success" << endl;
        delete cmd1;
        cout << "delete cmd1" << endl;
        delete cmd2;
        cout << "delete cmd2" << endl;
        delete cmd3;
        cout << "delete cmd3" << endl;
    }
  private:
    MoveCommand *p;
};


int main(int argc, char **argv)
{
    A *a = new A();
    a->test();

    ArmGroup *arm = new ArmGroup();
    vector<ErrorCode> err;
    arm->initArmGroup(err);
    delete arm;

    return 0;
}
