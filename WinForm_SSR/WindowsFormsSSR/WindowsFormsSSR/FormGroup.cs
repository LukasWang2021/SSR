using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using ProcessMotion;

namespace WindowsFormsSSR
{
    public partial class FormGroup : Form
    {
        public FormGroup()
        {
            InitializeComponent();
        }

        private Int32 getGroupId()
        {
            Int32 groupId;
            if (!int.TryParse(textBox1.Text, out groupId))
            {
                MessageBox.Show("Invalid Input from <Group_ID>!! Please enter valid number.");
                return -1;
            }
            return groupId;
        }

        private void button1_Click(object sender, EventArgs e)
        {
            Int32 groupId = getGroupId();
            if (groupId < 0) return;

            UInt64 result = ProcessMotionCtrl.c_groupReset(groupId);
            if (result != 0)
            {
                MessageBox.Show("group[" + groupId.ToString() + "] reset error failed,ErrorCode: 0x" + result.ToString("X"));
                return;
            }
        }

        private void button2_Click(object sender, EventArgs e)
        {
            Int32 groupId = getGroupId();
            if (groupId < 0) return;

            UInt64 result = ProcessMotionCtrl.c_groupEnable(groupId);
            if (result != 0)
            {
                MessageBox.Show("group[" + groupId.ToString() + "] power on failed,ErrorCode: 0x" + result.ToString("X"));
                return;
            }
        }

        private void button3_Click(object sender, EventArgs e)
        {
            Int32 groupId = getGroupId();
            if (groupId < 0) return;

            UInt64 result = ProcessMotionCtrl.c_groupDisable(groupId);
            if (result != 0)
            {
                MessageBox.Show("group[" + groupId.ToString() + "] power off failed,ErrorCode: 0x" + result.ToString("X"));
                return;
            }
        }

        private void button4_Click(object sender, EventArgs e)
        {
            Int32 groupId = getGroupId();
            if (groupId < 0) return;

            UInt64 result = ProcessMotionCtrl.c_groupResetAllEncoder(groupId);
            if (result != 0)
            {
                MessageBox.Show("group[" + groupId.ToString() + "] encoder reset failed,ErrorCode: 0x" + result.ToString("X"));
                return;
            }
        }

        private void button5_Click(object sender, EventArgs e)
        {
            Int32 groupId = getGroupId();
            if (groupId < 0) return;

            Int32 velPercent;
            if (!int.TryParse(textBoxVelRatio.Text, out velPercent))
            {
                MessageBox.Show("Invalid Input from <setVel>!! Please enter valid number.");
                return;
            }

            UInt64 result = ProcessMotionCtrl.c_mcSetGlobalVelRatio(velPercent);
            if (result != 0)
            {
                MessageBox.Show("group[" + groupId.ToString() + "] set velocity ratio failed,ErrorCode: 0x" + result.ToString("X"));
                return;
            }
        }

        private void button6_Click(object sender, EventArgs e)
        {
            Int32 groupId = getGroupId();
            if (groupId < 0) return;

            Int32 accPercent;
            if (!int.TryParse(textBoxAccRatio.Text, out accPercent))
            {
                MessageBox.Show("Invalid Input from <setVel>!! Please enter valid number.");
                return;
            }

            UInt64 result = ProcessMotionCtrl.c_mcSetGlobalAccRatio(accPercent);
            if (result != 0)
            {
                MessageBox.Show("group[" + groupId.ToString() + "] set acceleration ratio failed,ErrorCode: 0x" + result.ToString("X"));
                return;
            }
        }

        private void button7_Click(object sender, EventArgs e)
        {
            Int32 groupId = getGroupId();
            if (groupId < 0) return;

            Int32 axisID;
            if (!int.TryParse(comboBoxAxisID1.SelectedItem.ToString(), out axisID))
            {
                MessageBox.Show("Invalid Input from MoveStep <Axis_ID>!! Please enter valid number.");
                return;
            }
            axisID--;

            Int32 direction = 0;
            switch (comboBoxAxisDirection1.SelectedItem.ToString())
            {
                case "StandStill":
                    direction = 0;
                    break;
                case "Increase":
                    direction = 1;
                    break;
                case "Decrease":
                    direction = 2;
                    break;
                default:
                    MessageBox.Show("Invalid Input from MoveStep <Axis_direction>!! Please enter valid number.");
                    break;
            }

            UInt64 result = ProcessMotionCtrl.c_mcDoStepManualMove(axisID, direction);
            if (result != 0)
            {
                MessageBox.Show("group[" + groupId.ToString() + "] set acceleration ratio failed,ErrorCode: 0x" + result.ToString("X"));
                return;
            }
        }

        private void button8_Click(object sender, EventArgs e)
        {
            Int32 groupId = getGroupId();
            if (groupId < 0) return;

            UInt64 result = ProcessMotionCtrl.c_mcIgnoreLostZeroError();
            if (result != 0)
            {
                MessageBox.Show("group[" + groupId.ToString() + "] ignore the zero offset error failed,ErrorCode: 0x" + result.ToString("X"));
                return;
            }
        }

        private void button9_Click(object sender, EventArgs e)
        {
            Int32 groupId = getGroupId();
            if (groupId < 0) return;

            UInt64 result = ProcessMotionCtrl.c_mcSetAllZeroPointOffsets();
            if (result != 0)
            {
                MessageBox.Show("group[" + groupId.ToString() + "] set the zero offset error failed,ErrorCode: 0x" + result.ToString("X"));
                return;
            }
        }

        private void button10_Click(object sender, EventArgs e)
        {
            Int32 groupId = getGroupId();
            if (groupId < 0) return;

            Int32 axisID;
            if (!int.TryParse(comboBoxAxisID2.SelectedItem.ToString(), out axisID))
            {
                MessageBox.Show("Invalid Input from MoveContinous <Axis_ID>!! Please enter valid number.");
                return;
            }
            axisID--;
            Console.WriteLine("moveContinuous axis id =" + comboBoxAxisID1.SelectedItem.ToString());

            Int32 direction = 0;
            switch (comboBoxAxisDirection2.SelectedItem.ToString())
            {
                case "StandStill":
                    direction = 0;
                    break;
                case "Increase":
                    direction = 1;
                    break;
                case "Decrease":
                    direction = 2;
                    break;
                default:
                    MessageBox.Show("Invalid Input from MoveStep <Axis_direction>!! Please enter valid number.");
                    break;
            }
            Console.WriteLine("moveContinuous axis direction =" + comboBoxAxisID1.SelectedItem.ToString());

            UInt64 result = ProcessMotionCtrl.c_mcDoContinuousManualMove(axisID, direction);
            if (result != 0)
            {
                MessageBox.Show("group[" + groupId.ToString() + "] move continuous failed,ErrorCode: 0x" + result.ToString("X"));
                return;
            }
        }

        private void button11_Click(object sender, EventArgs e)
        {
            Int32 groupId = getGroupId();
            ProcessMotionCtrl.c_mcDoContinuousManualToStandstill();
        }

        private void button12_Click(object sender, EventArgs e)
        {
            Int32 groupId = getGroupId();
            if (groupId < 0) return;

            double[] joint = new double[6];
            if (!double.TryParse(textBoxJoint1.Text, out joint[0]))
            {
                MessageBox.Show("Invalid Input from moveJoint <J1>!! Please enter valid number.");
                return;
            }
            
            if (!double.TryParse(textBoxJoint2.Text, out joint[1]))
            {
                MessageBox.Show("Invalid Input from moveJoint <J2>!! Please enter valid number.");
                return;
            }
          
            if (!double.TryParse(textBoxJoint3.Text, out joint[2]))
            {
                MessageBox.Show("Invalid Input from moveJoint <J3>!! Please enter valid number.");
                return;
            }
         
            if (!double.TryParse(textBoxJoint4.Text, out joint[3]))
            {
                MessageBox.Show("Invalid Input from moveJoint <J4>!! Please enter valid number.");
                return;
            }
    
            if (!double.TryParse(textBoxJoint5.Text, out joint[4]))
            {
                MessageBox.Show("Invalid Input from moveJoint <J5>!! Please enter valid number.");
                return;
            }

            if (!double.TryParse(textBoxJoint6.Text, out joint[5]))
            {
                MessageBox.Show("Invalid Input from moveJoint <J6>!! Please enter valid number.");
                return;
            }

            for(Int32 i = 0; i < 6; ++i)
            {
                joint[i] = joint[i] * Math.PI / 180;
            }

            UInt64 result = ProcessMotionCtrl.c_mcDoGotoJointMove(groupId, joint[0], joint[1], joint[2], joint[3], joint[4], joint[5], 0, 0, 0);
            if (result != 0)
            {
                MessageBox.Show("group[" + groupId.ToString() + "] move to Joints failed,ErrorCode: 0x" + result.ToString("X"));
                return;
            }
        }

        private void button13_Click(object sender, EventArgs e)
        {
            UInt64 result = ProcessMotionCtrl.c_mcSetCoordinate(comboBoxCoordinate.SelectedItem.ToString());
            if (result != 0)
            {
                MessageBox.Show("Set acceleration ratio failed,ErrorCode: 0x" + result.ToString("X"));
                return;
            }

        }

        private void button14_Click(object sender, EventArgs e)
        {
            double jointStep = 0;
            if (!double.TryParse(textBoxJointStep.Text, out jointStep))
            {
                MessageBox.Show("Invalid Input from setStep <Joint>!! Please enter valid number.");
                return;
            }
            jointStep = jointStep * Math.PI / 180;//degree -> rad

            double cartStep = 0; //mm
            if (!double.TryParse(textBoxCartStep.Text, out cartStep))
            {
                MessageBox.Show("Invalid Input from setStep <Cartesian>!! Please enter valid number.");
                return;
            }

            double OrienStep = 0;
            if (!double.TryParse(textBoxOrienStep.Text, out OrienStep))
            {
                MessageBox.Show("Invalid Input from setStep <Orientation>!! Please enter valid number.");
                return;
            }
            OrienStep = OrienStep * Math.PI / 180;//degree -> rad

            UInt64 result = ProcessMotionCtrl.c_mcSetStep(jointStep, cartStep, OrienStep);
            if (result != 0)
            {
                MessageBox.Show("Set Step failed,ErrorCode: 0x" + result.ToString("X"));
                return;
            }
        }

        private void button15_Click(object sender, EventArgs e)
        {
            Int32 groupId = getGroupId();
            if (groupId < 0) return;

            double cartX;
            if (!double.TryParse(textBoxCartX.Text, out cartX))
            {
                MessageBox.Show("Invalid Input from move cart <X>!! Please enter valid number.");
                return;
            }
            double cartY; 
            if (!double.TryParse(textBoxCartY.Text, out cartY))
            {
                MessageBox.Show("Invalid Input from moce cart <Y>!! Please enter valid number.");
                return;
            }
            double cartZ;
            if (!double.TryParse(textBoxCartZ.Text, out cartZ))
            {
                MessageBox.Show("Invalid Input from move cart <Z>!! Please enter valid number.");
                return;
            }
            double cartA;
            if (!double.TryParse(textBoxCartA.Text, out cartA))
            {
                MessageBox.Show("Invalid Input from moce cart <A>!! Please enter valid number.");
                return;
            }
            double cartB;
            if (!double.TryParse(textBoxCartB.Text, out cartB))
            {
                MessageBox.Show("Invalid Input from move cart <B>!! Please enter valid number.");
                return;
            }
            double cartC;
            if (!double.TryParse(textBoxCartC.Text, out cartC))
            {
                MessageBox.Show("Invalid Input from moce cart <C>!! Please enter valid number.");
                return;
            }

            int PostureArm;
            if (!int.TryParse(textBoxPostureArm.Text, out PostureArm))
            {
                MessageBox.Show("Invalid Input from moce cart <A>!! Please enter valid number.");
                return;
            }
            int PostureElbow; ;
            if (!int.TryParse(textBoxPostureElbow.Text, out PostureElbow))
            {
                MessageBox.Show("Invalid Input from move cart <B>!! Please enter valid number.");
                return;
            }
            int PostureWrist;
            if (!int.TryParse(textBoxPostureWrist.Text, out PostureWrist))
            {
                MessageBox.Show("Invalid Input from moce cart <C>!! Please enter valid number.");
                return;
            }
            int uf;
            if (!int.TryParse(textBoxUF.Text, out uf))
            {
                MessageBox.Show("Invalid Input from move cart <B>!! Please enter valid number.");
                return;
            }
            int tf;
            if (!int.TryParse(textBoxTF.Text, out tf))
            {
                MessageBox.Show("Invalid Input from move cart <C>!! Please enter valid number.");
                return;
            }

            UInt64 result = ProcessMotionCtrl.c_mcDoGotoCartesianMove(groupId, cartX, cartY, cartZ, cartA, cartB, cartC, PostureArm, PostureElbow, PostureWrist, uf, tf);
            if (result != 0)
            {
                MessageBox.Show("group[" + groupId.ToString() + "] move to cartesian failed,ErrorCode: 0x" + result.ToString("X"));
                return;
            }
        }
    }
}
