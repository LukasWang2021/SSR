using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using ProcessMotion;

namespace WindowsFormsSSR
{
    public partial class FormAxis : Form
    {
        public FormAxis()
        {
            InitializeComponent();
        }

        private Int32 getAxisId()
        {
            Int32 axisId;
            if (!int.TryParse(comboBoxAxisID.SelectedItem.ToString(), out axisId))
            {
                MessageBox.Show("Invalid Input from <Axis_ID>!! Please enter valid number.");
                return -1;
            }
            return axisId;
        }

        private void button4_Click(object sender, EventArgs e)
        {
            Int32 axisId = getAxisId();
            if (axisId < 0) return;

            double pos;
            if (!double.TryParse(textBoxOffsetPos.Text, out pos))
            {
                MessageBox.Show("Invalid Input from <SetPosition_pos>!! Please enter valid number.");
                return;
            }

            UInt64 result = ProcessMotionCtrl.c_axisSetPosition(axisId, pos);
            if (result != 0)
            {
                MessageBox.Show("axis[" + axisId.ToString() + "] set offset failed,ErrorCode: 0x" + result.ToString("X"));
                return;
            }
        }

        private void button5_Click(object sender, EventArgs e)
        {
            Int32 axisId = getAxisId();
            if (axisId < 0) return;

            double pos;
            if (!double.TryParse(textBoxPosAbs.Text, out pos))
            {
                MessageBox.Show("Invalid Input from MoveAbs <pos>!! Please enter valid number.");
                return;
            }

            double vel;
            if (!double.TryParse(textBoxVel1.Text, out vel))
            {
                MessageBox.Show("Invalid Input from SetMotion <vel>!! Please enter valid number.");
                return;
            }
            double acc;
            if (!double.TryParse(textBoxAcc1.Text, out acc))
            {
                MessageBox.Show("Invalid Input from SetMotion <acc>!! Please enter valid number.");
                return;
            }
            double jerk;
            if (!double.TryParse(textBoxJerk1.Text, out jerk))
            {
                MessageBox.Show("Invalid Input from SetMotion <jerk>!! Please enter valid number.");
                return;
            }

            UInt64 result = ProcessMotionCtrl.c_axisMoveAbsolute(axisId, pos, vel, acc, acc, jerk);
            if (result != 0)
            {
                MessageBox.Show("axis[" + axisId.ToString() + "] move absolute failed,ErrorCode: 0x" + result.ToString("X"));
                return;
            }
        }

        private void button1_Click(object sender, EventArgs e)
        {
            Int32 axisId = getAxisId();
            if (axisId < 0) return;

            UInt64 result = ProcessMotionCtrl.c_axisReset(axisId);
            if (result != 0)
            {
                MessageBox.Show("axis[" + axisId.ToString() + "] reset error failed,ErrorCode: 0x" + result.ToString("X"));
                return;
            }
        }

        private void button2_Click(object sender, EventArgs e)
        {
            Int32 axisId = getAxisId();
            if (axisId < 0) return;

            UInt64 result = ProcessMotionCtrl.c_axisPower(axisId, 1);
            if (result != 0)
            {
                MessageBox.Show("axis[" + axisId.ToString() + "] power on failed,ErrorCode: 0x" + result.ToString("X"));
                return;
            }
        }

        private void button3_Click(object sender, EventArgs e)
        {
            Int32 axisId = getAxisId();
            if (axisId < 0) return;

            UInt64 result = ProcessMotionCtrl.c_axisPower(axisId, 0);
            if (result != 0)
            {
                MessageBox.Show("axis[" + axisId.ToString() + "] power off failed,ErrorCode: 0x" + result.ToString("X"));
                return;
            }
        }

        private void button6_Click(object sender, EventArgs e)
        {

            Int32 axisId = getAxisId();
            if (axisId < 0) return;

            double pos;
            if (!double.TryParse(textBoxPosAbs.Text, out pos))
            {
                MessageBox.Show("Invalid Input from MoveAbs <pos>!! Please enter valid number.");
                return;
            }

            double vel;
            if (!double.TryParse(textBoxVel2.Text, out vel))
            {
                MessageBox.Show("Invalid Input from SetMotion <vel>!! Please enter valid number.");
                return;
            }
            double acc;
            if (!double.TryParse(textBoxAcc2.Text, out acc))
            {
                MessageBox.Show("Invalid Input from SetMotion <acc>!! Please enter valid number.");
                return;
            }
            double jerk;
            if (!double.TryParse(textBoxJerk2.Text, out jerk))
            {
                MessageBox.Show("Invalid Input from SetMotion <jerk>!! Please enter valid number.");
                return;
            }

            UInt64 result = ProcessMotionCtrl.c_axisMoveAbsolute(axisId, pos, vel, acc, acc, jerk);
            if (result != 0)
            {
                MessageBox.Show("axis[" + axisId.ToString() + "] move absolute failed,ErrorCode: 0x" + result.ToString("X"));
                return;
            }
        }

        private void button7_Click(object sender, EventArgs e)
        {
            Int32 axisId = getAxisId();
            if (axisId < 0) return;

            UInt64 result = ProcessMotionCtrl.c_axisStop(axisId);
            if (result != 0)
            {
                MessageBox.Show("axis[" + axisId.ToString() + "] stop motion failed,ErrorCode: 0x" + result.ToString("X"));
                return;
            }
        }
        
    }
}
