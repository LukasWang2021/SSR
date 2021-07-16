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
    public partial class FormMain : Form
    {
        private bool isValid = false;
        public FormMain()
        {
            InitializeComponent();
            this.IsMdiContainer = true;
        }

        private void axisToolStripMenuItem_Click(object sender, EventArgs e)
        {
            if (!isValid)
            {
                MessageBox.Show("Please connect first!!!");
                return;
            }
            FormAxis form = new FormAxis();
            form.MdiParent = this;
            form.Show();
        }

        private void groupToolStripMenuItem_Click(object sender, EventArgs e)
        {
            if (!isValid)
            {
                MessageBox.Show("Please connect first!!!");
                return;
            }
            FormGroup form = new FormGroup();
            form.MdiParent = this;
            form.Show();
        }

        private void button1_Click(object sender, EventArgs e)
        {
            UInt64 result = ProcessMotionCtrl.StartConnection(textBoxIP.Text);
            if (result != 0)
            {
                MessageBox.Show("Connect failed: 0x" + result.ToString("X"));
                isValid = false;
                return;
            }
            isValid = true;
            MessageBox.Show("Connect success.");
        }
    }
}
