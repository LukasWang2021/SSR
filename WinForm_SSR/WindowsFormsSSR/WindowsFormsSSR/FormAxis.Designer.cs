namespace WindowsFormsSSR
{
    partial class FormAxis
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.button7 = new System.Windows.Forms.Button();
            this.label11 = new System.Windows.Forms.Label();
            this.textBoxPosRelative = new System.Windows.Forms.TextBox();
            this.button6 = new System.Windows.Forms.Button();
            this.label6 = new System.Windows.Forms.Label();
            this.textBoxJerk1 = new System.Windows.Forms.TextBox();
            this.label4 = new System.Windows.Forms.Label();
            this.textBoxAcc1 = new System.Windows.Forms.TextBox();
            this.label3 = new System.Windows.Forms.Label();
            this.textBoxVel1 = new System.Windows.Forms.TextBox();
            this.label2 = new System.Windows.Forms.Label();
            this.textBoxPosAbs = new System.Windows.Forms.TextBox();
            this.button5 = new System.Windows.Forms.Button();
            this.textBoxOffsetPos = new System.Windows.Forms.TextBox();
            this.button4 = new System.Windows.Forms.Button();
            this.button3 = new System.Windows.Forms.Button();
            this.button2 = new System.Windows.Forms.Button();
            this.button1 = new System.Windows.Forms.Button();
            this.comboBoxAxisID = new System.Windows.Forms.ComboBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.textBoxJerk2 = new System.Windows.Forms.TextBox();
            this.label7 = new System.Windows.Forms.Label();
            this.textBoxAcc2 = new System.Windows.Forms.TextBox();
            this.label8 = new System.Windows.Forms.Label();
            this.textBoxVel2 = new System.Windows.Forms.TextBox();
            this.SuspendLayout();
            // 
            // button7
            // 
            this.button7.Location = new System.Drawing.Point(60, 299);
            this.button7.Name = "button7";
            this.button7.Size = new System.Drawing.Size(90, 23);
            this.button7.TabIndex = 56;
            this.button7.Text = "StopMotion";
            this.button7.UseVisualStyleBackColor = true;
            this.button7.Click += new System.EventHandler(this.button7_Click);
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(170, 232);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(77, 12);
            this.label11.TabIndex = 55;
            this.label11.Text = "relative_pos";
            // 
            // textBoxPosRelative
            // 
            this.textBoxPosRelative.Location = new System.Drawing.Point(172, 247);
            this.textBoxPosRelative.Name = "textBoxPosRelative";
            this.textBoxPosRelative.Size = new System.Drawing.Size(70, 21);
            this.textBoxPosRelative.TabIndex = 54;
            this.textBoxPosRelative.Text = "0";
            // 
            // button6
            // 
            this.button6.Location = new System.Drawing.Point(60, 245);
            this.button6.Name = "button6";
            this.button6.Size = new System.Drawing.Size(90, 23);
            this.button6.TabIndex = 53;
            this.button6.Text = "MoveRelative";
            this.button6.UseVisualStyleBackColor = true;
            this.button6.Click += new System.EventHandler(this.button6_Click);
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(456, 163);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(29, 12);
            this.label6.TabIndex = 52;
            this.label6.Text = "jerk";
            // 
            // textBoxJerk1
            // 
            this.textBoxJerk1.Location = new System.Drawing.Point(458, 178);
            this.textBoxJerk1.Name = "textBoxJerk1";
            this.textBoxJerk1.Size = new System.Drawing.Size(70, 21);
            this.textBoxJerk1.TabIndex = 51;
            this.textBoxJerk1.Text = "10000";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(366, 163);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(23, 12);
            this.label4.TabIndex = 50;
            this.label4.Text = "acc";
            // 
            // textBoxAcc1
            // 
            this.textBoxAcc1.Location = new System.Drawing.Point(368, 178);
            this.textBoxAcc1.Name = "textBoxAcc1";
            this.textBoxAcc1.Size = new System.Drawing.Size(70, 21);
            this.textBoxAcc1.TabIndex = 49;
            this.textBoxAcc1.Text = "1000";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(274, 163);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(23, 12);
            this.label3.TabIndex = 48;
            this.label3.Text = "vel";
            // 
            // textBoxVel1
            // 
            this.textBoxVel1.Location = new System.Drawing.Point(276, 178);
            this.textBoxVel1.Name = "textBoxVel1";
            this.textBoxVel1.Size = new System.Drawing.Size(70, 21);
            this.textBoxVel1.TabIndex = 47;
            this.textBoxVel1.Text = "100";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(170, 163);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(77, 12);
            this.label2.TabIndex = 46;
            this.label2.Text = "absolute_pos";
            // 
            // textBoxPosAbs
            // 
            this.textBoxPosAbs.Location = new System.Drawing.Point(172, 178);
            this.textBoxPosAbs.Name = "textBoxPosAbs";
            this.textBoxPosAbs.Size = new System.Drawing.Size(70, 21);
            this.textBoxPosAbs.TabIndex = 45;
            this.textBoxPosAbs.Text = "0";
            // 
            // button5
            // 
            this.button5.Location = new System.Drawing.Point(60, 176);
            this.button5.Name = "button5";
            this.button5.Size = new System.Drawing.Size(90, 23);
            this.button5.TabIndex = 44;
            this.button5.Text = "MoveAbsolute";
            this.button5.UseVisualStyleBackColor = true;
            this.button5.Click += new System.EventHandler(this.button5_Click);
            // 
            // textBoxOffsetPos
            // 
            this.textBoxOffsetPos.Location = new System.Drawing.Point(172, 103);
            this.textBoxOffsetPos.Name = "textBoxOffsetPos";
            this.textBoxOffsetPos.Size = new System.Drawing.Size(70, 21);
            this.textBoxOffsetPos.TabIndex = 43;
            this.textBoxOffsetPos.Text = "0";
            // 
            // button4
            // 
            this.button4.Location = new System.Drawing.Point(60, 101);
            this.button4.Name = "button4";
            this.button4.Size = new System.Drawing.Size(82, 23);
            this.button4.TabIndex = 42;
            this.button4.Text = "SetPosition";
            this.button4.UseVisualStyleBackColor = true;
            this.button4.Click += new System.EventHandler(this.button4_Click);
            // 
            // button3
            // 
            this.button3.Location = new System.Drawing.Point(500, 42);
            this.button3.Name = "button3";
            this.button3.Size = new System.Drawing.Size(75, 23);
            this.button3.TabIndex = 41;
            this.button3.Text = "Disable";
            this.button3.UseVisualStyleBackColor = true;
            this.button3.Click += new System.EventHandler(this.button3_Click);
            // 
            // button2
            // 
            this.button2.Location = new System.Drawing.Point(391, 42);
            this.button2.Name = "button2";
            this.button2.Size = new System.Drawing.Size(75, 23);
            this.button2.TabIndex = 40;
            this.button2.Text = "Enable";
            this.button2.UseVisualStyleBackColor = true;
            this.button2.Click += new System.EventHandler(this.button2_Click);
            // 
            // button1
            // 
            this.button1.Location = new System.Drawing.Point(281, 42);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(75, 23);
            this.button1.TabIndex = 39;
            this.button1.Text = "ResetError";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click);
            // 
            // comboBoxAxisID
            // 
            this.comboBoxAxisID.FormattingEnabled = true;
            this.comboBoxAxisID.Items.AddRange(new object[] {
            "1",
            "2",
            "3",
            "4",
            "5",
            "6",
            "7",
            "8",
            "9",
            "10",
            "11",
            "12",
            "13",
            "14",
            "15",
            "16"});
            this.comboBoxAxisID.Location = new System.Drawing.Point(172, 44);
            this.comboBoxAxisID.Name = "comboBoxAxisID";
            this.comboBoxAxisID.Size = new System.Drawing.Size(54, 20);
            this.comboBoxAxisID.TabIndex = 38;
            this.comboBoxAxisID.Text = "1";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(82, 45);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(47, 12);
            this.label1.TabIndex = 37;
            this.label1.Text = "Axis_ID";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(455, 232);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(29, 12);
            this.label5.TabIndex = 63;
            this.label5.Text = "jerk";
            // 
            // textBoxJerk2
            // 
            this.textBoxJerk2.Location = new System.Drawing.Point(457, 247);
            this.textBoxJerk2.Name = "textBoxJerk2";
            this.textBoxJerk2.Size = new System.Drawing.Size(70, 21);
            this.textBoxJerk2.TabIndex = 62;
            this.textBoxJerk2.Text = "10000";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(365, 232);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(23, 12);
            this.label7.TabIndex = 61;
            this.label7.Text = "acc";
            // 
            // textBoxAcc2
            // 
            this.textBoxAcc2.Location = new System.Drawing.Point(367, 247);
            this.textBoxAcc2.Name = "textBoxAcc2";
            this.textBoxAcc2.Size = new System.Drawing.Size(70, 21);
            this.textBoxAcc2.TabIndex = 60;
            this.textBoxAcc2.Text = "1000";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(273, 232);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(23, 12);
            this.label8.TabIndex = 59;
            this.label8.Text = "vel";
            // 
            // textBoxVel2
            // 
            this.textBoxVel2.Location = new System.Drawing.Point(275, 247);
            this.textBoxVel2.Name = "textBoxVel2";
            this.textBoxVel2.Size = new System.Drawing.Size(70, 21);
            this.textBoxVel2.TabIndex = 58;
            this.textBoxVel2.Text = "100";
            // 
            // FormAxis
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(800, 450);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.textBoxJerk2);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.textBoxAcc2);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.textBoxVel2);
            this.Controls.Add(this.button7);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.textBoxPosRelative);
            this.Controls.Add(this.button6);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.textBoxJerk1);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.textBoxAcc1);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.textBoxVel1);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.textBoxPosAbs);
            this.Controls.Add(this.button5);
            this.Controls.Add(this.textBoxOffsetPos);
            this.Controls.Add(this.button4);
            this.Controls.Add(this.button3);
            this.Controls.Add(this.button2);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.comboBoxAxisID);
            this.Controls.Add(this.label1);
            this.Name = "FormAxis";
            this.Text = "FormAxis";
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion
        private System.Windows.Forms.Button button7;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.TextBox textBoxPosRelative;
        private System.Windows.Forms.Button button6;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.TextBox textBoxJerk1;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TextBox textBoxAcc1;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.TextBox textBoxVel1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.TextBox textBoxPosAbs;
        private System.Windows.Forms.Button button5;
        private System.Windows.Forms.TextBox textBoxOffsetPos;
        private System.Windows.Forms.Button button4;
        private System.Windows.Forms.Button button3;
        private System.Windows.Forms.Button button2;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.ComboBox comboBoxAxisID;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.TextBox textBoxJerk2;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.TextBox textBoxAcc2;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.TextBox textBoxVel2;
    }
}