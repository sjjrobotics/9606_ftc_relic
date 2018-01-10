/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


public class SoccerBot extends OpMode {



    DcMotor motorRight;
    DcMotor motorLeft;
    int count;
    boolean rightGood = false;
    boolean singleOnly = true;

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    boolean switchPressed = false;
    boolean switchOn = false;
    @Override
    public void init() {

        motorLeft = hardwareMap.dcMotor.get("left");
        motorRight = hardwareMap.dcMotor.get("right");

    }

   /*
    * This method will be called repeatedly in a loop
    *
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
    */

    @Override
    public void loop() { // When new battery is put in, robot crosses field in about 18.3 seconds

        /*if (gamepad2.dpad_left) {
            grabberRight.setPosition(0.85);
            grabberLeft.setPosition(0);
        }
        if (gamepad2.dpad_right) {
            grabberRight.setPosition(0.25);
            grabberLeft.setPosition(0.5);
        }*/

        // note that if y equal -1 then joystick is pushed all of the way forward.
        float left = gamepad1.left_stick_y;
        float right = (float) ((float) 0.5 - Range.scale(gamepad1.right_trigger, 0.0, 1.0, 0.0, 1.0));
        right = right * 2;
        // clip the right/left values so that the values never exceed +/- 1


        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        // scale the joystick value to make it easier to control
        // makes the robot more precise at slower speeds.
        right = (float) scaleInput(right);
        left = (float) scaleInput(left);

        // write the values to the motors

        motorRight.setPower(right);
        motorLeft.setPower(left);

        if (gamepad1.dpad_up) {
            drive(1);
        } else if (gamepad1.dpad_down) {
            drive(-1);
        }


    }

//

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
//    double dScale = 0.0;
        double dScale;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

    void drive(double power)
    {
        motorLeft.setPower(power);
        motorRight.setPower(power);

    }

    void driveTime(double power, int time){
        if (!gamepad1.x){
            motorLeft.setPower(power);
            motorRight.setPower(power);
            waiting(time);
            motorLeft.setPower(0);
            motorRight.setPower(0);
        }
    }

    void waiting(long time) {
        count = 0;
        while (count < time && !gamepad1.x) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            count = count + 100;
        }
    }

    void turn(double power, int time, int direction){
        if (!gamepad1.x){
            if (direction == 1) {
                motorLeft.setPower(power);
                motorRight.setPower(-power);
                waiting(time);
                motorLeft.setPower(0);
                motorRight.setPower(0);
            }
            else if (direction == -1) {
                motorLeft.setPower(-power);
                motorRight.setPower(power);
                waiting(time);
                motorLeft.setPower(0);
                motorRight.setPower(0);
            }
        }
    }



}
