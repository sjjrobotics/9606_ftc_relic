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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.lang.Math;

import org.firstinspires.ftc.robotcore.internal.android.multidex.MainDexListBuilder;

public class dumpBot extends OpMode {


    /**
     * Declare Motor Vars
     */

    private DcMotor RightBack,
            LeftBack,
            LeftFront,
            RightFront,
            Dumper,
            Mover;
    private CRServo Grabber;


@Override
    public void init() {

        LeftBack = hardwareMap.dcMotor.get("motorLeft");
        RightBack = hardwareMap.dcMotor.get("motorRight");
        Grabber = hardwareMap.crservo.get("Grabber");
        LeftFront = hardwareMap.dcMotor.get("MotorBLeft");
        RightFront = hardwareMap.dcMotor.get("MotorBRight");
        Dumper = hardwareMap.dcMotor.get("Dumper");
        Mover = hardwareMap.dcMotor.get("Mover");

    }

   /*
    * This method will be called repeatedly in a loop
    *
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
    */

    public void loop() { // When new battery is put in, robot c@Overriderosses field in about 18.3 seconds


        // note that if y equal -1 then joystick is pushed all of the way forward.
        float left = gamepad1.left_stick_y;
        float right = gamepad1.right_stick_y;
        float RightStick = gamepad2.right_stick_y;
        float LeftStick = gamepad2.left_stick_y;
        //(float) ((float) 0.5 - Range.scale(gamepad1.right_trigger, 0.0, 1.0, 0.0, 1.0)) * 2;

        // clip the right/left values so that the values never exceed +/- 1
        //right = Range.clip(right, -1, 1);
        //left = Range.clip(left, 1, -1);
        // scale the joystick value to make it easier to control

        // makes the robot more precise at slower speeds.
        //right = (float) scaleInput(right);
        //left = (float) scaleInput(left);
        // write the values to the motors
        double leftR = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double leftRobotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double leftCos = Math.cos(leftRobotAngle);
        double leftSin = Math.sin(leftRobotAngle);

        double rightR = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
        double rightRobotAngle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI / 4;
        double rightCos = Math.cos(rightRobotAngle);
        double rightSin = Math.sin(rightRobotAngle);

        double leftFront = 0;
        double leftBack = 0;
        double rightFront = 0;
        double rightBack = 0;

        if (leftCos > leftSin) {
            leftFront = leftR * leftSin/Math.abs(leftCos);
            leftBack = leftR * leftCos/Math.abs(leftCos);
        } else {
            leftFront = leftR * leftSin/Math.abs(leftSin);
            leftBack = leftR * leftCos/Math.abs(leftSin);
        }

        if (rightCos > rightSin) {
            rightFront = rightR * rightCos/Math.abs(rightCos);
            rightBack = rightR * rightSin/Math.abs(rightCos);
        } else {
            rightFront = rightR * rightCos/Math.abs(rightSin);
            rightBack = rightR * rightSin/Math.abs(rightSin);
        }



/*rightFront = rightR*rightCos;
rightBack = rightR*rightSin;
leftFront = leftR*leftSin;
leftBack = leftR*leftCos;*/

        if (rightR > 0.2) {
            RightFront.setPower(rightFront);
            RightBack.setPower(rightBack);
        } else {
            RightFront.setPower(0);
            RightBack.setPower(0);
        }

        if (leftR > 0.2) {
            LeftFront.setPower(leftFront);
            LeftBack.setPower(leftBack);
        } else {
            LeftFront.setPower(0);
            LeftBack.setPower(0);
        }
        // controls the grabbers' movement
       if (gamepad2.left_bumper) {
        Grabber.setPower(0.5);
       } else Grabber.setPower(0);
        if (gamepad2.right_bumper) {
            Grabber.setPower(-0.5);
        } else Grabber.setPower(0);
        Dumper.setPower(RightStick);
        Mover.setPower(LeftStick);
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
        LeftFront.setPower(power);
        RightFront.setPower(power);

    }

    void driveTime(double power, int time){
        if (!gamepad1.x){
            LeftFront.setPower(power);
            RightFront.setPower(power);
            waiting(time);
        }
        LeftFront.setPower(0);
        RightFront.setPower(0);

    }

    void waiting(long time) {
        int count = 0;
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
        if (gamepad1.x){
            if (direction == 1) {
                LeftFront.setPower(power);
                RightFront.setPower(-power);
                waiting(time);
                LeftFront.setPower(0);
                RightFront.setPower(0);
            }
            else if (direction == -1) {
                LeftFront.setPower(-power);
                RightFront.setPower(power);
                waiting(time);
                LeftFront.setPower(0);
                RightFront.setPower(0);
            }
        }
    }

}
