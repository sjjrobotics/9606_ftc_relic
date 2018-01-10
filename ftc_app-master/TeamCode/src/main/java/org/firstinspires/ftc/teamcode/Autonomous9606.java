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

 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.CRServo;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.Servo;


 public class Autonomous9606 extends LinearOpMode {

     private DcMotor motorLeft,
             liftmotor,
             motorRight,
             armotor;
     private Servo grabberLeft,
             grabberRight,
             relicsgrabber;
     private CRServo Armservo;
     private double armLocation;
     int gamemode;
     int count;


     @Override
     public void runOpMode() throws InterruptedException {
         motorLeft = hardwareMap.dcMotor.get("motorLeft");
         motorRight = hardwareMap.dcMotor.get("motorRight");
         liftmotor = hardwareMap.dcMotor.get("liftmotor");
         grabberRight = hardwareMap.servo.get("grabberight");
         armotor = hardwareMap.dcMotor.get("armotor");
         Armservo = hardwareMap.crservo.get("armservo");
         grabberLeft = hardwareMap.servo.get("grabberleft");
         relicsgrabber = hardwareMap.servo.get("relicsgrabber");


         motorLeft.setDirection(DcMotor.Direction.REVERSE);

         while (!opModeIsActive())

             // First and second variables are power and time (ms)

             waiting(100); // waits for a tenth of a second so we don't do too many things too fast

         drivetime(1, 3000); // Drive forward first time

         waiting(100);

         // The last variable is the direction, 1 for right, -1 for left
         turn(1, 100, 1); // Turns right to face le bin

         waiting(100);

         drivetime(1, 100); // moves into posititon

         waiting(100);

         //turn(1, 1500, -1); // Turns straight to aim at bin

         waiting(100);

         drivetime(0.5, 100);

         waiting(100);



         //largeArm(1, 2350); // Dumps climbers into bin


//        waiting(100);
//
//        drivetime(-1, 3);
//
//        waiting(100);
//
//        turn(1, 2000, -1);
//
//        waiting(100);
//
//        drivetime(1, 2000);
//
//        waiting(100);
//
//        smallBar(-1, 1000);
//
//        waiting(100);
//
//        drivetime(1, 2000);
//
//        waiting(100);
//
//        smallBar(1, 1700);
//
     }


     /*public boolean opModeIsActive() {
         return super.opModeIsActive();
     }*/


     void drivetime(double power, int time) {
         if (!gamepad1.x) {
             motorLeft.setPower(power);
             motorRight.setPower(power);
             waiting(time);
             motorLeft.setPower(0);
             motorRight.setPower(0);
         }
     }

     void waiting(long time) {
         count = 0;
         while (count < time && !gamepad1.x && opModeIsActive()) {
             try {
                 Thread.sleep(50);
             } catch (InterruptedException e) {
                 e.printStackTrace();
             }
             count = count + 50;
         }
     }

     void turn(double power, int time, int direction) {
         if (!gamepad1.x) {
             if (direction == 1) {
                 motorLeft.setPower(power);
                 motorRight.setPower(-power);
                 waiting(time);
                 motorLeft.setPower(0);
                 motorRight.setPower(0);
             } else if (direction == -1) {
                 motorLeft.setPower(-power);
                 motorRight.setPower(power);
                 waiting(time);
                 motorLeft.setPower(0);
                 motorRight.setPower(0);
             }
         }
     }

 }















