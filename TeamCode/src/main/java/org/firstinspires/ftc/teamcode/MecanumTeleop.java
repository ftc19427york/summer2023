/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


//@Disabled

@TeleOp(name="Mecanum: Teleop", group="Mecanum")

public class MecanumTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareMecanum robot = new HardwareMecanum();   // Use a Mecanum's hardware
    double servoPosition = robot.droneARM_HOME;  //Servo's position
    final  double ARM_SPEED = 0.01;  //sets rate that servo moves increments each time button is pressed

    @Override
    public void runOpMode() {
        double x1; // left/right
        double y1; // forward/backward
        double fortyFiveInRads = -Math.PI / 4;
        double cosine45 = Math.cos(fortyFiveInRads);
        double sine45 = Math.sin(fortyFiveInRads);

        double x2=0;
        double y2=0;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double spin = gamepad1.right_stick_x;  // for controlling spin
            if (Math.abs(spin) > 0.1) {
                //if someone is moving the right joystick, spin
                robot.frontRightDrive.setPower(-spin);
                robot.backRightDrive.setPower(-spin);

                robot.frontLeftDrive.setPower(spin);
                robot.backLeftDrive.setPower(spin);
            } else {
                // if no one is pressing the right joystick, do the normal driving code

                y1 = -gamepad1.left_stick_y; //the y values are backwards
                x1 = gamepad1.left_stick_x;

                //need to rotate 45 degrees
                y2 = y1 * cosine45 + x1 * sine45;
                x2 = x1 * cosine45 - y1 * sine45;


                // Combine drive and turn for blended motion.
                //   left  = drive + turn;
                //  right = drive - turn;


                // Output the safe vales to the motor drives.++
                robot.frontLeftDrive.setPower(x2);    //motor3
                robot.backRightDrive.setPower(x2);    //motor
                robot.frontRightDrive.setPower(y2);   //motor2
                robot.backLeftDrive.setPower(y2);     //motor1


               // Use gamepad  Y & A raise lower arm
                if (gamepad1.a) // if the "a" button is pressed on gamepad, do this next line of code
                    servoPosition += ARM_SPEED;  // add to the servo position so it moves
                else if (gamepad1.y)  // if the "y" button is pressed, then do the next line of code
                    servoPosition -= ARM_SPEED;   // subtract from the servo position so it moves in the other direction

                //Move both servos to new position
                servoPosition = Range.clip(servoPosition, robot.droneARM_MIN_RANGE, robot.droneARM_MAX_RANGE); //make sure the position is valid
       //         robot.servo.setPosition(servoPosition);  // this code here actually sets the position of the servo so it moves


                // Send telemetry message to signify robot running;
                telemetry.addData("arm", "%.2f", servoPosition);  //very important code. shows the values on the screen
                telemetry.addData("x1", "%.2f", x1);
                telemetry.addData("y1", "%.2f", y1);
                telemetry.update();

                // Pace this loop so jaw action is reasonable speed.
                sleep(50);
            }
        }
    }
}