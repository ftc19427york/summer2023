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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Auto Drive By Time", group="Robot")
//@Disabled
public class AutoDriveByTime extends LinearOpMode {
    HardwareMecanum robot = new HardwareMecanum();
    static final int MOTOR_TICK_COUNT = 537;

    private ElapsedTime     runtime = new ElapsedTime();

/*
    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

*/
    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // drive forward 18 inches
        //reset the encoders
/*      robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       robot.driveWithEncode();
        //how many turns do i need the wheels to go 18 inches
        // the distance you drive with one turn of the wheel is the circumference of the wheel
        double circumference = 3.14*3.78;  //pi* diameter
        double rotationsNeeded = 18/circumference;
        int encoderDrivingTarget = (int) (rotationsNeeded*537*1.45); //  Don't need to look at the existing encoder counts
            // because it was reset above to 0.

        robot.frontLeftDrive.setTargetPosition(encoderDrivingTarget);
        robot.frontRightDrive.setTargetPosition(-encoderDrivingTarget);
        robot.backLeftDrive.setTargetPosition(-encoderDrivingTarget);
        robot.backRightDrive.setTargetPosition(encoderDrivingTarget);


        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
        robot.frontRightDrive.setPower(-.25);
        robot.backRightDrive.setPower(.25);
        robot.frontLeftDrive.setPower(.25);
        robot.backLeftDrive.setPower(-.25);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.frontLeftDrive.isBusy() ||  robot.backRightDrive.isBusy()){
            telemetry.addData("Path", "Driving 18 inches");
            telemetry.update();
        }
        robot.stopDriving();

       // wait until touch sensor is pressed
 /*       while (!robot.toucher.isPressed()){
            telemetry.addData("Path", "Waiting for touch sensor to be pressed");
            telemetry.update();
        }
        robot.stopDriving(); */

/*
        // drive forward until wall is within 5in from sensor
        while (robot.rangeFinder.getDistance(DistanceUnit.INCH) > 5)  {

            telemetry.addData("Path", "Distance: %2.5f inches", robot.rangeFinder.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
        robot.stopDriving();

        robot.driveWithoutEncode();

        robot.driveStraight(0.5, 3, telemetry, this);


        // Step 4:  Stop
        robot.stopDriving();
        robot.driveStraight(-0.5, 3, telemetry, this);
      //  robot.stopDriving();
        robot.driveRight(0.5, 1, telemetry, this);
     //   robot.stopDriving();
        robot.driveLeft(0.5, 1, telemetry, this);
        robot.driveSpinRight(0.5, 1, telemetry, this);
        robot.driveSpinLeft(0.5, 1, telemetry, this);
        robot.stopDriving();

*/      robot.driveWithEncode();
        strafeRight(48, telemetry, this);
        //Make the servo move
        robot.servo.setPosition(0);
        sleep(1000);
        robot.servo.setPosition(0.5);
        sleep(1000);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
public void strafeRight (double distanceInches, Telemetry telemetry, LinearOpMode opMode) {

    double circumference = 3.14 * 3.78;  //pi* diameter
    double rotationsNeeded = distanceInches / circumference;

    int encoderDrivingTarget = (int) (rotationsNeeded * 537 * 1.45); //  Don't need to look at the existing encoder counts
    // because it was reset above to 0.

    robot.frontLeftDrive.setTargetPosition(encoderDrivingTarget);
    robot.frontRightDrive.setTargetPosition(-encoderDrivingTarget);
    robot.backLeftDrive.setTargetPosition(-encoderDrivingTarget);
    robot.backRightDrive.setTargetPosition(encoderDrivingTarget);


    // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

    // Step 1:  Drive forward for 3 seconds
    robot.frontRightDrive.setPower(-.25);
    robot.backRightDrive.setPower(.25);
    robot.frontLeftDrive.setPower(.25);
    robot.backLeftDrive.setPower(-.25);

    robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    while (robot.frontLeftDrive.isBusy() || robot.backRightDrive.isBusy()) {
        telemetry.addData("Path", "Driving 18 inches");
        telemetry.update();
    }
    robot.stopDriving();
}
}
