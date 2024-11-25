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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp()

public class MecanumTeleop_FieldCentric extends LinearOpMode {


    HardwareMecanum robot = new HardwareMecanum();   // Use a Mecanum's hardware
    double dronePosition = robot.droneARM_HOME;  // servo's position
    final double droneARM_SPEED = 0.10;  // set rate to move servo
    double twistPosition = robot.twistClawARM_HOME;  // servo's position
    final double twistARM_SPEED = 0.005;  // set rate to move servo

    double clawPosition = robot.clawARM_HOME;  // servo's position
    final double clawARM_SPEED = 0.005;  // set rate to move servo

    static final double COUNTS_PER_MOTOR_REV = 537.7;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public BNO055IMU imu;

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        // int newTarget=robot.lift.getTargetPosition() + (int) COUNTS_PER_MOTOR_REV*2;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Defines the parameters for the gyro (units)
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuParameters.loggingEnabled = true;
        imuParameters.loggingTag = "IMU";
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Intializes the parameters previously defined
        imu.initialize(imuParameters);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.twistClaw.setPosition(0.28);
        robot.tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /**gets the angle from the imu**/
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
            double angle = angles.firstAngle;

            /**gets squared values from the driver's stick input**/
            double r = Math.hypot(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
            /**finds the desired angle that the driver wants to move the robot**/
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            /**sets the movement angle by finding the difference of the robots angle, the input angle and the offset value
             * the offset value is set by the the driver if the imu does not reset after auto*/
            robotAngle = robotAngle - Math.toRadians(angle);

            double rightX = gamepad1.right_stick_x;

            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            // Output the safe vales to the motor drives.++
            robot.frontLeftDrive.setPower(v1);    //motor3
            robot.backRightDrive.setPower(v4);    //motor
            robot.frontRightDrive.setPower(v2);   //motor2
            robot.backLeftDrive.setPower(v3);     //motor1


            if (gamepad1.right_trigger > 0.5) { // if the "b" is pressed on gamepad do this next line of code
                robot.drone.setPosition(0);  //this code here actually sets the position of the servo so it moves
                sleep(500);
                robot.drone.setPosition(.5);  //this code here actually sets the position of the servo so it moves
            }

            if (gamepad2.dpad_up == true) {
                twistPosition += twistARM_SPEED; // a position os it movesdd to the servo
                robot.twistClaw.setPosition(twistPosition);  //this code here actually sets the position of the servo so it moves
            }


            if (gamepad2.dpad_down == true) { //if the "y" button is pressed then do the next line of code
                twistPosition -= twistARM_SPEED;   //substract from the servo position so it moves in the other direction
                robot.twistClaw.setPosition(twistPosition);  //this code here actually sets the position of the servo so it moves
            }
            //    dronePosition = Range.clip(dronePosition, robot.ARM_MIN_RANGE, robot.ARM_MAX_RANGE); // make sure the position is valid

            if (gamepad2.y == true) { // if the "a" is pressed on gamepad do this next line of code
                robot.lift.setPower(1);  //this code here actually sets the position of the servo so it moves
                robot.lift.setTargetPosition(1000);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad2.right_bumper == true) { // if the "a" is pressed on gamepad do this next line of code
                robot.lift.setPower(1);  //this code here actually sets the position of the servo so it moves
                robot.lift.setTargetPosition(1800);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad2.a == true) {
                robot.lift.setPower(.75);  //this code here actually sets the position of the servo so it moves
                robot.lift.setTargetPosition(30);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(gamepad1.right_trigger > 0.5) {
                robot.lift.setPower(1.0);
            } else if (gamepad1.left_trigger > 0.5) {
                robot.lift.setPower(-1.0);
            } else {
                robot.lift.setPower(0);

            }
                       }




                if (gamepad2.x == true && gamepad2.b == false) { // if the "a" is pressed on gamepad do this next line of code
                    robot.tilt.setPower(-1);
                } //this code here actually sets the position of the servo so it moves
                if (gamepad2.x == false && gamepad2.b == false) { // if the "a" is pressed on gamepad do this next line of code
                    robot.tilt.setPower(0);
                    //  robot.tilt.setTargetPosition(9000);
                    //  robot.tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if (gamepad2.b == true && gamepad2.x == false) {
                    robot.tilt.setPower(1);  //this code here actually sets the position of the servo so it moves
                }
                if (gamepad2.x == false && gamepad2.b == false) { // if the "a" is pressed on gamepad do this next line of code
                    robot.tilt.setPower(0);
                    //  robot.tilt.setTargetPosition(0);
                    //  robot.tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if (gamepad2.right_trigger > 0.5) { // if the "b" is pressed on gamepad do this next line of code
                    //robot.twistClaw.setPosition(0);  //this code here actually sets the position of the servo so it moves
                    // sleep(500);
                    robot.claw.setPosition(.15);  //this code here actually sets the position of the servo so it moves
                    clawPosition += clawARM_SPEED; // a position os it movesdd to the servo

                }
                if (gamepad2.left_trigger > 0.5) { // if the "b" is pressed on gamepad do this next line of code
                    robot.claw.setPosition(0);  //this code here actually sets the position of the servo so it moves
                    clawPosition -= clawARM_SPEED; // a position os it movesdd to the servo
                }

                telemetry.addData("drone", "%.2f", dronePosition);
                telemetry.addData("twist", "%.2f", twistPosition);
                telemetry.addData("claw", "%.2f", clawPosition);


                telemetry.update();

                // Pace this loop so jaw action is reasonable speed.
                //  sleep(50);
            }
        }
    }
