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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareMecanum
{
    /* Public OpMode members. */
    public wheels drivetrain;
    public DcMotor  frontLeftDrive   = null;
    public DcMotor  frontRightDrive  = null;
    public DcMotor  backLeftDrive     = null;
    public DcMotor  backRightDrive     = null;
    public DcMotor  lift     = null;
    public DcMotor  tilt     = null;
    public Servo drone = null;
    public Servo claw = null;
    public Servo twistClaw = null;

    //  public DistanceSensor rangeFinder = null;
  //  public TouchSensor toucher = null;
 //   public RevColorSensorV3 frontColor = null;

    public final static double droneARM_HOME = 0.5; // Starting point for Servo Arm
    public final static double clawARM_HOME = 0.15; // Starting point for Servo Arm

    public final static double twistClawARM_HOME = 0.6; // Starting point for Servo Arm

    public final static double droneARM_MIN_RANGE = 0.0;  //Smallest number value allowed for servo position
    public final static double droneARM_MAX_RANGE = 1.0;  //Largest number allowed for servo position


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMecanum(){

    }

        /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftDrive  = hwMap.get(DcMotor.class, "motor3");
        frontRightDrive = hwMap.get(DcMotor.class, "motor2");
        backLeftDrive    = hwMap.get(DcMotor.class, "motor1");
        backRightDrive    = hwMap.get(DcMotor.class, "motor");

        drivetrain = new wheels(frontLeftDrive, frontRightDrive);
        drivetrain.hello();

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE); // motor3
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);// motor2
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE); // motor1
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);// motor

        lift    = hwMap.get(DcMotor.class, "motor4");
        tilt    = hwMap.get(DcMotor.class, "motor5");

        lift.setDirection(DcMotor.Direction.REVERSE); // motor4
        tilt.setDirection(DcMotor.Direction.FORWARD);// motor5
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Set all motors to zero power
        this.stopDriving();



        // Define and initialize ALL installed servos
        drone=hwMap.servo.get("servo2"); // set equal to name of the servo motor in DS
        drone.setPosition(droneARM_HOME); //setPosition actually sets the servo's position and move it
        claw=hwMap.servo.get("servo1"); // set equal to name of the servo motor in DS
        claw.setPosition(clawARM_HOME); //setPosition actually sets the servo's position and move it
        twistClaw=hwMap.servo.get("servo"); // set equal to name of the servo motor in DS
        twistClaw.setPosition(twistClawARM_HOME); //setPosition actually sets the servo's position and move it


   //     rangeFinder = hwMap.get(DistanceSensor.class, "sensor_distance");
    //    toucher = hwMap.get(TouchSensor.class, "touch");
    //   frontColor = hwMap.get(RevColorSensorV3.class, "sensor_color");
    }
    public void driveWithoutEncode() {
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void driveWithEncode() {
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void stopResetEncode() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runToPosition() {
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void stopDriving() {
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    public void driveStraight(double power, float seconds, Telemetry telemetry){
        frontRightDrive.setPower(power);
        backRightDrive.setPower(power);
        frontLeftDrive.setPower(power);
        backLeftDrive.setPower(power);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset(); //starts the time at 0
        while (runtime.seconds() < seconds) {
           //print time to DS
            telemetry.addData("Path", "driveStraight: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    public void driveRight(double power, float seconds, Telemetry telemetry, LinearOpMode opMode){
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        backLeftDrive.setPower(-power);
        backRightDrive.setPower(power);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset(); //starts the time at 0
        while (opMode.opModeIsActive() &&  (runtime.seconds() < seconds)) {
            //print time to DS
            telemetry.addData("Path", "driveStraight: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void driveLeft(double power, float seconds, Telemetry telemetry, LinearOpMode opMode){
        frontLeftDrive.setPower(-power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(-power);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset(); //starts the time at 0
        while (opMode.opModeIsActive() &&  (runtime.seconds() < seconds)) {
            //print time to DS
            telemetry.addData("Path", "driveStraight: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    public void driveSpinRight(double power, float seconds, Telemetry telemetry, LinearOpMode opMode){
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(-power);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset(); //starts the time at 0
        while (opMode.opModeIsActive() &&  (runtime.seconds() < seconds)) {
            //print time to DS
            telemetry.addData("Path", "driveStraight: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void driveSpinLeft(double power, float seconds, Telemetry telemetry, LinearOpMode opMode){
        frontLeftDrive.setPower(-power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(-power);
        backRightDrive.setPower(power);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset(); //starts the time at 0
        while (opMode.opModeIsActive() &&  (runtime.seconds() < seconds)) {
            //print time to DS
            telemetry.addData("Path", "driveStraight: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

}
