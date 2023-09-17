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
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Gyroscope;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Blue_Right_Final", group="competition")
//@Disabled
public class Final_Blue_Right extends LinearOpMode {

    /* Declare OpMode members. */
    Robot_Hardware         robot   = new Robot_Hardware();   // Use a Pushbot's hardware

    private ElapsedTime     runtime = new ElapsedTime();
    Orientation angle = new Orientation();
    Orientation lastAngles = new Orientation();

    BNO055IMU               imu;
    static final double     COUNTS_PER_MOTOR_REV    = 512 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/Hat2.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Hat";

    private static final String VUFORIA_KEY =
            "Af7vkp7/////AAABmUk12D74IES9l/tUK8PAUUJJ6ByVCr2iazSnU0dTZSxqOyAQYobkrcrY+wtkBFD3QefJP6JAbrkWiI7CmrkJYUEOpAlZxO5oRckCXOLoDMSG/+VGualBuF/H4/VpWHjc3GLOQueZrhcgvAhcPJ1WMkD8KTAs8clyUNbyMpUPS6Xih/q1K9spVmJz+DiCpE2o+3kQInHZ03YL9KzzKhslFVemKt6WR/68q94zJVjFccJvxlvyoFYtKR045iO9N5RQk5lvrX+58V5MOziSkf1wx6BNnOco98XAx7ySxNtRjPgYh5tG3p+PYXuCEvFNs+456PXk0fKuve7iHh70hv0k38mf8cVRkvJhSPnwA9N7UjSm";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    int size;
    double left;
    Recognition recognition;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftFront.getCurrentPosition(),
                robot.rightFront.getCurrentPosition(),
                robot.leftBack.getCurrentPosition(),
                robot.rightBack.getCurrentPosition());
        telemetry.update();
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.1, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        hub(0);
        waitForStart();
        if (opModeIsActive()) {

            for (int count = 0; count < 1; count++) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        size = updatedRecognitions.size();
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            left = recognition.getLeft();
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            i++;
                        }
                        telemetry.update();
                    }
                }
            }
            if (size == 0) {
                hub(0);//sets the arm height
                gyro(250, 42, 10, 40);//moves towards hub  orig 800 37 10 50
                sleep(1000);
                encoderDrive(1250, 10, 0, 0, 0.5);//turns around orig:1 last
                //hub(1);//sets arm to lowest position for the hub
                //sleep(2000);//waits for one second
                //hub(0);//sets arm to lowest position for the hub
                //Drives to carousel to spin duck and staf to storage space
                //gyro(900, -115, 10, 50);
                sleep(1000);
                encoderDrive(950, 10, 0, 0, 1);
                sleep(1000);
                encoderDrive(1100, 10, 0, -0.5, 0.25);  //500, 10, 0, -1, 0.1
                spin(6);  //spins carousel
                encoderDrive(1000, 25, 1, 0.25, 0.05); //straf to stoarge
                //sleep(1000);
                //encoderDrive(150, 10, 0, -1, 0.05); //200
            }
            if (size == 1&&left<100) {
                hub(0);//sets the arm height
                gyro(250, 42, 10, 40);//moves towards hub  orig 800 37 10 50
                sleep(1000);
                encoderDrive(1250, 10, 0, 0, 0.5);//turns around orig:1 last
                //hub(2);//sets arm to lowest position for the hub
                //sleep(2000);//waits for one second
                //hub(0);//sets arm to lowest position for the hub
                //Drives to carousel to spin duck and staf to storage space
                //gyro(900, -115, 10, 50);
                sleep(1000);
                encoderDrive(950, 10, 0, 0, 1);
                sleep(1000);
                encoderDrive(1125, 10, 0, -0.5, 0.15);  //500, 10, 0, -1, 0.1
                spin(6);  //spins carousel
                encoderDrive(1000, 25, 1, 0.25, 0.05); //straf to stoarge
                //sleep(1000);
                //encoderDrive(150, 10, 0, -1, 0.05); //200
            }
            if (size == 1&&left>100){
                hub(0);//sets the arm height
                gyro(250, 42, 10, 40);//moves towards hub  orig 800 37 10 50
                sleep(1000);
                encoderDrive(1250, 10, 0, 0, 0.5);//turns around orig:1 last
                //hub(3);//sets arm to lowest position for the hub
                //sleep(2000);//waits for one second
                //hub(0);//sets arm to lowest position for the hub
                //Drives to carousel to spin duck and staf to storage space
                //gyro(900, -115, 10, 50);
                sleep(1000);
                encoderDrive(950, 10, 0, 0, 1);
                sleep(1000);
                encoderDrive(1125, 10, 0, -0.5, 0.15);  //500, 10, 0, -1, 0.1
                spin(6);  //spins carousel
                encoderDrive(1000, 25, 1, 0.3, 0.05); //straf to stoarge
                //sleep(1000);
                //encoderDrive(150, 10, 0, -1, 0.05); //200*/
            }
        }


        // 250 equals 1 tile
        //gyro(30000, 10, 50);
        //steering(1000, 20, 10, 50);
        //encoderDrive(24, 10, 0, 1, 0);
        //sleep(1000);     // pause for servos to move

        //telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void spin (int timeoutS){
        if(opModeIsActive()){
            robot.arm_1.setPower(1.0); //spin the duck during auto
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running at %7d",
                        robot.spinner.getCurrentPosition());
                telemetry.update();

            }

            // Stop all motion;
            robot.arm_1.setPower(0);
        }
    }
    public void steering(int ticks, double steering, double timeoutS, double power){
        if (opModeIsActive()) {
            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            double v5;
            double v6;
            double v7;
            double v8;

            if(steering>0){
                v5 = (power/100)-(steering/100);
                v6 = (power/100)+(steering/100);
                v7 = (power/100)-(steering/100);
                v8 = (power/100)+(steering/100);
            } else if (steering<0){
                v5 = (power/100)+(steering/100);
                v6 = (power/100)-(steering/100);
                v7 = (power/100)+(steering/100);
                v8 = (power/100)-(steering/100);
            }else {
                v5 = (power/100);
                v6 = (power/100);
                v7 = (power/100);
                v8 = (power/100);
            }



            v5*=-1;
            v7*=-1;

            if(0<v5){
                robot.leftFront.setTargetPosition(-ticks);
            } else {
                robot.leftFront.setTargetPosition(ticks);
            }
            if(0<v7){
                robot.leftBack.setTargetPosition(-ticks);
            } else {
                robot.leftBack.setTargetPosition(ticks);
            }
            if(0<v6){
                robot.rightFront.setTargetPosition(-ticks);
            }else {
                robot.rightFront.setTargetPosition(ticks);
            }
            if(0<v8){
                robot.rightBack.setTargetPosition(-ticks);
            } else {
                robot.rightBack.setTargetPosition(ticks);
            }

            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftFront.setPower(v5);
            robot.rightFront.setPower(v6);
            robot.leftBack.setPower(v7);
            robot.rightBack.setPower(v8);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFront.isBusy() && robot.leftBack.isBusy()
                            &&robot.rightFront.isBusy() && robot.rightBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running at %7d :%7d",
                        robot.leftFront.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition());
                telemetry.update();
            }

            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);

        }
    }
    public void encoderDrive(int ticks, double timeoutS, double x, double y, double angleX) {
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller

            // Turn On RUN_TO_POSITION


            // reset the timeout time and start motion.
            runtime.reset();
            double r = Math.hypot(x, y);
            double robotAngle = Math.atan2(y, x) - Math.PI / 4;
            double rightX = angleX;
            double v1 = r * Math.cos(robotAngle) + rightX;
            double v2 = r * Math.sin(robotAngle) - rightX;
            double v3 = r * Math.sin(robotAngle) + rightX;
            double v4 = r * Math.cos(robotAngle) - rightX;

            v1*=-1;
            v3*=-1;

            if(0<v1){
                robot.leftFront.setTargetPosition(-ticks);
            } else {
                robot.leftFront.setTargetPosition(ticks);
            }
            if(0<v3){
                robot.leftBack.setTargetPosition(-ticks);
            } else {
                robot.leftBack.setTargetPosition(ticks);
            }
            if(0<v2){
                robot.rightFront.setTargetPosition(-ticks);
            }else {
                robot.rightFront.setTargetPosition(ticks);
            }
            if(0<v4){
                robot.rightBack.setTargetPosition(-ticks);
            } else {
                robot.rightBack.setTargetPosition(ticks);
            }

            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftFront.setPower(v1);
            robot.rightFront.setPower(v2);
            robot.leftBack.setPower(v3);
            robot.rightBack.setPower(v4);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFront.isBusy() && robot.rightBack.isBusy()
                            &&robot.leftBack.isBusy() && robot.rightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running at %7d :%7d",
                        robot.leftFront.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition());
                telemetry.update();

            }

            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void gyro(int ticks, int angleX, double timeoutS, double power){
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive()&&(((-robot.leftFront.getCurrentPosition()+-robot.leftBack.getCurrentPosition()
                +robot.rightFront.getCurrentPosition()+robot.rightBack.getCurrentPosition())
                /4)>-ticks)) {
            double v5;
            double v6;
            double v7;
            double v8;
            double steering;
            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double Gyro = angle.firstAngle;
            steering=(Gyro-angleX)*-1.5;

            if(steering<0){
                v5 = (power/100)-(steering/100);  //front left
                v6 = (power/100)+(steering/100);  //front right
                v7 = (power/100)-(steering/100);  //rear left
                v8 = (power/100)+(steering/100);  //rear right
            } else if (steering>0){
                v5 = (power/100)-(steering/100);
                v6 = (power/100)+(steering/100);
                v7 = (power/100)-(steering/100);
                v8 = (power/100)+(steering/100);
            }else {
                v5 = (power/100);
                v6 = (power/100);
                v7 = (power/100);
                v8 = (power/100);
            }



            v5*=-1;
            v7*=-1;

            robot.leftFront.setPower(-v5);
            robot.rightFront.setPower(-v6);
            robot.leftBack.setPower(-v7);
            robot.rightBack.setPower(-v8);
            telemetry.addData("Path1",  "Running at %f :%7d",
                    Gyro,
                    robot.rightFront.getCurrentPosition());
            telemetry.update();
        }
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
    }
    public void arm(int height, double speed) {
        //robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setTargetPosition(height);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(speed);
        while (robot.lift.isBusy()) {

            // Display it for the driver.
            telemetry.addData("Path1",  "Running at %7d",
                    robot.lift.getCurrentPosition());
            telemetry.update();
        }
        robot.lift.setPower(0);
    }
    public void hub(int position){
        if(position==3){
            arm(1200, 1.0);
            robot.spinner.setTargetPosition(1300);
            robot.spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.spinner.setPower(-1.0);
            robot.arm_2.setPosition(0.3);
        } else if (position==2){
            arm(900, 1.0);
            robot.spinner.setTargetPosition(1200);
            robot.spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.spinner.setPower(-0.6);
            robot.arm_2.setPosition(0.3);
        }else if (position==1){
            arm(1, 1.0);
            robot.spinner.setTargetPosition(1000);
            robot.spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.spinner.setPower(-0.6);
            robot.arm_2.setPosition(0.25);
        }else if (position==0){
            //robot.arm_1.setPosition(0.625); //0.65
            robot.arm_2.setPosition(1.0); //0.7
            robot.spinner.setTargetPosition(-1);
            robot.spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.spinner.setPower(-1.0);
            sleep(1000);
            arm(0, 1.0);
            sleep(1000);

        }
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.94f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT);

    }
}
