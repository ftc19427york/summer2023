/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;


@Autonomous(name="Basic: Iterative OpMode", group="Iterative OpMode")
//@Disabled
public class RedLeft extends OpMode
{
    HardwareMecanum robot = new HardwareMecanum();
    static final int MOTOR_TICK_COUNT = 537;

    private ElapsedTime runtime = new ElapsedTime();
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "blueVexBall.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    //private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "blue marker",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    // static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private IMU             imu         = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    double x;


    int tiltEncoderDrivingTarget = 0;
    @Override
    public void init() {
        robot.init(hardwareMap);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        robot.driveWithEncode();
        initTfod();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        x = telemetryTfod();
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
       // x = telemetryTfod();
        if (x> 500) {
            driveStraight(19, telemetry, this);
            driveSpinRight(.25,480, telemetry);//535
            armTilt(13);
            robot.twistClaw.setPosition(0.30);
            robot.claw.setPosition(0);
        }else if (x> 135 && x< 385) {
            driveStraight(23, telemetry, this);
             armTilt(13);
            robot.twistClaw.setPosition(0.30);
            robot.claw.setPosition(0);
        }else {
            driveStraight(23, telemetry, this);
            driveSpinLeft(.25,750, telemetry);//535
            armTilt(13);
            robot.twistClaw.setPosition(0.30);
            robot.claw.setPosition(0);
        }

            telemetry.addData("Path", "Complete");
            telemetry.update();
            }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */


    public void armTilt (int tiltDistance) {

            //   double tiltRotationsNeeded = rotations / circumference;
            int tiltEncoderDrivingTarget = (int) (tiltDistance * 537.7); //  Don't need to look at the existing encoder counts
            // because it was reset above to 0.

            robot.tilt.setTargetPosition(tiltEncoderDrivingTarget);
            robot.tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.tilt.setPower(.8);
        while( robot.tilt.isBusy()) {

        }
            robot.tilt.setPower(0);
        robot.tilt.setTargetPosition(0);
        robot.tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.tilt.setPower(.8);

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way



/*
        while (robot.frontLeftDrive.isBusy() || robot.backRightDrive.isBusy()) {
            telemetry.addData("Path", "Driving 18 inches");
            telemetry.update();
        }
        robot.stopDriving(); */
    }



    public void driveStraight (double distanceInches, Telemetry telemetry, OpMode opMode) {

        double circumference = 3.14 * 3.78;  //pi* diameter
        double rotationsNeeded = distanceInches / circumference;

        int encoderDrivingTarget = (int) (rotationsNeeded * 537.7); //  Don't need to look at the existing encoder counts
        // because it was reset above to 0.

        robot.frontLeftDrive.setTargetPosition(encoderDrivingTarget);
        robot.frontRightDrive.setTargetPosition(encoderDrivingTarget);
        robot.backLeftDrive.setTargetPosition(encoderDrivingTarget);
        robot.backRightDrive.setTargetPosition(encoderDrivingTarget);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
        robot.frontRightDrive.setPower(.25);
        robot.backRightDrive.setPower(.25);
        robot.frontLeftDrive.setPower(.25);
        robot.backLeftDrive.setPower(.25);


        while (robot.frontLeftDrive.isBusy() || robot.backRightDrive.isBusy()) {
            telemetry.addData("Path", "Driving 18 inches");
            telemetry.update();
        }
        robot.stopDriving();
    }


    public void driveSpinLeft(double power, int spinTicks, Telemetry telemetry){
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftDrive.setTargetPosition(-spinTicks);  //1075 535
        robot.frontRightDrive.setTargetPosition(spinTicks);
        robot.backLeftDrive.setTargetPosition(-spinTicks);
        robot.backRightDrive.setTargetPosition(spinTicks);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeftDrive.setPower(-power);
        robot.frontRightDrive.setPower(power);
        robot.backLeftDrive.setPower(-power);
        robot.backRightDrive.setPower(power);
/*
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset(); //starts the time at 0
        while (opMode.opModeIsActive() &&  (runtime.seconds() < seconds)) {
            //print time to DS
            telemetry.addData("Path", "driveStraight: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
 */
        while( robot.frontRightDrive.isBusy()) {

        }
    }

    public void driveSpinRight(double power, int spinTicks, Telemetry telemetry){
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftDrive.setTargetPosition(spinTicks);  //1075 535
        robot.frontRightDrive.setTargetPosition(-spinTicks);
        robot.backLeftDrive.setTargetPosition(spinTicks);
        robot.backRightDrive.setTargetPosition(-spinTicks);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeftDrive.setPower(power);
        robot.frontRightDrive.setPower(-power);
        robot.backLeftDrive.setPower(power);
        robot.backRightDrive.setPower(-power);
/*
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset(); //starts the time at 0
        while (opMode.opModeIsActive() &&  (runtime.seconds() < seconds)) {
            //print time to DS
            telemetry.addData("Path", "driveStraight: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
 */
        while( robot.frontRightDrive.isBusy()) {

        }
    }


    public void strafeRight (double distanceInches, Telemetry telemetry, OpMode opMode) {

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
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    public double telemetryTfod() {
        //double x=0;
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop
        return x;
    }   // end method telemetryTfod()

}
