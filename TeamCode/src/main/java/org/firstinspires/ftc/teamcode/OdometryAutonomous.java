package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Odometry Autonomous")
public class OdometryAutonomous extends LinearOpMode {

    // Declare hardware components
    DcMotor leftFront, rightFront, leftRear, rightRear;
    // Declare odometry variables
    double robotX = 0, robotY = 0, robotHeading = 0;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;

    @Override
    public void runOpMode() {

        // Initialize hardware components
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");

        // Set motor directions
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for start
        waitForStart();

        // Autonomous actions using odometry
        driveToPosition(24, 0); // Drive 24 inches forward
        turnToAngle(90);        // Turn 90 degrees clockwise
        driveToPosition(0, 24); // Strafe 24 inches right
    }

    // Function to drive to a specified position
    private void driveToPosition(double targetX, double targetY) {
        while (opModeIsActive()) {
            // Update odometry (implement updateOdometry() method)
            updateOdometry();

            // Calculate error
            double xError = targetX - robotX;
            double yError = targetY - robotY;

            // Implement control logic to move the robot based on errors
            // ...
        }
    }

    // Function to turn to a specified angle
    private void turnToAngle(double targetAngle) {
        while (opModeIsActive()) {
            // Update odometry
            updateOdometry();

            // Calculate error
            double angleError = targetAngle - robotHeading;

            // Implement control logic to turn the robot based on error
            // ...
        }
    }

    // Function to update odometry (implement based on your robot's configuration)
    private void updateOdometry() {
        // Read encoder values and calculate robot pose (x, y, heading)
        // ...
    }
}