package org.firstinspires.ftc.teamcode;

/*
@Autonomous
public class testColor extends LinearOpMode {
    // Define a variable for our color sensor
    HardwareMecanum robot = new HardwareMecanum();

  //  RevColorSensorV3 frontColor;

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        robot.init(hardwareMap);

        robot.frontColor = hardwareMap.get(RevColorSensorV3.class, "sensor_color");

        // Wait for the Play button to be pressed
        waitForStart();
        robot.frontRightDrive.setPower(.25);
        robot.backRightDrive.setPower(.25);
        robot.frontLeftDrive.setPower(.25);
        robot.backLeftDrive.setPower(.25);

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()  && (robot.frontColor.blue() < 4000) ) {
            telemetry.addData("Red", robot.frontColor.red());
            telemetry.addData("Green", robot.frontColor.green());
            telemetry.addData("Blue", robot.frontColor.blue());
            telemetry.update();
        }
            robot.stopDriving();


        }
    }
*/