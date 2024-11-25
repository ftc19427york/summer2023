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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Iterative OpMode", group="Iterative OpMode")
//@Disabled
public class Drive_To_Point extends OpMode
{
    // Declare OpMode members.
    HardwareMecanum robot = new HardwareMecanum();   // Use a Mecanum's hardware
    GoBildaPinpointDriver odo;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    double kp = 0.05;
    Pose2D desiredPose = new Pose2D(DistanceUnit.INCH, 20, 20, AngleUnit.DEGREES, 0);

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        robot.init(hardwareMap);

        odo.setOffsets(108, 54);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        odo.resetPosAndIMU();
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        //update current pos
        Pose2D currentPose = odo.getPosition();
        //find error from current position
        double errorX = desiredPose.getX(DistanceUnit.INCH) - currentPose.getX(DistanceUnit.INCH);
        double errorY = desiredPose.getY(DistanceUnit.INCH) - currentPose.getY(DistanceUnit.INCH);
        double errorH = desiredPose.getHeading(AngleUnit.DEGREES) - currentPose.getHeading(AngleUnit.DEGREES);

        //distance formula
        double distanceToPoint = Math.sqrt(Math.pow(errorX, 2) + Math.pow(errorY, 2));
        //check if distance is greater than 1 inch
        if(distanceToPoint > 1){
            //move robot | increase gains to get better accuracy
            drive(kp*errorX, kp*errorY, kp*errorH, currentPose.getHeading(AngleUnit.DEGREES));
        }
        //pull new data from pinpoint
        odo.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void drive(double x, double y, double theta, double angle) {

        /**gets squared values from the driver's stick input**/
        double r = Math.hypot(-y, -x);
        /**finds the desired angle that the driver wants to move the robot**/
        double robotAngle = Math.atan2(-y, x) - Math.PI / 4;
        /**sets the movement angle by finding the difference of the robots angle, the input angle and the offset value
         * the offset value is set by the the driver if the imu does not reset after auto*/
        robotAngle = robotAngle - Math.toRadians(angle);

        double rightX = theta;

        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        // Output the safe vales to the motor drives.++
        robot.frontLeftDrive.setPower(v1);    //motor3
        robot.backRightDrive.setPower(v4);    //motor
        robot.frontRightDrive.setPower(v2);   //motor2
        robot.backLeftDrive.setPower(v3);     //motor1
    }

}
