package org.firstinspires.ftc.teamcode;
//disable
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp()

public class ProgrammingBoard2 extends OpMode {
    private DigitalChannel touchSensor;
    private DcMotor motor;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private double ticksPerRotation;

    @Override
    public void init() {
        touchSensor = hardwareMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerRotation = motor.getMotorType().getTicksPerRev();
    }
    public boolean getTouchSensorState() {
        return touchSensor.getState();
    }
    public void setMotorSpeed(double speed) {
        motor.setPower(-speed);
        motor1.setPower(speed);
        motor2.setPower(-speed);
        motor3.setPower(speed);
    }
    public double getMotorRotations(){
        return motor.getCurrentPosition() / ticksPerRotation;
    }

    @Override
    public void loop() {

    }
}