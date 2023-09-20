package org.firstinspires.ftc.teamcode;
//disable
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ProgrammingBoard1 {
   // private DigitalChannel touchSensor;
    private DcMotor motor;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private double ticksPerRotation;

    public void init(HardwareMap hwMap) {
      //  touchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");
      //  touchSensor.setMode(DigitalChannel.Mode.INPUT);
        motor = hwMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1 = hwMap.get(DcMotor.class, "motor1");
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2 = hwMap.get(DcMotor.class, "motor2");
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3 = hwMap.get(DcMotor.class, "motor3");
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerRotation = motor.getMotorType().getTicksPerRev();
    }
   /* public boolean getTouchSensorState() {
         return touchSensor.getState();
    }  */
    public void setMotorSpeed(double speed) {
        motor.setPower(-speed);
       // motor1.setPower(-speed);
       // motor2.setPower(speed);
        motor3.setPower(speed);
    }
    public double getMotorRotations(){
        return motor.getCurrentPosition() / ticksPerRotation;
    }
}