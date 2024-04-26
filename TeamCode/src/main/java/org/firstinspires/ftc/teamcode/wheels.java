package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class wheels extends HardwareMecanum{
    private final double diveSpeed = 0.4;
    private DcMotor motor;
    public wheels (DcMotor motor1, DcMotor motor2) {
        this.motor = motor1;
    }

    public double hello() {return 1;}
}
