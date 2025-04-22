package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Project1Hardware {
//    DcMotor FL, FR, BL, BR;
    Servo claw, clawYaw;
    IMU imu;

    Telemetry telemetry;


    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
//        FL = hardwareMap.get(DcMotor.class, "FL");
//        FR = hardwareMap.get(DcMotor.class, "FR");
//        BL = hardwareMap.get(DcMotor.class, "BL");
//        BR = hardwareMap.get(DcMotor.class, "BR");
        claw = hardwareMap.get(Servo.class, "claw");
        clawYaw = hardwareMap.get(Servo.class, "clawYaw");
        imu = hardwareMap.get(IMU.class, "imu");

        this.telemetry = telemetry;
    }

    public void reset() {
        claw.setDirection(Servo.Direction.FORWARD);
    }

    public void clawOpen() {
        claw.setPosition(0);
    }

    public void clawClose() {
        claw.setPosition(0.2);
    }

    public void setScoringPos(int pos) {
        switch (pos) {
            case 0: // transfer
                clawOpen();
                clawYaw.setPosition(0);
                break;
            case 1: // scoring
                clawClose();
                clawYaw.setPosition(.65);
        }
    }

}
