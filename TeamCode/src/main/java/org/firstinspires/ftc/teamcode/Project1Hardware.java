package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Project1Hardware {
    DcMotor FL, FR, BL, BR;
    IMU imu;

    Telemetry telemetry;


    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        imu = hardwareMap.get(IMU.class, "imu");
        this.telemetry = telemetry;
    }

    public void reset() {

    }


}
