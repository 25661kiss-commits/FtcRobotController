package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class DistTest extends OpMode {
    DistanceSensor dist;
    DistanceSensor dist2;
    @Override
    public void init(){
        dist = hardwareMap.get(DistanceSensor.class,"dist_sensor_left");
        dist2 = hardwareMap.get(DistanceSensor.class,"dist_sensor_right");
    }
    @Override
    public void loop(){
        telemetry.addData("leftD",dist.getDistance(DistanceUnit.CM));
        telemetry.addData("rightD",dist2.getDistance(DistanceUnit.CM));
    }
}
