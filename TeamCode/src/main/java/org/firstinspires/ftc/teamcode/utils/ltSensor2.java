package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
//@Disabled
public class ltSensor2 extends OpMode {
    private RevColorSensorV3 color;



    @Override
    public void init() {
        color = hardwareMap.get(RevColorSensorV3.class,"color");

    }

    @Override
    public void loop() {

        telemetry.addData("color",color.getDistance(DistanceUnit.CM));


    }
}
