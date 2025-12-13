package org.firstinspires.ftc.teamcode;

import static java.lang.Double.max;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ltSensor extends OpMode {
    private RevColorSensorV3 color;
    private RevColorSensorV3 color2;
    private RevColorSensorV3 rtcolor2;
    private RevColorSensorV3 rtcolor;


    @Override
    public void init() {
        color = hardwareMap.get(RevColorSensorV3.class,"color_sensor_left");
        color2 = hardwareMap.get(RevColorSensorV3.class,"color_sensor_left_front");
        rtcolor = hardwareMap.get(RevColorSensorV3.class,"color_sensor_right");
        rtcolor2 = hardwareMap.get(RevColorSensorV3.class,"color_sensor_right_front");
    }

    @Override
    public void loop() {

        telemetry.addData("dist",color.getDistance(DistanceUnit.CM));
        telemetry.addData("dist2",color2.getDistance(DistanceUnit.CM));
        telemetry.addData("rtdist",rtcolor.getDistance(DistanceUnit.CM));
        telemetry.addData("rtdist2",rtcolor2.getDistance(DistanceUnit.CM));


    }
}
