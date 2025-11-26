package org.firstinspires.ftc.teamcode;

import static java.lang.Double.max;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ltSensor extends OpMode {
    private RevColorSensorV3 color;
    @Override
    public void init() {
        color = hardwareMap.get(RevColorSensorV3.class,"color_sensor_left_front");
    }

    @Override
    public void loop() {
        double r = color.red();
        double g = color.green();
        double b = color.blue();
        double a = color.alpha();
        r = r/a;
        g=g/a;
        b=b/a;
        telemetry.addData("r",r);
        telemetry.addData("g",g);
        telemetry.addData("b",b);
        telemetry.addData("a",a);
        double maxi = max(r,max(g,b));
        telemetry.addData("nr",r / maxi);
        telemetry.addData("ng",g / maxi);
        telemetry.addData("nb",b / maxi);
        telemetry.addData("dist",color.getDistance(DistanceUnit.CM));


    }
}
