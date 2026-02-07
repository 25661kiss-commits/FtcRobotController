package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechaisms.gobuildaPinpoint;

@TeleOp
public class pinpointTest extends OpMode {
    gobuildaPinpoint pin = new gobuildaPinpoint();
    GoBildaPinpointDriver odo;
    @Override
    public void init() {
        pin.init(hardwareMap);
        odo = pin.getPinpoint();
    }

    @Override
    public void loop() {
        telemetry.addData("yaw" , odo.getHeading(AngleUnit.DEGREES));
        telemetry.addData("x" , odo.getPosition().getX(DistanceUnit.CM));
        telemetry.addData("y" , odo.getPosition().getY(DistanceUnit.CM));
        odo.update();
    }
}
