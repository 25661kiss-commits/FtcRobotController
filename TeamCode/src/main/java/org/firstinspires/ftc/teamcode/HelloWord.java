package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Disabled
@TeleOp
public class HelloWord extends OpMode {

    @Override
    public void init() {
        telemetry.addData("hello","world");
    }

    @Override
    public void loop() {

    }
}

