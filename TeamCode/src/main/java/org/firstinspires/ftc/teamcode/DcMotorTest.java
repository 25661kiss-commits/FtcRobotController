package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechaisms.TestBenchB;
import org.firstinspires.ftc.teamcode.mechaisms.TestBenchD;

@TeleOp
public class DcMotorTest extends OpMode {
    TestBenchD bench = new TestBenchD();
    TestBenchB button = new TestBenchB();
    @Override
    public void init() {
        bench.init(hardwareMap);
        button.init(hardwareMap);
    }

    @Override
    public void loop() {
        if(button.TouchSensorPressed()) {
            bench.setMotorSpeed(0.5);
        }else{
            bench.setMotorSpeed(0);
        }
        telemetry.addData("motor revs:", bench.getMotorRevs());
    }
}