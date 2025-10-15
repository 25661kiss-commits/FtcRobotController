package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechaisms.TestBenchDistance;

@TeleOp
public class DistanceTest extends OpMode {
    TestBenchDistance bench = new TestBenchDistance();
    double distance;

    @Override
    public void init() {
        bench.init(hardwareMap);
    }

    @Override
    public void loop() {
        distance = bench.getDistance();

        if (distance < 0.100)
            telemetry.addLine("TOO CLOSE BACK UP BRO!!!!!!!!");
        telemetry.addData("Disteance", bench.getDistance());
    }
}
