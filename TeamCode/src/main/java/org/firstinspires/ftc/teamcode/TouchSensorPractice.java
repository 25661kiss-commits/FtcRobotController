package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechaisms.TestBench1;
@TeleOp
@Disabled
public class TouchSensorPractice extends OpMode {

    TestBench1 bench =new TestBench1();

    @Override
    public void init() {
       bench.init(hardwareMap);



    }


    @Override
    public void loop() {
        telemetry.addData("Touch sensor state",bench.isTouchSensorPressed());


    }
}
