package org.firstinspires.ftc.teamcode.mechaisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.easyopencv.OpenCvCameraBase;
@TeleOp
public class ColorSensor1 extends OpMode {
    TestColor bench = new TestColor();
    @Override
    public void init() {
        bench.init(hardwareMap);
    }

    @Override
    public void loop() {
        bench.getDetectedColor(telemetry);
    }
}
