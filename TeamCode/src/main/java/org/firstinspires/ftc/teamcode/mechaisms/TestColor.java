package org.firstinspires.ftc.teamcode.mechaisms;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestColor {
    ColorSensor colorSenseor;

    public enum DetectedColor{
        RED,
        BLUE,
        GREEN,
        UNKNOWN
    }
    public void init(HardwareMap HwMap){
        colorSenseor = HwMap.get(ColorSensor.class, "color_sensor");
    }

    public DetectedColor getDetectedColor(Telemetry telemetry){

        float NormRed,NormGreen,NormBlue;
        NormRed = (float)colorSenseor.red() / (float)colorSenseor.alpha();
        NormBlue = (float)colorSenseor.blue() / (float)colorSenseor.alpha();
        NormGreen = (float)colorSenseor.green() / (float)colorSenseor.alpha();

        telemetry.addData("red" , NormRed);
        telemetry.addData("green", NormGreen);
        telemetry.addData("blue", NormBlue);
        telemetry.addData("Alpha", colorSenseor.alpha());
        String colorOut;
        if(NormGreen > 0.1){
            telemetry.addLine("green");
        }
        return DetectedColor.UNKNOWN;
    }
}
