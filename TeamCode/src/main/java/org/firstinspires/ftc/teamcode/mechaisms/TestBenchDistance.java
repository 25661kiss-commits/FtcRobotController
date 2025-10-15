package org.firstinspires.ftc.teamcode.mechaisms;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TestBenchDistance {
    private DistanceSensor distance;

    public void init(HardwareMap hamap) {
        distance = hamap.get(DistanceSensor.class, "distance_sensor");
    }

    public double getDistance() {
        return distance.getDistance(DistanceUnit.METER);
    }
}

