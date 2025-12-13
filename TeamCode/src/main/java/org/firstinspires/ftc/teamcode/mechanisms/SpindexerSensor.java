package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SpindexerSensor {
    private final NormalizedColorSensor colorSensor;
    private final DistanceSensor distanceSensor;
    private final Telemetry telemetry;
    private final String deviceName;
    public enum SensorState {
        green,
        purple,
        empty,
        midSpin
    }

    public SpindexerSensor(HardwareMap hwMap, String deviceName, Telemetry telemetry) {
        colorSensor = hwMap.get(NormalizedColorSensor.class, deviceName);
        distanceSensor = hwMap.get(DistanceSensor.class, deviceName);
        colorSensor.setGain(5);
        this.telemetry = telemetry;
        this.deviceName = deviceName;
    }

    public double test() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }


public float hopperRed, hopperGreen, hopperBlue;



    public SensorState getDetectedColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double dist = distanceSensor.getDistance(DistanceUnit.INCH);
        float normRed,normGreen,normBlue;

        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
        hopperRed = normRed;
        hopperGreen = normGreen;
        hopperBlue = normBlue;

        if (this.deviceName.contains("hopper")) {
            if (1.5  < dist && dist <= 2.1)
            {
                return SensorState.midSpin;
            }
            if (normBlue > normGreen){
                return SensorState.purple;
            } else{
                return SensorState.green;
            }
        } else {
            if (dist > 1.75) {
                return SensorState.empty;
            } else if (dist < 0.5) {
                if (normBlue > normGreen) {
                    return SensorState.purple;
                } else {
                    return SensorState.green;
                }
            }
        }

        return SensorState.midSpin;
    }
}
