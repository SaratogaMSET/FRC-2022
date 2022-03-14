package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensorThread implements AutoCloseable {
    private class SensorData {
        public RawColor raw;
        public Color color;
    }

    private final Multi2c mux = new Multi2c();
    private final Notifier notifier;
    private final int numSensors = 2;
    private final double samplePeriod = 0.02;

    private final int[] sensorMuxIndex = {1, 2};
    private volatile SensorData[] sensorData = new SensorData[numSensors];
    private final List<ColorSensorV3> sensors = new ArrayList<>();

    public ColorSensorThread(Port port) {
        sensorData[0] = new SensorData();
        sensorData[1] = new SensorData();

        if (numSensors > mux.availableBuses()) {
            throw new RuntimeException("numSensors " + numSensors + " exceeds maximum number of available buses!");
        }
        for (var index : sensorMuxIndex) {
            if (index > mux.availableBuses()) {
                throw new RuntimeException("Sensor index " + index +  " exceeds maximum number of available buses!");
            }
        }

        for (int i = 0; i < numSensors; ++i) {
            mux.setEnabledBuses(sensorMuxIndex[i]);
            var sensor = new ColorSensorV3(port);
            sensor.configureColorSensor(
                ColorSensorResolution.kColorSensorRes13bit, 
                ColorSensorMeasurementRate.kColorRate25ms, 
                GainFactor.kGain9x
            );
            sensors.add(sensor);
            sensorData[i].raw = new RawColor(0, 0, 0, 0);
            sensorData[i].color = new Color(0, 0, 0);
        }

        notifier = 
            new Notifier(
                () -> {
                    for (int i = 0; i < numSensors; ++i) {
                        mux.setEnabledBuses(sensorMuxIndex[i]);
                        var sensor = sensors.get(i);
                        sensorData[i].raw = sensor.getRawColor();
                        sensorData[i].color = sensor.getColor();
                    }
                }
            );
        
        notifier.startPeriodic(samplePeriod);
    }

    public RawColor getRawColor(int index) {
        if (index > numSensors) {
            throw new RuntimeException("Color sensor index " + index + " exceeds maximum number of available buses!");
        }
        return sensorData[index].raw;
    }

    public Color getColor(int index) {
        if (index > numSensors) {
            throw new RuntimeException("Color sensor index " + index + " exceeds maximum number of available buses!");
        }
        return sensorData[index].color;
    }

    @Override
    public void close() {
        notifier.stop();
        notifier.close();
    }
}
