// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Feeder.FeederState;

/** Add your docs here. */
public class RobotState {

    public static FeederState feederState;

    public RobotState() {
        feederState = FeederState.IDLE;
    }

    public static boolean canRunFeeder() {
        if (feederState == FeederState.IDLE) {
            return true;
        }
        return false;
    }
}
