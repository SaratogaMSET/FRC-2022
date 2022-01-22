// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Feeder.FeederState;
import frc.robot.subsystems.Intake.IntakeState;

/** Add your docs here. */
public class RobotState {

    public static FeederState feederState;
    public static IntakeState intakeState;

    public RobotState() {
        feederState = FeederState.IDLE;
        intakeState = IntakeState.IDLE;
    }

    public static boolean canRunFeeder() {
        if (feederState == FeederState.IDLE) return true;
        return false;
    }

    public static boolean canDeployIntake() {
        if (intakeState == IntakeState.IDLE) return true;
        return false;
    }
}
