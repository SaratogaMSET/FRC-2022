// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.FeederSubsystem.FeederState;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.ShooterSubsystem.ShooterZone;
import frc.robot.subsystems.VisionSubsystem.VisionState;

/** Add your docs here. */
public class RobotState {
    public static IntakeState intakeState;
    public static FeederState feederState;
    public static VisionState visionState;
    public static ShooterZone shooterState;
    

    public RobotState() {
        intakeState = IntakeState.UP;
        feederState = FeederState.IDLE;
        shooterState = ShooterZone.EMERGENCY;
        visionState = VisionState.NO_TARGET;
    }
}
