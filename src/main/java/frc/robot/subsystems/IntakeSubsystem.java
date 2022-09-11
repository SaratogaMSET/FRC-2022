// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    // TODO: Verify that these are correct constants. We might need to switch 
    // them when we test this.
    private static final boolean INTAKE_UP = false;
    private static final boolean INTAKE_DOWN = true;

    // There is just one solenoid that controls both the intake pistons.
    private Solenoid intakeSolenoid;

    public static enum IntakeState {
        DOWN,
        UP
    }

    public IntakeSubsystem() {
        intakeSolenoid = new Solenoid(2, PneumaticsModuleType.REVPH, Constants.IntakeConstants.INTAKE_SOLENOID);
    }

    @Override
    public void periodic() {

    }

    public IntakeState getIntakeState() {
        // Check the current state of the intake from the intakeSolenoid
        if (intakeSolenoid.get() == INTAKE_UP) {
            return IntakeState.UP;
        } else {
            return IntakeState.DOWN;
        }
    }

    public void updateIntakeState(IntakeState newIntakeState) {
        if (getIntakeState() == IntakeState.DOWN && newIntakeState == IntakeState.UP) {
            // Intake is down and we are being asked to move it up.
            intakeSolenoid.set(INTAKE_UP);
        }

        if (getIntakeState() == IntakeState.UP && newIntakeState == IntakeState.DOWN) {
            // Intake is up and we are being asked to move it down.
            intakeSolenoid.set(INTAKE_DOWN);
        }
    }
    
}