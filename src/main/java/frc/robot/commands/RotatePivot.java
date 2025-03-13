// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BargeIntake;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotatePivot extends Command {
  /** Creates a new RotatePivot. */
  private final BargeIntake intake ;
  private final double targetAngle ;
  public RotatePivot(BargeIntake intake, double angle) {
    this.intake = intake;
    this.targetAngle = angle;
    addRequirements(intake);
  }

  @Override
    public void initialize() {
        intake.setPivotPosition(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return intake.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            intake.stop();
        }
    }
}
