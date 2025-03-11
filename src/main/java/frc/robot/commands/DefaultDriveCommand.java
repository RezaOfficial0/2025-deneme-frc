// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultDriveCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private final DoubleSupplier xSupplier, ySupplier, rotSupplier;

 
  public DefaultDriveCommand(DriveSubsystem driveSubsystem2, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    this.driveSubsystem = driveSubsystem2;
    this.xSupplier = x;
    this.ySupplier = y;
    this.rotSupplier = rot;
    addRequirements(driveSubsystem);
}

@Override
  public void execute() {
      driveSubsystem.drive(xSupplier.getAsDouble(), 
                           ySupplier.getAsDouble(), 
                           rotSupplier.getAsDouble());
  }
}
