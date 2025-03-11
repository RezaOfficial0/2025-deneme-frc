// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.MoveToHeight;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Elevator.Elevator;


import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //Subsystems
     private final Elevator elevator = new Elevator();
      private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final Joystick joystick = new Joystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    driveSubsystem.setDefaultCommand(new DefaultDriveCommand(
      driveSubsystem,
      () -> -driverController.getRawAxis(1), // Forward/Backward
      () -> -driverController.getRawAxis(0), // Left/Right
      () -> -driverController.getRawAxis(4)  // Rotation
  ));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
     new JoystickButton(joystick, 1).onTrue(new MoveToHeight(elevator, 10)); // Move to 10 inches
        new JoystickButton(joystick, 2).onTrue(new MoveToHeight(elevator, 30)); // Move to 30 inches
  
  
       
  }

  
  public Elevator getElevator() {
    return elevator;
}

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
