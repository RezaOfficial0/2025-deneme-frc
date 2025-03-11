// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  private final PWMTalonFX driveMotor;
  private final PWMTalonFX turnMotor;
  private final Encoder driveEncoder;
  private final Encoder turnEncoder;
  
  private final PIDController turnPID;
  private final SimpleMotorFeedforward driveFeedforward;
  
  private static final double WHEEL_DIAMETER_METERS = 0.1016; // 4 inches in meters
  private static final double ENCODER_CPR = 2048.0; // Adjust based on your encoder specs

  public SwerveModule(int driveMotorPort, int turnMotorPort, 
                    int driveEncoderA, int driveEncoderB, 
                    int turnEncoderA, int turnEncoderB) {
    driveMotor = new PWMTalonFX(driveMotorPort);
    turnMotor = new PWMTalonFX(turnMotorPort);

    driveEncoder = new Encoder(driveEncoderA, driveEncoderB);
    turnEncoder = new Encoder(turnEncoderA, turnEncoderB);

    turnPID = new PIDController(0.5, 0, 0.001);
    turnPID.enableContinuousInput(-Math.PI, Math.PI);

    driveFeedforward = new SimpleMotorFeedforward(0.1, 0.2);

    // Configure encoder conversion factors
    driveEncoder.setDistancePerPulse((Math.PI * WHEEL_DIAMETER_METERS) / ENCODER_CPR);
    turnEncoder.setDistancePerPulse((2 * Math.PI) / ENCODER_CPR);
}

  public void setDesiredState(SwerveModuleState state) {
      Rotation2d currentAngle = Rotation2d.fromRadians(turnEncoder.getDistance());
      SwerveModuleState optimizedState = SwerveModuleState.optimize(state, currentAngle);

      double driveOutput = optimizedState.speedMetersPerSecond;
      double turnOutput = turnPID.calculate(currentAngle.getRadians(), optimizedState.angle.getRadians());

      driveMotor.set(driveOutput);
      turnMotor.set(turnOutput);
  }

  public SwerveModuleState getState() {
      double velocityMetersPerSecond = driveEncoder.getRate();
      Rotation2d moduleAngle = Rotation2d.fromRadians(turnEncoder.getDistance());
      return new SwerveModuleState(velocityMetersPerSecond, moduleAngle);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveEncoder.getDistance(), 
                                    Rotation2d.fromRadians(turnEncoder.getDistance()));
}

}