// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  private final SwerveModule[] swerveModules;
  private final ADIS16470_IMU gyro;
  
  private final SwerveDriveOdometry odometry;
  
  public DriveSubsystem() {
      swerveModules = new SwerveModule[] {
        new SwerveModule(1, 2, 0, 1, 8, 9),  // Front Left
        new SwerveModule(3, 4, 2, 3, 10, 11), // Front Right
        new SwerveModule(5, 6, 4, 5, 12, 13), // Back Left
        new SwerveModule(7, 8, 6, 7, 14, 15)  // Back Right
      };

      gyro = new ADIS16470_IMU();
      gyro.calibrate();

      odometry = new SwerveDriveOdometry(
          Constants.SWERVE_KINEMATICS, 
          Rotation2d.fromDegrees(gyro.getAngle()), 
          getModulePositions()
      );
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        swerveModules[0].getPosition(),
        swerveModules[1].getPosition(),
        swerveModules[2].getPosition(),
        swerveModules[3].getPosition()
    };
}

  public void drive(double xSpeed, double ySpeed, double rotSpeed) {
      SwerveModuleState[] states = Constants.SWERVE_KINEMATICS.toSwerveModuleStates(
          new ChassisSpeeds(xSpeed, ySpeed, rotSpeed)
      );
      for (int i = 0; i < swerveModules.length; i++) {
          swerveModules[i].setDesiredState(states[i]);
      }
  }

  public void updateOdometry() {
      odometry.update(Rotation2d.fromDegrees(gyro.getAngle()), getModulePositions());
  }

  public Pose2d getPose() {
      return odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
      updateOdometry();
  }

}
