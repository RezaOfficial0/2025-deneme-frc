// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BargeIntake extends SubsystemBase {
 private final PWMTalonFX PivotMotor ;
    private final Encoder PivotEncoder ;
    private final PIDController pidController;


    private static final double TOLERANCE = 0.5; 
    private static final double DISTANCE_PER_PULSE = 0.01;

    private static final double kP = 0.05;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private double setpoint = 0.0;
 public BargeIntake(int motorID, int encoderChannelA, int encoderChannelB) {
        PivotMotor = new PWMTalonFX(motorID);
        PivotEncoder = new Encoder(encoderChannelA, encoderChannelB);
        pidController = new PIDController(kP, kI, kD);
        
        PivotEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);// Adjust if necessary
    }

    public void setPivotPosition(double targetAngle) {
        double currentAngle = getPivotAngle();
        double output = pidController.calculate(currentAngle, targetAngle);
        PivotMotor.set(output);
    }

    public double getPivotAngle() {
      return PivotEncoder.getDistance(); 
    }

    public void stop() {
        PivotMotor.set(0);
    }
    public boolean atSetpoint() {
      return Math.abs(getPivotAngle() - setpoint) < TOLERANCE;
  }
  @Override
  public void periodic() {
      double output = pidController.calculate(getPivotAngle(), setpoint);
      PivotMotor.set(output);
  }
}
