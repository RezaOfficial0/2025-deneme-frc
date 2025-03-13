// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {
    private final PWMTalonFX elevatorMotor = new PWMTalonFX(0);
    private final Encoder elevatorEncoder = new Encoder(0, 1);
    private final PIDController pidController = new PIDController(0.1, 0, 0);

    private static final double TOLERANCE = 0.5; 
    private static final double DISTANCE_PER_PULSE = 0.01;

    private double setpoint = 0.0;

    public Elevator() {
        elevatorEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);


       
    }

    public void setTargetHeight(double height) {
        setpoint = height;
    }

    public double getCurrentHeight() {
        return elevatorEncoder.getDistance(); 
    }

    public boolean atSetpoint() {
        return Math.abs(getCurrentHeight() - setpoint) < TOLERANCE;
    }

    @Override
    public void periodic() {
        double output = pidController.calculate(getCurrentHeight(), setpoint);
        elevatorMotor.set(output);
    }

    public void stop() {
        elevatorMotor.set(0);
    }
}
