// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;


public class ShooterSubsystem extends SubsystemBase {
  
  private final CANSparkMax shooterMotorLeft = new CANSparkMax(ShooterConstants.kLiftMotorLeftPort, MotorType.kBrushless);
  private final CANSparkMax shooterMotorRight = new CANSparkMax(ShooterConstants.kLiftMotorRightPort, MotorType.kBrushless);

  public ShooterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Lift Motor Velocity", shooterMotorLeft.get());
    SmartDashboard.putNumber("Right Lift Motor Velocity", shooterMotorRight.get());
  }

  public void setShooterMotor(double velocity) {
  
    try {
      shooterMotorRight.set(velocity);
      //motors are facing opposite ways
      shooterMotorLeft.set(-1 * velocity);

    } catch(Exception e) {
        System.out.println("Error: Shooter Motor exception:" + e.toString());
    }
  }

  public void stopShooterMotor() {
    shooterMotorRight.set(0);
    shooterMotorLeft.set(0);
  }

}
