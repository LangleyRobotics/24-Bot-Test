// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DropperConstants;

public class DropperSubsystem extends SubsystemBase{
    private final CANSparkMax dropperMotor = new CANSparkMax(DropperConstants.kDropperMotor, MotorType.kBrushless);

    @Override
    public void periodic() {

    }

    public void setDropperMotor(double velocity){
        dropperMotor.set(velocity);
    }

    public void stopDropperMotor(){
        dropperMotor.set(0);
    }
}