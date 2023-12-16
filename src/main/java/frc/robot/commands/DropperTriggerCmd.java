// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DropperSubsystem;
import frc.robot.Constants.DropperConstants;

public class DropperTriggerCmd extends CommandBase{
    private DropperSubsystem bugsBunny;
    private double direction;

    public DropperTriggerCmd(DropperSubsystem dropperSubsystem, double direction){
    this.bugsBunny = dropperSubsystem;
    this.direction = direction;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
       bugsBunny.setDropperMotor(DropperConstants.kDropperMotorSpeed * direction);
    }

    @Override
    public void end(boolean interrupted) {
        bugsBunny.stopDropperMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
