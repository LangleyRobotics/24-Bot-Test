// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DropperSubsystem;
import frc.robot.Constants.DropperConstants;

public class DropperAutoCmd extends CommandBase{
    private long lastActivationTiming;
    private DropperSubsystem bugsBunny;

    public DropperAutoCmd(DropperSubsystem dropperSubsystem){
    this.bugsBunny = dropperSubsystem;
    }

    @Override
    public void initialize() {
        lastActivationTiming = System.currentTimeMillis();
    }

    @Override
    public void execute() {
       bugsBunny.setDropperMotor(-DropperConstants.kDropperMotorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        bugsBunny.stopDropperMotor();
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - lastActivationTiming > 100;
    }
}
