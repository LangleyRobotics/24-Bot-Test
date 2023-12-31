package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;



public class SullyCmd extends CommandBase{
    private final PneumaticsSubsystem pneumaticsSubsystem;
    private long lastActivationTime;

    public SullyCmd(PneumaticsSubsystem pneumaticsSubsystem) {
        this.pneumaticsSubsystem = pneumaticsSubsystem;
        addRequirements(pneumaticsSubsystem);
    }

    @Override
    public void initialize() {
        pneumaticsSubsystem.toggleSully();
        lastActivationTime=System.currentTimeMillis();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
            //pneumaticsSubsystem.toggleSully();
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis()-lastActivationTime>1000;
    }



}
