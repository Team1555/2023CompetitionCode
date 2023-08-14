package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class driveForwardCMD extends CommandBase{
    
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final double distance;

    public driveForwardCMD(DrivetrainSubsystem drivetrainSubsystem, double distance){
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.distance = distance;
        
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize(){
        drivetrainSubsystem.resetEncoders();
    }


    @Override
    public void execute(){
        drivetrainSubsystem.DistancePID(distance);
        
    }


    @Override
    public void end(boolean interrupted){
        drivetrainSubsystem.tankDrive(0, 0);

    }


    @Override
    public boolean isFinished(){
        return drivetrainSubsystem.madeIt(distance);
    }

}
