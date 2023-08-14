package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class driveTurnCMD extends CommandBase{
    
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final double turn;
    private double angle;

    public driveTurnCMD(DrivetrainSubsystem drivetrainSubsystem, double turn){
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.turn = turn;
        
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize(){
        drivetrainSubsystem.resetEncoders();
        angle = turn;
        drivetrainSubsystem.resetYAW();
    }


    @Override
    public void execute(){
        drivetrainSubsystem.TurnPID(angle);
        
    }


    @Override
    public void end(boolean interrupted){
        drivetrainSubsystem.tankDrive(0, 0);

    }


    @Override
    public boolean isFinished(){
        return false;
    }

}
