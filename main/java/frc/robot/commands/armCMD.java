package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class armCMD extends CommandBase{
    
    private final ArmSubsystem armsubsystem;
    private final GripperSubsystem grippersubsystem;
    private Double position;
      

    public armCMD(ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem, Double position){
        this.armsubsystem = armSubsystem;
        this.grippersubsystem = gripperSubsystem;
        this.position = position;
        
        addRequirements(armSubsystem);
        addRequirements(gripperSubsystem);

    }

    @Override
    public void initialize(){
    }


    @Override
    public void execute(){
        armsubsystem.setTargetPosition(position, grippersubsystem);
        armsubsystem.runAutomatic();
    }


    @Override
    public void end(boolean interrupted){
    }


    @Override
    public boolean isFinished(){
        return armsubsystem.isDoneMoving();
    }

}
