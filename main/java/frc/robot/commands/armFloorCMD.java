package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class armFloorCMD extends CommandBase{
    
    private final ArmSubsystem armsubsystem;
    private final GripperSubsystem grippersubsystem;   

    public armFloorCMD(ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem){
        this.armsubsystem = armSubsystem;
        this.grippersubsystem = gripperSubsystem;
        
        addRequirements(armSubsystem);
        addRequirements(gripperSubsystem);

    }

    @Override
    public void initialize(){
    }


    @Override
    public void execute(){
        armsubsystem.setTargetPosition(Constants.Arm.kIntakePosition, grippersubsystem);
        armsubsystem.runAutomatic();
        if (armsubsystem.checkArmPosition() > 2.2){
          grippersubsystem.openGripperFloor();
    
        }

    }


    @Override
    public void end(boolean interrupted){
    }


    @Override
    public boolean isFinished(){
        return armsubsystem.isDoneMoving();
    }

}
