// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import java.lang.Object;

// import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GripperSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.armFloorCMD;
import frc.robot.commands.armCMD;
import frc.robot.commands.driveForwardCMD;
import frc.robot.commands.driveTurnCMD;
// import frc.robot.commands.driveTurnCMD;
import frc.robot.commands.gripperCloseCMD;
import frc.robot.commands.gripperOpenCMD;

import edu.wpi.first.cameraserver.CameraServer;//CAMERA!!
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
//End of Camera

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final GripperSubsystem m_gripper = new GripperSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  private final Timer m_timer = new Timer();

  private XboxController m_driveController = new XboxController(Constants.OIConstants.kDriverController); 
  private XboxController m_armController = new XboxController(Constants.OIConstants.kArmController);

  Thread m_visionThread;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_visionThread = new Thread(() -> {
      UsbCamera m_camera = CameraServer.startAutomaticCapture();//turns on camera
      m_camera.setFPS(30);
      m_camera.setResolution(10, 10);

      CvSink cvsink = CameraServer.getVideo();
      CvSource cvsource = CameraServer.putVideo("Rectangle", 160, 120);
      Mat mat = new Mat(); //From Wii Sports?!

      while (!Thread.interrupted()) {
        if (cvsink.grabFrame(mat) == 0){
          cvsource.notifyError(cvsink.getError());
          continue;
        }
        Imgproc.rectangle(mat, new Point(80, 60), new Point(160, 120), new Scalar(255, 255, 255), 5);
        cvsource.putFrame(mat);
      }
    });
    m_visionThread.setDaemon(true);
    m_visionThread.start();

    // Configure the button bindings
    configureButtonBindings(); 
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //set up the drivetrain command that runs all the time
    m_drivetrain.setDefaultCommand(new RunCommand(
      () -> 
          // m_drivetrain.driveArcade(
          //   MathUtil.applyDeadband(m_driveController.getLeftY(), Constants.OIConstants.kDriveDeadband),
          //   MathUtil.applyDeadband(m_driveController.getRightX(), Constants.OIConstants.kDriveDeadband))
        m_drivetrain.tankDrive(
          MathUtil.applyDeadband(m_driveController.getLeftY(), Constants.OIConstants.kDriveDeadband),
          MathUtil.applyDeadband(m_driveController.getRightY(), Constants.OIConstants.kDriveDeadband))
      , m_drivetrain)
    );

    // //Move the robot 10 feet
    // new JoystickButton(m_driveController, XboxController.Button.kA.value)
    // .onTrue(new driveForwardCMD(m_drivetrain, 10));
    // //Turn the robot 90 degrees
    // new JoystickButton(m_driveController, XboxController.Button.kX.value)
    // .onTrue(new driveTurnCMD(m_drivetrain, -90));
    
    //set up gripper open big/small/close
    new JoystickButton(m_armController, XboxController.Button.kLeftBumper.value)
    .onTrue(new InstantCommand(() -> m_gripper.openGripperFloor()))
    .onFalse(new InstantCommand(() -> m_gripper.closeGripper()));
    new JoystickButton(m_armController, XboxController.Button.kRightBumper.value)
      .onTrue(new InstantCommand(() -> m_gripper.openGripperSubstation()))
      .onFalse(new InstantCommand(() -> m_gripper.closeGripper()));

    //set up arm preset positions
    new JoystickButton(m_armController, XboxController.Button.kA.value)
      .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kHomePosition, m_gripper)));
    new JoystickButton(m_armController, XboxController.Button.kX.value)
      .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kScoringPosition, m_gripper)));
    new JoystickButton(m_armController, XboxController.Button.kY.value)
      .toggleOnTrue(new armFloorCMD(m_arm, m_gripper));
    new JoystickButton(m_armController, XboxController.Button.kB.value)
      .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kFeederPosition, m_gripper)));

    //set up arm manual and auto functions
    m_arm.setDefaultCommand(new RunCommand(
      () ->
        m_arm.runAutomatic()
      , m_arm)
    );
    new Trigger(() -> 
      Math.abs(m_armController.getRightTriggerAxis() - m_armController.getLeftTriggerAxis()) > Constants.OIConstants.kArmManualDeadband
      ).whileTrue(new RunCommand(
        () ->
          m_arm.runManual((m_armController.getRightTriggerAxis() - m_armController.getLeftTriggerAxis()) * Constants.OIConstants.kArmManualScale)
        , m_arm));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    m_timer.reset();
    m_timer.start();
    double auto = 0;
    auto = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
    
    // if(auto == 1 || auto == 3){
    // return new SequentialCommandGroup(new armCMD(m_arm, m_gripper, 3.0),
    //                            new gripperOpenCMD(m_gripper), new gripperCloseCMD(m_gripper),
    //                            new armCMD(m_arm, m_gripper, Constants.Arm.kHomePosition),
    //                            new driveForwardCMD(m_drivetrain, 11));
    // }
    // if(auto == 6 || auto == 8){
    // return new SequentialCommandGroup(new armCMD(m_arm, m_gripper, 3.0),
    //                            new gripperOpenCMD(m_gripper), new gripperCloseCMD(m_gripper),
    //                            new armCMD(m_arm, m_gripper, Constants.Arm.kHomePosition),
    //                            new driveForwardCMD(m_drivetrain, 11));
    // }
    // if(auto == 2 || auto == 7){
    // return new SequentialCommandGroup(new armCMD(m_arm, m_gripper, 3.0),
    //                            new gripperOpenCMD(m_gripper), new gripperCloseCMD(m_gripper),
    //                            new armCMD(m_arm, m_gripper, Constants.Arm.kHomePosition),
    //                            new driveForwardCMD(m_drivetrain, 12));
    // }
    // else{
      return new SequentialCommandGroup(new armCMD(m_arm, m_gripper, 3.0),
                                 new gripperOpenCMD(m_gripper), new gripperCloseCMD(m_gripper),
                                 new armCMD(m_arm, m_gripper, Constants.Arm.kHomePosition),
                                 new driveForwardCMD(m_drivetrain, 11.5));
    // return new driveTurnCMD(m_drivetrain, 90);
    // }
  }
}