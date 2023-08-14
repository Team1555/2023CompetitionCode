// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DrivetrainSubsystem extends SubsystemBase {
  private CANSparkMax m_frontLeftMotor;
  private CANSparkMax m_frontRightMotor;
  private CANSparkMax m_rearLeftMotor;
  private CANSparkMax m_rearRightMotor;

  //If PID doesn't move correctly remove all of right, and remove L pieces of values
  //private PIDController PIDL;
  //private PIDController PIDR;
  private double outputSpeedL;
  private double errorL;
  private double sensorPositionL;
  private double Position;
  private double outputSpeed;
  private double error;
  //private double outputSpeedR;
  // private double errorR;
  // private double sensorPositionR;

  private RelativeEncoder m_frontLeftEncoder;
  private RelativeEncoder m_frontRightEncoder;

  private AHRS ahrs;

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    m_frontLeftMotor  = new CANSparkMax(Constants.Drivetrain.kFrontLeftCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_frontLeftMotor.setInverted(Constants.Drivetrain.kFrontLeftInverted);
    m_frontLeftMotor.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
    m_frontLeftMotor.setIdleMode(IdleMode.kBrake);
    //Hey, encoder and stuff
    m_frontLeftEncoder = m_frontLeftMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    m_frontLeftEncoder.setPositionConversionFactor(Constants.Arm.kPositionFactor);
    m_frontLeftEncoder.setVelocityConversionFactor(Constants.Arm.kVelocityFactor);

    m_frontLeftMotor.burnFlash();

    m_frontRightMotor = new CANSparkMax(Constants.Drivetrain.kFrontRightCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_frontRightMotor.setInverted(Constants.Drivetrain.kFrontRightInverted);
    m_frontRightMotor.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
    m_frontRightMotor.setIdleMode(IdleMode.kBrake);
    
    m_frontRightEncoder = m_frontLeftMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    m_frontRightEncoder.setPositionConversionFactor(Constants.Arm.kPositionFactor);
    m_frontRightEncoder.setVelocityConversionFactor(Constants.Arm.kVelocityFactor);
    //Hey, encoder and stuff
    m_frontRightEncoder = m_frontRightMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    m_frontRightEncoder.setPositionConversionFactor(Constants.Arm.kPositionFactor);
    m_frontRightEncoder.setVelocityConversionFactor(Constants.Arm.kVelocityFactor);

    m_frontRightMotor.burnFlash();

    m_rearLeftMotor   = new CANSparkMax(Constants.Drivetrain.kRearLeftCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_rearLeftMotor.setInverted(Constants.Drivetrain.kRearLeftInverted);
    m_rearLeftMotor.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
    m_rearLeftMotor.setIdleMode(IdleMode.kBrake);
    m_rearLeftMotor.burnFlash();

    m_rearRightMotor  = new CANSparkMax(Constants.Drivetrain.kRearRightCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_rearRightMotor.setInverted(Constants.Drivetrain.kRearRightInverted);
    m_rearRightMotor.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
    m_rearRightMotor.setIdleMode(IdleMode.kBrake);
    m_rearRightMotor.burnFlash();

    ahrs = new AHRS(SPI.Port.kMXP);
  }

  public void driveArcade(double _straight, double _turn) {
    double left  = MathUtil.clamp(_straight + _turn, -1.0, 1.0);
    double right = MathUtil.clamp(_straight - _turn, -1.0, 1.0);

    m_frontLeftMotor.set(left);
    m_frontRightMotor.set(right);
    m_rearLeftMotor.set(left);
    m_rearRightMotor.set(right);
  }

  public void tankDrive(double left, double right) {
      m_frontLeftMotor.set(left);
      m_frontRightMotor.set(right);
      m_rearLeftMotor.set(left);
      m_rearRightMotor.set(right);
  }

  public void resetEncoders(){
    m_frontLeftEncoder.setPosition(0);
    m_frontRightEncoder.setPosition(0);
  }

  public double getEncoder(){
    return m_frontLeftEncoder.getPosition();
  }

  // public void DistancePID(double setpoint){

  //   PIDL = new PIDController(Constants.Drivetrain.kP, Constants.Drivetrain.kI, Constants.Drivetrain.kD, 0.1);
  //   PIDL.setTolerance(0.5);
  //   PIDL.setSetpoint(setpoint / 4);
  //   double outputL = PIDL.calculate(m_frontLeftEncoder.getPosition(), setpoint);
    
  //   PIDR = new PIDController(Constants.Drivetrain.kP, Constants.Drivetrain.kI, Constants.Drivetrain.kD, 0.1);
  //   PIDR.setTolerance(0.5);
  //   PIDR.enableContinuousInput(-1.0, 1.0);
  //   double outputR = PIDR.calculate(m_frontRightEncoder.getPosition(), setpoint);

  //   m_frontLeftMotor.set(outputL);
  //   m_frontRightMotor.set(outputR);
  //   m_rearLeftMotor.set(outputL);
  //   m_rearRightMotor.set(outputR);
  // }

  // public void TurnPID(double angle){
  //   double turnAmount = ahrs.getYaw() + angle;
  //   PIDL = new PIDController(Constants.Drivetrain.kP, Constants.Drivetrain.kI, Constants.Drivetrain.kD, 0.3);
  //   PIDL.setTolerance(0.5);
  //   PIDL.setSetpoint(angle);
  //   double outputL = PIDL.calculate(turnAmount, angle);
    
  //   PIDR = new PIDController(Constants.Drivetrain.kP, Constants.Drivetrain.kI, Constants.Drivetrain.kD, 0.3);
  //   PIDR.setTolerance(0.5);
  //   PIDR.enableContinuousInput(-1.0, 1.0);
  //   double outputR = PIDR.calculate(turnAmount, angle);

  //   m_frontLeftMotor.set(outputL);
  //   m_frontRightMotor.set(outputR);
  //   m_rearLeftMotor.set(outputL);
  //   m_rearRightMotor.set(outputR);


  // }

  public void DistancePID(double setpoint){
    //Get encoder position Left
    sensorPositionL = m_frontLeftEncoder.getPosition() * Constants.Drivetrain.kDistanceConversion;
    
    //Get encoder position Right
    //sensorPositionR = m_frontRightEncoder.getPosition() * Constants.Drivetrain.kDistanceConversion;

    //Calculations Left
    double currentL = (setpoint / 4) - m_frontLeftEncoder.getPosition();
    errorL = currentL - sensorPositionL;
    double dtL = Timer.getFPGATimestamp() - Constants.Drivetrain.lastTimeStampL;

    if(Math.abs(errorL) < Constants.Drivetrain.ilimit){
      Constants.Drivetrain.errorSumL += errorL*dtL;
    }

    double errorRateL = (errorL - Constants.Drivetrain.lastErrorL) / dtL;
    outputSpeedL = Constants.Drivetrain.kP * errorL + Constants.Drivetrain.kI * Constants.Drivetrain.errorSumL + Constants.Drivetrain.kD * errorRateL;

    // //Calculations Right
    // double currentR = (setpoint / 4) - m_frontRightEncoder.getPosition();
    // errorL = currentR - sensorPositionR;
    // double dtR = Timer.getFPGATimestamp() - Constants.Drivetrain.lastTimeStampR;

    // if(Math.abs(errorR) < Constants.Drivetrain.ilimit){
    //   Constants.Drivetrain.errorSumR += errorR*dtR;
    //}

    //double errorRateR = (errorR - Constants.Drivetrain.lastErrorR) / dtR;
    //outputSpeedR = Constants.Drivetrain.kP * errorR + Constants.Drivetrain.kI * Constants.Drivetrain.errorSumR + Constants.Drivetrain.kD * errorRateR;

    //Output to motors
    m_frontLeftMotor.set(outputSpeedL / 1.75);
    m_frontRightMotor.set(outputSpeedL / 2);
    m_rearLeftMotor.set(outputSpeedL / 1.75);
    m_rearRightMotor.set(outputSpeedL / 2);

    //Update variables
    Constants.Drivetrain.lastTimeStampL = Timer.getFPGATimestamp();
    Constants.Drivetrain.lastErrorL = errorL;

    // Constants.Drivetrain.lastTimeStampR = Timer.getFPGATimestamp();
    // Constants.Drivetrain.lastErrorR = errorR;

  }

  
  public void TurnPID(double angle){
    //Get turn position
    Position = ahrs.getYaw();
    
    double current = (angle) - ahrs.getYaw();
    error = current - Position;
    double dt = Timer.getFPGATimestamp() - Constants.Drivetrain.lastTimeStamp;

    if(Math.abs(error) < Constants.Drivetrain.ilimit){
      Constants.Drivetrain.errorSum += error*dt;
    }

    double errorRate = (error - Constants.Drivetrain.lastError) / dt;
    outputSpeed = Constants.Drivetrain.turnP * error + Constants.Drivetrain.turnI * Constants.Drivetrain.errorSum + Constants.Drivetrain.turnD * errorRate;

    //Output to motors
    m_frontLeftMotor.set(outputSpeed / -1.75);
    m_frontRightMotor.set(outputSpeed / 2);
    m_rearLeftMotor.set(outputSpeed / -1.75);
    m_rearRightMotor.set(outputSpeed / 2);

    //Update variables
    Constants.Drivetrain.lastTimeStamp = Timer.getFPGATimestamp();
    Constants.Drivetrain.lastError = error;

  }

  public void resetYAW(){
    ahrs.reset();
  }

  public boolean madeIt(double setpoint){
    return Math.abs(m_frontLeftEncoder.getPosition() - setpoint / 4) < 0.1;

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Front Encoder", m_frontRightEncoder.getPosition() * Constants.Drivetrain.kDistanceConversion);
    SmartDashboard.putNumber("Yaw", ahrs.getYaw());
    SmartDashboard.putNumber("Pitch", ahrs.getPitch());
    SmartDashboard.putNumber("Roll", ahrs.getRoll());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    //builder.addDoubleProperty("Setpoint", () -> m_setpoint, (val) -> m_setpoint = val);
    //builder.addBooleanProperty("At Setpoint", () -> atSetpoint(), null);
    //addChild("Controller", m_controller);
  }

}

// Change the constant for the right hand motor output to make the robot drive straight
// Modify the autonomous so that it drives as close as possible to your cubes, depending on position