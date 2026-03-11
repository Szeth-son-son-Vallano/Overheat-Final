// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.opConstants;


public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */

  private final SparkMax leftLeader;
  private final SparkMax rightLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightFollower;

  private final RelativeEncoder leftLeaderEncoder;
  private final RelativeEncoder rightLeaderEncoder;

  private static Pigeon2 drivePigeon;

  private final DifferentialDrive robotDrive;
  private final PIDController driveController;
  private final PIDController alignController;


  private static final double wheelDiameter = Units.inchesToMeters(6);
  private static final double gearRatio = 8.46;
  private static final double positionFactor = ((1/gearRatio) * (Math.PI * wheelDiameter));

  private static Field2d field = new Field2d();

  private static DifferentialDrivePoseEstimator poseEstimator;
  private static DifferentialDriveKinematics driveKinematics;

  private final SparkClosedLoopController leftController;
  private final SparkClosedLoopController rightController;

  private static double distanceTraveled;
  
  
    public DriveSubsystem() {
      leftLeader = new SparkMax(DriveConstants.leftLeaderID, MotorType.kBrushless);
      rightLeader = new SparkMax(DriveConstants.rightLeaderID, MotorType.kBrushless);
      leftFollower = new SparkMax(DriveConstants.leftFollowerID, MotorType.kBrushless);
      rightFollower = new SparkMax(DriveConstants.rightFollowerID, MotorType.kBrushless);
      drivePigeon = new Pigeon2(12);
      driveController = new PIDController
      (DriveConstants.kDriveP, DriveConstants.kDriveI, DriveConstants.kDriveD);
      driveController.setTolerance(0.05);

      alignController = new PIDController
      (DriveConstants.kAlignP, DriveConstants.kAlignI, DriveConstants.kAlignD);
      alignController.setTolerance(0.05);
  
      
  
      leftLeaderEncoder = leftLeader.getEncoder();
      rightLeaderEncoder = rightLeader.getEncoder();
      
  
      var encoderConfig = new EncoderConfig();
      encoderConfig.positionConversionFactor(positionFactor);
      
      var leftConfig = 
      new SparkMaxConfig()
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kCoast)
      .apply(encoderConfig);

      leftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).
      p(DriveConstants.kDriveP).i(DriveConstants.kDriveI).d(DriveConstants.kDriveD).
      outputRange(-1, 1);
      
      var rightConfig = 
      new SparkMaxConfig()
      .inverted(true)
      .smartCurrentLimit(50).idleMode(IdleMode.kCoast)
      .apply(encoderConfig);

      rightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).
      p(DriveConstants.kDriveP).i(DriveConstants.kDriveI).d(DriveConstants.kDriveD).
      outputRange(-1, 1);

      leftController = leftLeader.getClosedLoopController();
      rightController = rightLeader.getClosedLoopController();
      
  
      leftLeader.configure(leftConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
      rightLeader.configure(rightConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
      leftFollower.configure(leftConfig.follow(1),ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
      rightFollower.configure(rightConfig.follow(2),ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
      
      rightLeaderEncoder.setPosition(0);
      leftLeaderEncoder.setPosition(0);

      driveKinematics = new DifferentialDriveKinematics(DriveConstants.kTrackWidthMeters);
      robotDrive = new DifferentialDrive(leftLeader::set, rightLeader::set);
  
      
      poseEstimator = new DifferentialDrivePoseEstimator(
        driveKinematics,
         getHeading(),
         leftLeaderEncoder.getPosition(),
         rightLeaderEncoder.getPosition(),
           new Pose2d()
      );

    drivePigeon.setYaw(0);

    field.setRobotPose(3.272, 4.035, drivePigeon.getRotation2d());
    SmartDashboard.putData("Field " ,field);
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    poseEstimator.update(getHeading(), leftLeaderEncoder.getPosition(), rightLeaderEncoder.getPosition());
    SmartDashboard.putNumber("Left leader velocity: ", leftLeaderEncoder.getVelocity());
    SmartDashboard.putNumber("Right leader velocity: ", rightLeaderEncoder.getVelocity());
    SmartDashboard.putNumber("Pigeon Info", drivePigeon.getYaw().getValueAsDouble());
    SmartDashboard.putNumber ("Distance Traveled", distanceTraveled);
    field.setRobotPose(poseEstimator.getEstimatedPosition());
    
  }

  public void driveArcade (double xSpeed, double xRotation, boolean squared){
    robotDrive.arcadeDrive(xSpeed, xRotation,squared);
  }

  public void driveToTarget(double currentPos, double distanceToTarget){
    distanceTraveled = leftLeaderEncoder.getPosition();
   leftController.setSetpoint(distanceToTarget, SparkBase.ControlType.kPosition);
   rightController.setSetpoint(distanceToTarget, SparkBase.ControlType.kPosition);

   double leftError = Math.abs(currentPos - distanceToTarget);
   double rightError = Math.abs(currentPos - distanceToTarget);

   if (leftError < 0.1 && rightError < 0.1){
    leftLeader.set(0);
    rightLeader.set(0);
    opConstants.autoStep++;
   }
  }
  

  public void resetOdometry (){
    leftLeaderEncoder.setPosition(0);
    rightLeaderEncoder.setPosition(0);
    poseEstimator.resetPosition(
      getHeading(),0.0,0.0, new Pose2d());
  }

  public double getEncoderPose(){
    double leftPos = leftLeaderEncoder.getPosition();
    double rightPos = rightLeaderEncoder.getPosition();

    double currentPos = (leftPos + rightPos) / 2;

    return currentPos;
  }

  public void resetGyro (){
    drivePigeon.setYaw(0);
  }

  public void stopDrive() {
    robotDrive.arcadeDrive(0, 0);
  }

  public void align(boolean hasTarget, boolean isAligned){
    if (!hasTarget){
      robotDrive.arcadeDrive(0, 0);
      return;
    }
    if (!isAligned){
   robotDrive.arcadeDrive(0, VisionConstants.rotationOutput);
    } else {
      robotDrive.arcadeDrive(0, 0);
    }
  }

  public void addVisionMeasurement(
    Pose2d visionMeasurement, double timestampSeconds, Matrix<N3,N1> stdDevs){
      poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds,stdDevs);
    }

  public Pose2d getPose(){
    return poseEstimator.getEstimatedPosition();
  }

  public void rotate(double targetHeading){
    double output = alignController.calculate(
      drivePigeon.getYaw().getValueAsDouble(), targetHeading);
    
    if (!alignController.atSetpoint()){
      robotDrive.arcadeDrive(0, output );
    } else {
      robotDrive.arcadeDrive(0, 0);
      opConstants.autoStep ++;
    }
  }

  public Rotation2d getHeading(){
    return drivePigeon.getRotation2d();
  }

}
