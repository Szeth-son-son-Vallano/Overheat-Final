// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelConstants;
import frc.robot.Constants.opConstants;

public class FuelSubsystem extends SubsystemBase {
  private final TalonFX leftShooter;
  private final TalonFX rightShooter;
  private final TalonFX indexer;
  private final VelocityVoltage shooterVoltage = new VelocityVoltage(0).withSlot(0);
  public static boolean isStuck = false;
  
    /** Creates a new FuelSubsystem. */
    public FuelSubsystem() {
      leftShooter = new TalonFX(FuelConstants.leftShooterID);
      rightShooter = new TalonFX(FuelConstants.rightShooterID);
      indexer = new TalonFX(FuelConstants.indexerID);
  
      var leftConfig = new TalonFXConfiguration();
      var rightConfig = new TalonFXConfiguration();
      var indexerConfig = new TalonFXConfiguration();
      var currentLimits = new CurrentLimitsConfigs();

      leftConfig.Slot0.kS = FuelConstants.kShooterS; 
      leftConfig.Slot0.kV = FuelConstants.kShooterV; 
      leftConfig.Slot0.kA = FuelConstants.kShooterA;
      leftConfig.Slot0.kP = FuelConstants.kShooterP;
      leftConfig.Slot0.kI = FuelConstants.kShooterI;
      leftConfig.Slot0.kD = FuelConstants.kShooterD;

      leftConfig.Voltage.
      withPeakForwardVoltage(FuelConstants.maxVoltage).withPeakReverseVoltage(-FuelConstants.maxVoltage);
      
      leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      rightConfig.Slot0.kS = FuelConstants.kShooterS;
      rightConfig.Slot0.kV = FuelConstants.kShooterV;
      rightConfig.Slot0.kA = FuelConstants.kShooterA;
      rightConfig.Slot0.kP = FuelConstants.kShooterP;
      rightConfig.Slot0.kI = FuelConstants.kShooterI;
      rightConfig.Slot0.kD = FuelConstants.kShooterD;

      rightConfig.Voltage
      .withPeakForwardVoltage(FuelConstants.maxVoltage).withPeakReverseVoltage(-FuelConstants.maxVoltage);
      rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      currentLimits.withStatorCurrentLimit(60);
      

      leftConfig.withCurrentLimits(currentLimits);
      rightConfig.withCurrentLimits(currentLimits);
      indexerConfig.withCurrentLimits(currentLimits);

      leftShooter.getConfigurator().apply(leftConfig);
      rightShooter.getConfigurator().apply(rightConfig);
      indexer.getConfigurator().apply(indexerConfig);

  
    }
  
    @Override
    public void periodic() {

      if (indexer.getStatorCurrent().getValueAsDouble() > 55){
        isStuck = true;
      } else {
        isStuck = false;
      }
      // This method will be called once per scheduler run
      SmartDashboard.putNumber("MotorSpeed", rightShooter.getVelocity().getValueAsDouble());
      SmartDashboard.putNumber("IndexerSpeed", indexer.getVelocity().getValueAsDouble());
      SmartDashboard.putNumber("TargetSpeed", FuelConstants.targetSpeed); 
      SmartDashboard.putBoolean("is the Intake Stuck? ", isStuck);
    }
  
    public void shoot(){
      leftShooter.setControl(shooterVoltage.withVelocity(-FuelConstants.targetVelocity));
      rightShooter.setControl(shooterVoltage.withVelocity(-FuelConstants.targetVelocity));
    if (leftShooter.getVelocity().getValueAsDouble() < -FuelConstants.targetVelocity + 1){
      indexer.set(FuelConstants.targetSpeed);}
    }
  
    public void intake (){
      leftShooter.set(-FuelConstants.targetSpeed);
      rightShooter.set(-FuelConstants.targetSpeed);
      indexer.set(-FuelConstants.targetSpeed);
    }

    public void outake (){
      leftShooter.set(FuelConstants.targetSpeed);
      rightShooter.set(FuelConstants.targetSpeed);
      indexer.set(FuelConstants.targetSpeed);
    }

    public void pass (){
      leftShooter.setControl(shooterVoltage.withVelocity(-FuelConstants.shootingIntakeSpeed));
      rightShooter.setControl(shooterVoltage.withVelocity(-FuelConstants.shootingIntakeSpeed));
    }

 public void stop(){
  leftShooter.set(0);
  rightShooter.set(0);
  indexer.set(0);
 }

 public void autoShoot (){
  var timer = new Timer();
  timer.restart();
  if (timer.get() < 4){
    if (timer.get() < 1.5){
      leftShooter.setControl(shooterVoltage.withVelocity(-FuelConstants.targetVelocity));
      rightShooter.setControl(shooterVoltage.withVelocity(-FuelConstants.targetVelocity));
    } else if (timer.get() > 1.5) {
      indexer.set(FuelConstants.targetSpeed);
      leftShooter.setControl(shooterVoltage.withVelocity(-FuelConstants.targetVelocity));
      rightShooter.setControl(shooterVoltage.withVelocity(-FuelConstants.targetVelocity));
    }
  } else {
    leftShooter.stopMotor();
    rightShooter.stopMotor();
    indexer.stopMotor();
    opConstants.autoStep++;
  }
 }
}
