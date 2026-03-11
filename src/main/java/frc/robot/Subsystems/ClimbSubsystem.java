// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  private final TalonFX climbMotor;
  private final PositionVoltage climbVoltage = new PositionVoltage(0).withSlot(1);
  public static boolean isFinished = false;
  public ClimbSubsystem() {
    climbMotor = new TalonFX(ClimbConstants.climberID);

    var climbConfig = new TalonFXConfiguration();
    var climbCurrentLimits = new CurrentLimitsConfigs();

    climbConfig.Slot1.kP = ClimbConstants.kClimberP;
    climbConfig.Slot1.kI = ClimbConstants.kClimberI;
    climbConfig.Slot1.kD = ClimbConstants.kClimberD;

    climbConfig.Voltage.withPeakForwardVoltage(ClimbConstants.kClimberMaxVoltage)
    .withPeakReverseVoltage(-ClimbConstants.kClimberMaxReverseVoltage);

    climbCurrentLimits.withStatorCurrentLimit(60);

    climbConfig.withCurrentLimits(climbCurrentLimits);

    climbMotor.getConfigurator().apply(climbConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Position ", climbMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Climb target ", ClimbConstants.climbUpPos);
    SmartDashboard.putNumber("Climb down target ", ClimbConstants.climbDownPos);
  }

  public void climbUp(){
    climbMotor.setControl(climbVoltage.withPosition(ClimbConstants.climbUpPos)); // get the claw in position
  }

  public void climbDown (double targetPos){
    climbMotor.setControl(climbVoltage.withPosition(targetPos)); // start to climb
  }

  public void setZero(){
    climbMotor.setControl(climbVoltage.withPosition(0));
  }

  public void returnToZero (Double targetPos){
    climbMotor.setControl(climbVoltage.withPosition(targetPos));
  }

  public void stopClimb (){
    climbMotor.stopMotor();
  }
}
