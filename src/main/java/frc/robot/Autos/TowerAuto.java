// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ClimbSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.FuelSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Constants.opConstants;
import frc.robot.Constants.towerAutoConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TowerAuto extends Command {
  /** Creates a new CenterAuto. */
  private final VisionSubsystem m_vision;
  private final DriveSubsystem m_drive;
  private final FuelSubsystem m_fuel;
  private final ClimbSubsystem m_climb;

  public TowerAuto(VisionSubsystem vision, DriveSubsystem drive, FuelSubsystem fuel, ClimbSubsystem climb) {
    m_vision = vision;
    m_drive = drive;
    m_fuel = fuel;
    m_climb = climb;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(m_vision, m_drive, m_fuel, m_climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (opConstants.autoStep == 1){
      m_drive.driveToTarget(towerAutoConstants.drive1Meters);
      m_climb.climbUp();
    } else if (opConstants.autoStep == 2){
      m_drive.stopDrive();
      m_drive.align(m_vision.hasRecentTarget(), m_vision.isAligned());
      m_fuel.autoShoot();
    } else if (opConstants.autoStep == 3){
      m_fuel.stop();
      m_drive.align(m_vision.hasRecentTarget(), m_vision.isAligned());
    } else if (opConstants.autoStep == 4){
      m_drive.driveToTarget(towerAutoConstants.drive2Meters);     
    } else if (opConstants.autoStep == 5){
      m_climb.climbDown();
    } else {
      m_drive.align(m_vision.hasRecentTarget(),m_vision.isAligned());
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climb.climbUp();
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
