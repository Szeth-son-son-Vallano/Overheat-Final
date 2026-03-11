// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ClimbSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.FuelSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.FuelConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.opConstants;
import frc.robot.Constants.towerAutoConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TowerAuto extends Command {
  /** Creates a new CenterAuto. */
  private final VisionSubsystem m_vision;
  private final DriveSubsystem m_drive;
  private final FuelSubsystem m_fuel;
  private final ClimbSubsystem m_climb;

  private final Timer climbTimer = new Timer();
  private boolean climbTimerStarted = false;

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
  public void initialize() {
    opConstants.autoStep = 0;
    m_drive.resetOdometry();
    m_drive.resetGyro();
    climbTimer.stop();
    climbTimer.reset();
    climbTimerStarted = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (opConstants.autoStep) {
      case 0:
      m_climb.climbUp();
      if (!climbTimerStarted){
        climbTimer.restart();
        climbTimerStarted = true;
      }
      if (climbTimer.get() > 4){
      m_drive.driveToTarget(m_drive.getEncoderPose(), towerAutoConstants.driveMeters);
        if (m_drive.getEncoderPose() > towerAutoConstants.driveMeters){
          opConstants.autoStep++;
        }
      } else {
        m_drive.stopDrive();
      }
        break;
      case 1:
        m_drive.align(m_vision.hasRecentTarget(), m_vision.isAligned());
        FuelConstants.targetVelocity = FuelSubsystem.getShooterSpeedFromDistance(VisionConstants.kDistanceToTarget);
        if (m_vision.isAligned()){opConstants.autoStep++;}
        break;
      case 2:
        m_fuel.autoShoot();
        break;
      case 3:
        m_drive.rotate(0);
        break;
      case 4:
        m_climb.climbDown(ClimbConstants.climbDownPos);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopDrive();
    m_fuel.stop();
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
