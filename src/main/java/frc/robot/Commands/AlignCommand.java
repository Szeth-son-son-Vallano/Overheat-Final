// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignCommand extends Command {
  /** Creates a new AlignCommand. */
  private final DriveSubsystem m_drive;
  private final VisionSubsystem m_vision;
  public AlignCommand(DriveSubsystem drive, VisionSubsystem vision) {
    m_drive = drive;
    m_vision = vision;

    addRequirements(m_drive,m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.align(m_vision.hasRecentTarget(), m_vision.isAligned());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_vision.isAligned() && m_vision.hasRecentTarget();
  }
}
