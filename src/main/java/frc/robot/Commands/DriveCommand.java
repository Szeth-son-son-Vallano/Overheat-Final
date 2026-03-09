// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  private final DoubleSupplier m_xSpeed;
  private final DoubleSupplier m_xRotation;
  private final DriveSubsystem m_drive;
  private final BooleanSupplier m_squaredInputs;
  /** Creates a new DriveCommand. */
  public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier xRotation,
  BooleanSupplier squaredInputs) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_xSpeed = xSpeed;
    m_xRotation = xRotation;
    m_drive = driveSubsystem;
    m_squaredInputs = squaredInputs;

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.driveArcade(m_xSpeed.getAsDouble(), m_xRotation.getAsDouble(), m_squaredInputs.getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.driveArcade(0,0,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
