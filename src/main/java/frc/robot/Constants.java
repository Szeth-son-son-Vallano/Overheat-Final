// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Add your docs here. */
public class Constants {
    public class DriveConstants{
        // Drive Motors ID
        public static final int leftLeaderID = 1;
        public static final int rightLeaderID = 2;
        public static final int leftFollowerID = 3;
        public static final int rightFollowerID = 4;

        //Track Width
        public static final double kTrackWidthMeters = 0.5207;

        //Auto Align PID Constants
        public static final double kAlignP = 0.08;
        public static final double kAlignI = 0.0;
        public static final double kAlignD = 0.03;

        //Auto Drive PID Constants
        public static final double kDriveP = 0.22;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;

    }

    public class VisionConstants {
        //April Tag Camera ID
        public static final String kCameraName = "Luma P1";
        //Drive Camera ID
        public static final String kDriveCameraName = "Drive Camera";

        //Camera Aim Constants
        public static final double camPitch = 0.20; //Camera angle
        public static final double camYaw = 0; //Camera position
        public static final double camRoll = 0; // Camera Roll
        //Camera Position Constants
        public static final double camX = -0.05;
        public static final double camY = 0.5;
        public static final double camZ = 0.3;

        //Camera Offset
        public static final double cameraOffset = 0;

        //Robot distance to target
        public static double kDistanceToTarget;

        //Field April Tag Layout
        public static final AprilTagFieldLayout kTagLayout = 
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        //Camera Standard Devs
        public static final Matrix<N3,N1> kSingleStdDev = VecBuilder.fill(4,4,8);
        public static final Matrix<N3,N1> kMultiStdDev = VecBuilder.fill(0.5,0.5,1);

        //Align Rotation Output
        public static double rotationOutput = 0;
    }

    public class FuelConstants {
        //Fuel Motors ID
        public static final int leftShooterID = 5;
        public static final int rightShooterID = 6;
        public static final int indexerID = 7;

        //Fuel Speed Constants
        public static double targetSpeed = 0.5; //Idexer and intake speed
        public static double shootingIntakeSpeed = 0.25;
        public static double targetVelocity = 75; //ShooterSpeed
        public static double maxVoltage = 12; //10.20
  
        //Fuel PID Constants
        public static final double kShooterP = 0.120; //
        public static final double kShooterI = 0;
        public static final double kShooterD = 0;
        public static final double kShooterS = 0.075;
        public static final double kShooterV = 0.115; //
        public static final double kShooterA = 0.125; //

        //Distance to shooting power converter
        public static final double[][] kShooterTable = {
            {1.0,75.0}, // {distance meters, RPS}
            {1.5,77.0},
            {2.0,80.0},
            {2.5,83.0},
            {3.0,85.0},
            {3.5,89.0},
            {4.0,92.0},
            {4.5,95.0},
            {5.0,100.0}
        };
    }
    public class ClimbConstants {
        //Climb Motor ID
        public static final int climberID = 8;

        //Declimb Pos
        public static final double climbUpPos = 325;
        //Climb L1 Pos
        public static final double climbL1Pos = -0;
        //Climb L2 Pos
        public static final double climbL2Pos = -0;

        //Climber PID Constants
        public static final double kClimberP = 0.3;
        public static final double kClimberI = 0;
        public static final double kClimberD = 0.006;

        public static final double kClimberMaxReverseVoltage = 9;
        public static final double kClimberMaxVoltage = 9;
    }

    public class towerAutoConstants {
        //Drive Meters
        public static final double driveMeters = 2.35;

    }

    public class neutralZoneAutoConstants {

    }

    public class opConstants {
        //Xbox Controllers Ports
        public static final int driveControllerID = 0;
        public static final int opControllerID = 1;

        //Auto step
        public static int autoStep = 0;

        public static Pose2d Hub_1 = new Pose2d(4.65,4.0, new Rotation2d()); // Blue Alliance Hub
        public static Pose2d Hub_2 = new Pose2d(12.0, 4.0, new Rotation2d()); //Red Alliance Hub
    }
}
