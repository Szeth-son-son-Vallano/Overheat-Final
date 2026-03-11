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
        public static final int leftLeaderID = 1;
        public static final int rightLeaderID = 2;
        public static final int leftFollowerID = 3;
        public static final int rightFollowerID = 4;

        public static final double kTrackWidthMeters = 0.5207;

        public static final double kAlignP = 0.08;
        public static final double kAlignI = 0.0;
        public static final double kAlignD = 0.03;

        public static final double kDriveP = 0.22;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;

    }

    public class VisionConstants {
        public static final String kCameraName = "Luma P1";
        public static final String kDriveCameraName = "Drive Camera";

        public static final double camPitch = 0.20; //Camera angle
        public static final double camYaw = 0; //Camera position
        public static final double camRoll = 0; // Camera Roll

        public static final double camX = -0.05;
        public static final double camY = 0.5;
        public static final double camZ = 0.3;

        public static final double cameraOffset = 0;

        public static double kDistanceToTarget;
        // Camera position

        public static final AprilTagFieldLayout kTagLayout = 
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        public static final Matrix<N3,N1> kSingleStdDev = VecBuilder.fill(4,4,8);
        public static final Matrix<N3,N1> kMultiStdDev = VecBuilder.fill(0.5,0.5,1);

        public static double rotationOutput = 0;
    }

    public class FuelConstants {
        public static final int leftShooterID = 5;
        public static final int rightShooterID = 6;
        public static final int indexerID = 7;
        public static double targetSpeed = 0.5; //Idexer and intake speed
        public static double shootingIntakeSpeed = 0.25;
        public static double targetVelocity = 75; //ShooterSpeed
        public static double maxVoltage = 12; //10.20
  
        public static final double kShooterP = 0.120; //
        public static final double kShooterI = 0;
        public static final double kShooterD = 0;
        public static final double kShooterS = 0.075;
        public static final double kShooterV = 0.115; //
        public static final double kShooterA = 0.125; //

        // Tune This ASAP
        public static final double[][] kShooterTable = {
            {1.0,68.0}, // {distance meters, RPS}
            {1.5,75.0},
            {2.0,77.0},
            {2.5,87.0},
            {3.0,87.0},
            {3.5,89.0},
            {4.0,90.0},
            {4.5,95.0},
            {5.0,100.0}
        };
    }
    public class ClimbConstants {
        public static final int climberID = 8;
        public static final double climbUpPos = 325;
        public static final double climbDownPos = -0;
        public static final double climbL2Pos = -30;

        public static final double kClimberP = 0.3;
        public static final double kClimberI = 0;
        public static final double kClimberD = 0.006;

        public static final double kClimberMaxReverseVoltage = 9;
        public static final double kClimberMaxVoltage = 9;
    }

    public class depotAutoConstants {

    }
    public class outpostAutoConstants {

    }
    public class towerAutoConstants {
        public static final double drive1Meters = 1; // Meters to tower
        public static final double drive2Meters = 1;
        public static final double driveMeters = 2.42;
        public static final double distanceToTower = 1;
        public static final double rotation1Degrees = -27.41;
        public static final double rotation2Degrees = 13.20;
        public static final double rotation3Degrees = 14.21;

    }

    public class neutralZoneAutoConstants {

    }

    public class opConstants {
        public static final int driveControllerID = 0;
        public static final int opControllerID = 1;
        public static int autoStep = 0;

        public static Pose2d Hub_1 = new Pose2d(4.65,4.0, new Rotation2d()); // Blue Alliance Hub
        public static Pose2d Hub_2 = new Pose2d(12.0, 4.0, new Rotation2d()); //Red Alliance Hub
    }
}
