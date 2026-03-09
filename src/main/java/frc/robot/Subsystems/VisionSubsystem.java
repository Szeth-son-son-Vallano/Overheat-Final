// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.opConstants;

public class VisionSubsystem extends SubsystemBase {
  
  public static final Transform3d kRobotToCam = 
     new Transform3d(
     new Translation3d(VisionConstants.camX, VisionConstants.camY, VisionConstants.camZ),
     new Rotation3d(VisionConstants.camRoll, -VisionConstants.camPitch, VisionConstants.camYaw));
  private Matrix<N3,N1> curStdDevs = VisionConstants.kSingleStdDev;
    

  public final PhotonCamera luma = new PhotonCamera (VisionConstants.kCameraName);

  public final PhotonPoseEstimator poseEstimator = 
  new PhotonPoseEstimator(VisionConstants.kTagLayout,
  PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
   kRobotToCam);
  
  private final PIDController alignControl =
   new PIDController(DriveConstants.kAlignP, DriveConstants.kAlignI, DriveConstants.kAlignD);

  private final EstimateConsumer estConsumer;
  private double lastTargetSeenTime;
  
    /**
     * @param estConsumer Lamba that will accept a pose estimate and pass it to your desired {@link
     *     edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator}
     */
     
    /** Creates a new VisionSubsystem. */
    public VisionSubsystem(EstimateConsumer estConsumer) {
      this.estConsumer = estConsumer;
      luma.setFPSLimit(120);
      luma.setDriverMode(false);
      alignControl.setTolerance(1.0);
      lastTargetSeenTime = 0;
  }

  @Override
  public void periodic() {
    var results = luma.getAllUnreadResults();
    double avgYaw = 0;
    int matchingTags = 0;
    boolean targetVisible = false;
    double targetYaw = 0;
    double targetPitch = 0;
    
    boolean isRedAlliance = 
    DriverStation.getAlliance().isPresent()
    && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    
    for(var identified : results){
      visionEst = poseEstimator.estimateCoprocMultiTagPose(identified);
      if (visionEst.isEmpty()){
        visionEst = poseEstimator.estimateLowestAmbiguityPose(identified);
      }
      updateEstimationStdDevs(visionEst,identified.getTargets());
    }

    if (visionEst.isPresent()) {
      var est = visionEst.get();
      var estStdDevs = getEstimationStdDevs();
      Pose2d robotPose = est.estimatedPose.toPose2d();

      double closestDist = Double.MAX_VALUE;
      for (var result : results){
        for (var tgt: result.getTargets()){
          var tagPose = VisionConstants.kTagLayout.getTagPose(tgt.getFiducialId());
          
          if (tagPose.isEmpty()) continue;
          
          targetYaw = tgt.getYaw();
          targetPitch = tgt.getPitch();

          targetVisible = true;

          
          if (targetVisible){
          lastTargetSeenTime = Timer.getFPGATimestamp();
          }

          if (isRedAlliance){
            if (tgt.getFiducialId() >= 8 && tgt.getFiducialId() <= 11)
          {
            avgYaw += tgt.getYaw();
            matchingTags ++;
          }} else {
            if (tgt.getFiducialId() >= 24 && tgt.getFiducialId() <= 27){ 
            avgYaw += tgt.getYaw();
            matchingTags ++;
            }
          }


          double dist = robotPose.getTranslation().getDistance(tagPose.get().toPose2d().getTranslation());
          if (dist < closestDist){ closestDist = dist;}

          VisionConstants.kDistanceToTarget = closestDist;
        } 
      } 
      if (matchingTags > 0){
          avgYaw = avgYaw/matchingTags;
          VisionConstants.rotationOutput = alignControl.calculate(
            avgYaw - VisionConstants.cameraOffset, 0);
          }
      
      estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
    };

    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Is target visible", targetVisible);
    SmartDashboard.putNumber("X Axis error to target ", targetYaw);
    SmartDashboard.putNumber("Y Axis error to target", targetPitch);
    SmartDashboard.putNumber("Distance to target", VisionConstants.kDistanceToTarget);
    SmartDashboard.putNumber("Auto Step", opConstants.autoStep);
  }

  private void updateEstimationStdDevs(Optional<EstimatedRobotPose>estimatedPose, List <PhotonTrackedTarget> targets){
    if(estimatedPose.isEmpty()){
      curStdDevs = VisionConstants.kSingleStdDev;
    } else {
      var estStdDvs = VisionConstants.kSingleStdDev;
      int numTags = 0;
      double avgDist = 0;

      for (var tgt:targets){
        var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist += tagPose.get().toPose2d().getTranslation().getDistance(
          estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0){
        curStdDevs = VisionConstants.kSingleStdDev;
      } else {
        avgDist /= numTags;

        if (numTags > 1){estStdDvs = VisionConstants.kMultiStdDev;}
        if (numTags == 1 && avgDist > 4){
          estStdDvs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          estStdDvs = estStdDvs.times(1 + (avgDist * avgDist/30));
        } 
          curStdDevs = estStdDvs;
      }
    }
  }

  public boolean hasRecentTarget() {
    return (Timer.getFPGATimestamp() - lastTargetSeenTime < 0.5); 
  }

  public boolean isAligned (){
    return (alignControl.atSetpoint());
  }

  public Matrix<N3,N1> getEstimationStdDevs(){
    return curStdDevs;
  }
  @FunctionalInterface
  public static interface EstimateConsumer {
    public void accept(Pose2d pose, double timestamp, Matrix<N3,N1> estimationDevs);
  }
}
