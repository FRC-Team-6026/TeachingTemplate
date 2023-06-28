package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.configs.Sparkmax.SwerveModuleInfo;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  private final AHRS gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private boolean isX = false;

  private boolean negativePitch = false;

  public static boolean leveling = false;

  public Swerve() {
    gyro = new AHRS();
    gyro.reset();
    zeroGyro();

    for(int i = 0; i < 3; i++){
        mSwerveMods[i] = new SwerveModule(new SwerveModuleInfo(i));
    }
    
    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getAngle(), getPositions());
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getAngle())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
      for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop, isX);
      var modState = swerveModuleStates[mod.moduleNumber];
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " desired angle: ", modState.angle.getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " desired velocity: ", modState.speedMetersPerSecond);
    }
  }

  public void xPattern(){
    isX = !isX;
  }

  public void xPatternTrue(){
    isX = true;
  }

  public void xPatternFalse(){
    isX = false;
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false, isX);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getAngle(), getPositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPostion();
    }
    return positions;
  }

  public void zeroGyro() {
    gyro.zeroYaw();
    gyro.setAngleAdjustment(0);
    negativePitch = false;
  }

  public void adjustAngle(double angle){
    gyro.setAngleAdjustment(angle);
  }

  public Rotation2d getAngle() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getAngle())
        : Rotation2d.fromDegrees(gyro.getAngle());
  }

  public void resetToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
        mod.resetToAbsolute();
    }
  }

  // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
  return new SequentialCommandGroup(
       new InstantCommand(() -> {
         // Reset odometry for the first path you run during auto
         if(isFirstPath){
             this.resetOdometry(traj.getInitialHolonomicPose());
         }
       }),
       new PPSwerveControllerCommand(
           traj, 
           this::getPose, // Pose supplier
           Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
           new PIDController(Constants.AutoConstants.kPXController, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           new PIDController(Constants.AutoConstants.kPYController, 0, 0), // Y controller (usually the same values as X controller)
           new PIDController(Constants.AutoConstants.kPThetaController, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           this::setModuleStates, // Module states consumer
           true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
           this // Requires this drive subsystem
       )
   );
}

  @Override
  public void periodic() {
    swerveOdometry.update(getAngle(), getPositions());
  }

  public float getPitch(){
    if (negativePitch){
      return -gyro.getPitch();
    } else {
      return gyro.getPitch();
    }
  }

  public void invertGyro(){
    gyro.setAngleAdjustment(180);
    negativePitch = true;
  }

  public AHRS getGyro(){
    return gyro;
  }
}