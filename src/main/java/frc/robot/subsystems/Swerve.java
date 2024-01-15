package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.configs.Sparkmax.SwerveModuleInfo;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  private final Pigeon2 gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private boolean isX = false;

  private boolean negativePitch = false;

  private Field2d field = new Field2d();

  public Swerve() {
    gyro = new Pigeon2(Constants.Setup.pigeonID);
    gyro.reset();
    zeroGyro();

    mSwerveMods = new SwerveModule[4];

    for(int i = 0; i <= 3; i++){
        mSwerveMods[i] = new SwerveModule(new SwerveModuleInfo(i));
    }
    
    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getAngle(), getPositions());

    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetOdometry, 
      this::getSpeeds, 
      this::driveRobotRelative, 
      Constants.Swerve.pathFollowerConfig,
      () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
      },
      this
    );

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic(){
    report();
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
        if(isX){
          mod.setDesiredState(mod.xState, isOpenLoop);
        } else {
          mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
      var modState = swerveModuleStates[mod.moduleNumber];
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " desired angle: ", modState.angle.getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " desired velocity: ", modState.speedMetersPerSecond);
    }
    swerveOdometry.update(getAngle(), getPositions());
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
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
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

  public ChassisSpeeds getSpeeds(){
    return Constants.Swerve.swerveKinematics.toChassisSpeeds(getStates());
  }

  public void zeroGyro() {
    gyro.setYaw(0);
    negativePitch = false;
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

  public float getPitch(){
    if (negativePitch){
      return -(float)gyro.getPitch().getValueAsDouble();
    } else {
      return (float)gyro.getPitch().getValueAsDouble();
    }
  }

  public void invertGyro(){
    gyro.setYaw(180);
    negativePitch = true;
  }

  public Pigeon2 getGyro(){
    return gyro;
  }

  public void report(){
    for (SwerveModule mod : mSwerveMods) {
        SmartDashboard.putNumber(
            "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
        SmartDashboard.putNumber(
            "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
        SmartDashboard.putNumber(
            "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);      
      }
}
}