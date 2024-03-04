package org.firstinspires.ftc.teamcode.avery;

import java.util.ArrayList;
import java.util.Arrays;

import org.firstinspires.ftc.teamcode.avery.Vector2D;
import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Path {
  public static final double agressiveness = 0.5;
  public static final double deccelRadius = 6;
  public static final double Kstatic = 0.014; //pulled from driveConstants.java

  private static final int STATIC = 0;
  private static final int FOLLOW_PATH = 1;
  private static final int FOLLOW_PATH_REVERSE = 2;
  private static final int FOLLOW_PATH_SEPERATE_END = 3;
  

  public double endHeading;

  public int headingPID_type;
  public static final double heading_P = 0.1;
  
  public Vector2D[] controlPoints;

  public Path(Vector2D start) {
    controlPoints = new Vector2D[]{start};
  }

  public Path(Vector2D[] points) {
    controlPoints = points;
  }

  public double headingPID(Pose2d robot, double distance, Vector2D follow){
    int target;
    switch(headingPID_type){
      case STATIC:{
        target = robot.getHeading();
        break;
      }
      case FOLLOW_PATH:{
        target = follow.getHeading();
        break;
      }
      case FOLLOW_PATH_REVERSE:{
        target = follow.mult(-1).getHeading();
        break;
      }
      case FOLLOW_PATH_SEPERATE_END:{
        if(distance <= deccelRadius){
          target = endHeading;
        }
        else{
          target = follow.getHeading();
        }
        break;
      }
      defualt:{
        break;
      }
    }
    return (robot.getHeading() - target) * heading_P;
    
  }
    
  }

  public path(Vector2D[] controlPoints){
    this.controlPoints = controlPoints;
  }
  
  /**
  * @param t [0 - 1] t parameter
  */
  public Vector2D point(double t){
    return new Vector2D(0, 0);
  }

  /**
  * @param t [0 - 1] t parameter
  */
  public Vector2D derivative(double t){
    return new Vector2D(0, 0);
  }

  /** 
  * @param point a vector representing the robot's coorinates
  */
  public Vector2D closest(Vector2D point){
    return point(closestT(point));
  }

  /**
  * @param point a vector representing the robot's coorinates
  */
  public double closestT(Vector2D point){
    return 0;
  }

  /** 
  * @param point a vector representing the robot's coorinates
  * @returns a unit vector representing the direction of robot travel
  */
  public Vector2D vector(Vector2D point){
    double closestT = closestT(point);
    Vector2D closest = point(closestT);
    Vector2D normal = closest.sub(point);
    
    Vector2D error = new Vector2D(0, 0);
    if(closestT < 1){
      error = derivative(closestT);
    }
    error.normalize();
    error.mult(agressiveness);

    Vector2D output = 
      normal
      .add(error)
      .normalize();

    double dist = distance(point);
    if(dist <= deccelRadius){
      double t = dist / deccelRadius;
      output = 
        output.mult(t)
        .add(output.mult(Kstatic * (1 - t)));
    }
    
    return output;
    
  }


  public Pose2d powers(Pose2d robot){
  
    Vector2D point = new Vector2D(robot.getX(), robot.getY());
    double closestT = closestT(point);
    Vector2D closest = point(closestT);
    Vector2D normal = closest.sub(point);
  
    Vector2D error = new Vector2D(0, 0);
    if(closestT < 1){
      error = derivative(closestT);
    }
    error.normalize();
    error.mult(agressiveness);
  
    Vector2D output = 
      normal
      .add(error)
      .normalize();
  
    double dist = distance(point);
    if(dist <= deccelRadius){
      double t = dist / deccelRadius;
      output = 
        output.mult(t)
        .add(output.mult(Kstatic * (1 - t)));
    }
  
    return new Pose2d(output.x, output.y, headingPID(robot, dist, output));
  
  }

  public double distance(Vector2D point){
    Vector2D endpoint = getEnd();
    return Math.sqrt(Math.pow(endpoint.x - point.x, 2) + Math.pow(endpoint.y - point.y, 2));
  }

  public Vector2D getEnd(){
    return controlPoints[controlPoints.length - 1];
  }

  public boolean equals(Path other){
    return Arrays.equals(this.controlPoints, other.controlPoints);
  }
  
}