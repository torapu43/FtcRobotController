import java.util.ArrayList;
import java.util.Vector2D;

class path {
  public static final double agressiveness = 0.5;
  public static final double deccelRadius = 6;
  public static final double Kstatic = 0.014; //pulled from driveConstants.java
  
  public Vector2D[] controlPoints;

  public headingPID(Pose2D robot){
    
  }

  public headingPID(Pose2D robot, double distance, double end){
    
  }

  public headingPID(Pose2D robot, double distance, double end, double transit){
    
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
    return new Vector2d(0, 0);
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
  public Vector2D closestT(Vector2D point){
    return new Vector2D(0, 0);
  }

  /** 
  * @param point a vector representing the robot's coorinates
  * @returns a unit vector representing the direction of robot travel
  */
  public Vector2D vector(Vector2D point){
    Vector2D closestT = closestT(point);
    Vector2D closest = point(closestT);
    Vector2D normal = closest.sub(point);
    
    Vector2D error = newVector2D(0, 0);
    if(closestT < 1){
      error = derivative(closestT);
    }
    error.normalize();
    error.mult(aggressiveness);

    Vector2D output = 
      normal
      .add(error)
      .normalize();

    double dist = distance(point);
    if(dist <= decelRadius){
      double t = dist / decelRadius;
      output = 
        output.mult(t)
        .add(output.mult(Kstatic * (1 - t)));
    }
    
    return output;
    
  }

  public Vector2D vector(Pose2D point){
    return vector(new Vector2D(point.getX(), point.getY()));
  }

  public double distance(Vector2D point){
    Vector2D endpoint = controlPoints[controlPoints.length - 1];
    return Math.sqrt(Math.pow(endpoint.x - point.x, 2) + Math.pow(endpoint.y - point.y, 2));
  }

  public Vector2D getEnd(){
    return controlPoints[controlPoints.length - 1];
  }

  public boolean equals(path other){
    return this.controlPoints.equals(other.controlPoints);
  }
  
}