import java.util.ArrayList;
import java.util.Vector2D;

class path {
  private static final double agressiveness = 0.5;
  
  public Vector2D[] controlPoints;

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

    return normal.add(error).normalize();
    
  }

  public boolean equals(path other){
    return this.controlPoints.equals(other.controlPoints);
  }
}