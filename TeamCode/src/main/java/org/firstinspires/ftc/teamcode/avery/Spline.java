package org.firstinspires.ftc.teamcode.avery;

import org.firstinspires.ftc.teamcode.avery.Path;
import org.firstinspires.ftc.teamcode.avery.Vector2D;

class Spline extends Path {
  private Vector2D[] controlPoints = new Vector2D[4];
  private Vector2D term2;
  private Vector2D term3;
  
  public Spline(Vector2D[] controlPoints){
    super(controlPoints);

  }

  public Spline(){
    super(new Vector2D[4]);
    //controlPoints = new Vector2D[4];
  }

  public Spline(Vector2D start, Vector2D cp1, Vector2D cp2, Vector2D end){
    super(new Vector2D[]{start, cp1, cp1, end});
  }

  public void addStart(Vector2D start){
    this.controlPoints[0] = start;
  }

  public void addEnd(Vector2D end){
    this.controlPoints[3] = end;
  }

  public void addControlPoint(int index, Vector2D controlPoint){
    controlPoints[index] = controlPoint;
  }

  public Spline withStart(double x, double y){
    return withStart(new Vector2D(x, y));
  }
  public Spline withStart(Vector2D start){
    Spline output = this.copy();
    output.addStart(start);
    return output;
  }

  public Spline withEnd(double x, double y){
    return withEnd(new Vector2D(x, y));
  }

  public Spline withEnd(Vector2D end){
    Spline output = this.copy();
    output.addEnd(end);



    return output;
  }

  public Spline withControlPoint(int index, int x, int y){
    Spline output = this.copy();
    output.addControlPoint(index, new Vector2D(x, y));
    return output;
  }

  public Spline copy(){
    return new Spline(this.controlPoints.clone());
  }

  public void addStartVelocity(Vector2D controlPoint){
    this.controlPoints[1] = controlPoint;
  }

  public Vector2D point(double t){
    controlPoints = getControlPoints();
    term2 =
            controlPoints[0].mult(-1)
                    .add(controlPoints[1].mult(-2))
                    .add(controlPoints[2].mult(3))
                    .add(controlPoints[3].mult(-1));

    term3 =
            controlPoints[0].mult(2)
                    .add(controlPoints[1].mult(1))
                    .add(controlPoints[2].mult(-2))
                    .add(controlPoints[3].mult(1));
    return
      controlPoints[0]
      .add(controlPoints[1].mult(t))
      .add(term2.mult(t * t))
      .add(term3.mult(t * t * t));
  }

  public Vector2D derivative(double t){
    controlPoints = getControlPoints();
    term2 =
            controlPoints[0].mult(-1)
                    .add(controlPoints[1].mult(-2))
                    .add(controlPoints[2].mult(3))
                    .add(controlPoints[3].mult(-1));

    term3 =
            controlPoints[0].mult(2)
                    .add(controlPoints[1].mult(1))
                    .add(controlPoints[2].mult(-2))
                    .add(controlPoints[3].mult(1));
    return 
      controlPoints[1]
      .add(term2.mult(2 * t))
      .add(term3.mult(3 * t * t));
  }

  public Spline build(){
    Spline output = copy();
    if(
       output.controlPoints[0] == null
       || output.controlPoints[0] == null
       || output.controlPoints[0] == null
       || output.controlPoints[0] == null
    ){
      return output;
    }
    output.term2 =
            output.controlPoints[0].mult(-1)
                    .add(output.controlPoints[1].mult(-2))
                    .add(output.controlPoints[2].mult(3))
                    .add(output.controlPoints[3].mult(-1));

    output.term3 =
            output.controlPoints[0].mult(2)
                    .add(output.controlPoints[1].mult(1))
                    .add(output.controlPoints[2].mult(-2))
                    .add(output.controlPoints[3].mult(1));

    return output;
  }

  /**
  * @param point a vector representing the robot's coorinates
  * @returns a unit vector representing the direction of robot travel
  */
  public double closestT(Vector2D point){
    double epsilon = 1.0 / 100;

    double closest = Integer.MAX_VALUE;
    double closestT = 1;

    for(double t = 0; t < 1; t ++){
      double dist = point.dist(this.point(t));

      if(dist <= closest){
        closest = dist;
        closestT = t;
      }
    }
    return closestT;
  }
}