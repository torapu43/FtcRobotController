class Spline extends path {
  private Vector2D term3;
  private Vector2D term4;
  
  public line(ArrayList<Vector2D> controlPoints){
    super(controlPoints);
    this.term2 = 
      controlPoints.get(0).mult(-1)
      .add(controlPoints.get(1).mult(-2))
      .add(controlPoints.get(2).mult(3))
      .add(controlPoints.get(3).mult(-1));

    this.term3 = 
      controlPoints.get(0).mult(2)
      .add(controlPoints.get(1).mult(1))
      .add(controlPoints.get(2).mult(-2))
      .add(controlPoints.get(3).mult(1));
  }

  @override
  public Vector2D point(double t){
    return 
      controlPoints.get(0)
      .add(controlPoints.get(1).mult(t))
      .add(term2.mult(t * t))
      .add(term3.mult(t * t * t));
  }

  @override
  public Vector2D derivative(double t){
    return 
      controlPoints.get(1)
      .add(term2.mult(2 * t))
      .add(term3.mult(3 * t * t));
  }

  /**
  * @param point a vector representing the robot's coorinates
  * @returns a unit vector representing the direction of robot travel
  * replit ghostwriter is amazing 
  */
  @override
  public Vector2D closestT(Vector2D point){
    //TODO: add
    return new Vector2D(0, 0);
  }
}