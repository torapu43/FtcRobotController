class line extends path {
  private Vector2D[] controlPoints;

  public line(Vector2D[] controlPoints){
    super(controlPoints);
  }

  public line(){
    super( new Vector2D[2] );
  }

  public void addStart(Vector2D start){
    this.controlPoints[0] = start;
  }

  public void addEnd(Vector2D end){
    this.controlPoints[1] = end;
  }

  @override
  public Vector2D point(double t){
    return controlPoints[0].mult(1 - t).add(controlPoints[1].mult(t));
  }

  @override
  public Vector2D derivative(double t){
    return controlPoints[1].sub(controlPoints[0]);
  }

  /**
  * @param point a vector representing the robot's coorinates
  * @returns a unit vector representing the direction of robot travel
  * replit ghostwriter is amazing 
  */
  @override
  public Vector2D closestT(Vector2D point){
    double l1 = Math.sqrt(Math.pow(point.x - controlPoints[0].x, 2) + Math.pow(point.y - controlPoints[0].y, 2));
    double alpha = Math.atan2(point.y - controlPoints[0].y, point.x - controlPoints[0].x) - Math.atan2(controlPoints[1].y - controlPoints[0].y, controlPoints[1].x - controlPoints[0].x);
    double beta = Math.PI / 2 - alpha;
    double l2 = Math.sin(beta) * l1;
    double d = Math.sqrt(Math.pow(controlPoints[1].x - controlPoints[0].x, 2) + Math.pow(controlPoints[1].y - controlPoints[0].y, 2));

    return this.point(l2 / d);
  }
}