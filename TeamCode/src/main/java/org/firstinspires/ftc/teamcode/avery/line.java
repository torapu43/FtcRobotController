class line extends path {
  public line(ArrayList<Vector2D> controlPoints){
    super(controlPoints);
  }

  @override
  public Vector2D point(double t){
    return controlPoints.get(0).mult(1 - t).add(controlPoints.get(1).mult(t));
  }

  @override
  public Vector2D derivative(double t){
    return controlPoints.get(1).sub(controlPoints.get(0));
  }

  /**
  * @param point a vector representing the robot's coorinates
  * @returns a unit vector representing the direction of robot travel
  * replit ghostwriter is amazing 
  */
  @override
  public Vector2D closestT(Vector2D point){
    double l1 = Math.sqrt(Math.pow(point.x - controlPoints.get(0).x, 2) + Math.pow(point.y - controlPoints.get(0).y, 2));
    double alpha = Math.atan2(point.y - controlPoints.get(0).y, point.x - controlPoints.get(0).x) - Math.atan2(controlPoints.get(1).y - controlPoints.get(0).y, controlPoints.get(1).x - controlPoints.get(0).x);
    double beta = Math.PI / 2 - alpha;
    double l2 = Math.sin(beta) * l1;
    double d = Math.sqrt(Math.pow(controlPoints.get(1).x - controlPoints.get(0).x, 2) + Math.pow(controlPoinst.get(1).y - controlPoints.get(0).y, 2));

    return this.point(l2 / d);
  }
}