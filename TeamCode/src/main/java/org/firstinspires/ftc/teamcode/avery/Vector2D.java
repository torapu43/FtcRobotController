/**
* chainable vector class written by replit ghostwriter
* replit ghostwriter is amazing
*/

class Vector2D{
  public double x;
  public double y;

  public Vector2D(double x, double y){
    this.x = x;
    this.y = y;
  }

  public Vector2D add(Vector2D other){
    return new Vector2D(this.x + other.x, this.y + other.y);
  }

  public Vector2D sub(Vector2D other){
    return new Vector2D(this.x - other.x, this.y - other.y);
  }

  public Vector2D mult(double scalar){
    return new Vector2D(this.x * scalar, this.y * scalar);
  }

  public Vector2D div(double scalar){
    return new Vector2D(this.x / scalar, this.y / scalar);
  }

  public double magnitude(){
    return magnitude = Math.sqrt(Math.pow(this.x, 2) + Math.pow(this.y, 2));

  }

  public Vector2D normalize(){
    double magnitude = magnitude();
    return new Vector2D(this.x / magnitude, this.y / magnitude);
  }

  public Vector2D rotate(double angle){
    angle = Math.toRadians(angle);
    return new Vector2D(this.x * Math.cos(angle) - this.y * Math.sin(angle), this.x * Math.sin(angle) + this.y * Math.cos(angle));
  }

  public Vector2D rotate(double angle, Vector2D origin){
    return this.sub(origin).rotate(angle).add(origin);
  }

  public Vector2D toUnitVector(){
    return this.normalize();
  }

}