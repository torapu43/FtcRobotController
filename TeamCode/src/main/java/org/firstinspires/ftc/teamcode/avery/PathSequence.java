import java.util.Arrays;

class PathSequence {
  public static final double deadBand = 0.1;
  
  public ArrayList<Path> paths;
  public int currentPath;
  public Vector2D start;

  public PathSequence(Path[] paths){
    this.start = paths[0];
    for(Path path : paths){
      this.paths.add(path);
    }
  }

  public PathSequence(Vector2D start){
    this.start = start;
  }

  public PathSequence LineTo(double x, double y){
    Vector2D end = new Vector2D(x, y);
    Vector2D start = this.start;
    if(paths.size() == 0){
    }
    else{
      start = paths.get(paths.size() - 1).getEnd();
    }

    Path[] output = Arrays.copyOf(getPaths, paths.size() + 1);
    output[paths.size()] = new Line(start, end);
    return new PathSequence(output);
  }

  public PathSequence LineTo(Vector2D end){
    Vector2D start = this.start;
    if(paths.size() == 0){
    }
    else{
      start = paths.get(paths.size() - 1).getEnd();
    }

    Path[] output = Arrays.copyOf(getPaths, paths.size() + 1);
    output[paths.size()] = new Line(start, end);
    return new PathSequence(output);
  }

  public Path[] getPaths(){
    Path[] output = new Path[paths.size()];

    for(int i = 0; i < paths.size(); i++){
      output[i] = paths.get(i);
    }
    return output;
  }

  public PathSequence SplineTo(double x, double y, double cx, double cy, double startVelo){
    Path last = paths.get(paths.size() - 1);
    Vector2D lastEnd = last.controlPoints[last.controlPoints.length - 1];
    Vector2D lastStart = last.controlPoints[last.controlPoints.length - 2];
    
    Vector2D end = new Vector2D(x, y);

    vector2D controlPoint1 = 
      lastEnd.sub(lastStart)
      .normalize()
      .mult(startVelo);
      
    Vector2D controlPoint2 = new Vector2D(cx - x, cy - y);
    Path[] output = Arrays.copyOf(getPaths, paths.size() + 1);
    output[paths.size()] = new Spline(lastEnd, controlPoint1, controlPoint2, end);
    return new PathSequence(output);
  }

  public PathSequence SplineTo(Vector2D end, Vector2D control, double startVelo){
    Path last = paths.get(paths.size() - 1);
    Vector2D lastEnd = last.controlPoints[last.controlPoints.length - 1];
    Vector2D lastStart = last.controlPoints[last.controlPoints.length - 2];


    vector2D controlPoint1 = 
      lastEnd.sub(lastStart)
      .normalize()
      .mult(startVelo);

    Vector2D controlPoint2 = control.sub(end);
    Path[] output = Arrays.copyOf(getPaths, paths.size() + 1);
    output[paths.size()] = new Spline(lastEnd, controlPoint1, controlPoint2, end);
    return new PathSequence(output);
  }

  public PathSequence SplineTo(Vector2D start, Vector2D control1, Vector2D end, Vector2D control2){

    control1 = control1.sub(start);
    control2 = control2.sub(end);

    Path[] output = Arrays.copyOf(getPaths, paths.size() + 1);
    output[paths.size()] = new Spline(start, controlPoint1, controlPoint2, end);
    return new PathSequence(output);
  }

  public PathSequence SplineTo(double startX, double startY, double cx1, double cy1, double endX, double endY, double cx2, double cy2){
    return SplineTo(new Vector2D(startX, startY), new Vector2D(cx1, cy1), new Vector2D(endX, endY), new Vector2D(cx2, cy2));

  }

  public PathSequence SplineTo(Spline spline){
    Path[] output = Arrays.copyOf(getPaths, paths.size() + 1);
    output(paths.size()) = spline;
    return new PathSequence(output);
  }
  
}