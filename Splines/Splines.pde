/* JESUS CHRIST */

/**
 * Splines.
 *
 * Here we use the interpolator.keyFrames() nodes
 * as control points to render different splines.
 *
 * Press ' ' to change the spline mode.
 * Press 'g' to toggle grid drawing.
 * Press 'c' to toggle the interpolator path drawing.
 */

import frames.input.*;
import frames.primitives.*;
import frames.core.*;
import frames.processing.*;
import java.util.List;

// global variables
// modes: 0 natural cubic spline; 1 Hermite;
// 2 (degree 7) Bezier; 3 Cubic Bezier
int mode;

Scene scene;
Interpolator interpolator;
OrbitNode eye;
boolean drawGrid = true, drawCtrl = true;
ArrayList< Vector > herpts  = new ArrayList< Vector >();
ArrayList< Vector > norpts  = new ArrayList< Vector >();
ArrayList< Vector > b3pts  = new ArrayList< Vector >();
ArrayList< Vector > b7pts  = new ArrayList< Vector >();
Vector p0, p1, p2;

//Choose P3D for a 3D scene, or P2D or JAVA2D for a 2D scene
String renderer = P3D;

void setup() {
  size(800, 800, renderer);
  scene = new Scene(this);
  eye = new OrbitNode(scene);
  eye.setDamping(0);
  scene.setEye(eye);
  scene.setFieldOfView(PI / 3);
  //interactivity defaults to the eye
  scene.setDefaultGrabber(eye);
  scene.setRadius(150);
  scene.fitBallInterpolation();
  interpolator = new Interpolator(scene, new Frame());
  // framesjs next version, simply go:
  //interpolator = new Interpolator(scene);

  // Using OrbitNodes makes path editable
  for (int i = 0; i < 8; i++) {
    Node ctrlPoint = new OrbitNode(scene);
    ctrlPoint.randomize();
    interpolator.addKeyFrame(ctrlPoint);
  }
}

void draw() {
  background(175);
  if (drawGrid) {
    stroke(255, 255, 0);
    scene.drawGrid(200, 50);
  }
  if (drawCtrl) {
    fill(255, 0, 0);
    stroke(255, 0, 255);
    for (Frame frame : interpolator.keyFrames())
      scene.drawPickingTarget((Node)frame);
  } else {
    fill(255, 0, 0);
    stroke(255, 0, 255);
    scene.drawPath(interpolator);
  }
  // implement me
  // draw curve according to control polygon an mode
  // To retrieve the positions of the control points do:
  // for(Frame frame : interpolator.keyFrames())
  //   frame.position();
  switch(mode){
    case 0:
    natural( interpolator.keyFrames()  ); 
    break;
    case 1:
    hermite( interpolator.keyFrames() );
    break;
    case 2:
    bezier3( interpolator.keyFrames() );
    break;
    case 3:
    bezier7( interpolator.keyFrames() );
    break;
  }
  
}

void keyPressed() {
  if (key == ' ')
    mode = mode < 3 ? mode+1 : 0;
  if (key == 'g')
    drawGrid = !drawGrid;
  if (key == 'c')
    drawCtrl = !drawCtrl;
}

void natural( List<Frame> pts ){
  int s = pts.size();
  Matrix m = new Matrix(s, s);
  
  for( int i = 0; i < s; i++ ){
    for( int j = 0; j < s; j++ ){
      if( i == j ){
        m.data[i][j] = 4;
      }
      else if( j-i == 1 || i-j == 1 ){
        m.data[i][j] = 1;
      }
    }
  }
  
  m.data[0][0] = 2;
  m.data[s-1][s-1] = 2;
  
  Matrix x = new Matrix( s, 1 );
  Matrix y = new Matrix( s, 1 );
  Matrix z = new Matrix( s, 1 );
  
  for( int i = 1; i < s-1; i++ ){
    x.data[i][0] =  3 * ( pts.get( i+1 ).position().x() - pts.get( i-1 ).position().x() );
    y.data[i][0] =  3 * ( pts.get( i+1 ).position().y() - pts.get( i-1 ).position().y() );
    z.data[i][0] =  3 * ( pts.get( i+1 ).position().z() - pts.get( i-1 ).position().z() );
  }
  
  x.data[0][0] = 3*(pts.get(1).position().x() - pts.get(0).position().x());
  x.data[s-1][0] = 3*(pts.get(s-1).position().x() - pts.get(s-2).position().x());
  y.data[0][0] = 3*(pts.get(1).position().y() - pts.get(0).position().y());
  y.data[s-1][0] = 3*(pts.get(s-1).position().y() - pts.get(s-2).position().y());
  z.data[0][0] = 3*(pts.get(1).position().z() - pts.get(0).position().z());
  z.data[s-1][0] = 3*(pts.get(s-1).position().z() - pts.get(s-2).position().z());
  
  Matrix cx = m.solve(x);
  Matrix cy = m.solve(y);
  Matrix cz = m.solve(z);
  
  for( int i = 0; i < s-1; i++ )
  {
    float pcx = 3*(pts.get(i+1).position().x() - pts.get(i).position().x()) - 2*cx.data[i][0] - cx.data[i+1][0];
    float pdx = 2*(pts.get(i).position().x() - pts.get(i+1).position().x()) + cx.data[i][0] + cx.data[i+1][0];
    float pcy = 3*(pts.get(i+1).position().y() - pts.get(i).position().y()) - 2*cy.data[i][0] - cy.data[i+1][0];
    float pdy = 2*(pts.get(i).position().y() - pts.get(i+1).position().y()) + cy.data[i][0] + cy.data[i+1][0];
    float pcz = 3*(pts.get(i+1).position().z() - pts.get(i).position().z()) - 2*cz.data[i][0] - cz.data[i+1][0];
    float pdz = 2*(pts.get(i).position().z() - pts.get(i+1).position().z()) + cz.data[i][0] + cz.data[i+1][0];
    
    for(int j = 0; j < 1000; j++){
      float t = norm(j, 0, 1000);
      float x_t = pdx * pow( t, 3 ) + pcx * sq( t ) + cx.data[i][0] * t + pts.get(i).position().x();
      float y_t = pdy * pow( t, 3 ) + pcy * sq( t ) + cy.data[i][0] * t + pts.get(i).position().y();
      float z_t = pdz * pow( t, 3 ) + pcz * sq( t ) + cz.data[i][0] * t + pts.get(i).position().z();
      strokeWeight(1);
      stroke(50,100,150);
      point(x_t, y_t, z_t);
    }    
  }
}

void hermite( List<Frame> pts ){ 
  int s = pts.size();
  ArrayList< Vector > rpts = new ArrayList< Vector >();
  for(Frame frame : pts) {
    rpts.add(frame.position());
  }
  
  for( int i = 0; i < s-1; i++ ){
    for( float t = 0; t <= 1.0; t += 0.001 ){
      heval( rpts.get(i), rpts.get(i+1), rpts.get(i), rpts.get(i+1), t );
    }
  }
  
  printitlikeitshot( herpts );
}

void heval( Vector p0, Vector p1, Vector m0, Vector m1, float t ){
  m0 = new Vector( m0.x() * ( t * pow( ( 1 - t ), 2 ) ), m0.y() * ( t * pow(( 1 - t ), 2 ) ) );
  m1 = new Vector( m1.x() * ( pow( t, 2 ) * ( t - 1 ) ) , m1.y() * ( pow( t, 2 ) * ( t - 1 ) ) );
  p0 = new Vector( p0.x() * ( ( 1 + ( 2 * t ) ) * pow( ( 1 - t ), 2 ) ) , p0.y() * ( ( 1 + ( 2 * t ) ) * pow( ( 1 - t ), 2 ) ) );
  p1 = new Vector( p1.x() * ( pow( t, 2 ) * ( 3 - ( 2 * t ) ) ) , p1.y() * ( pow( t, 2 ) * ( 3 - ( 2 * t ) ) ) );
  herpts.add( new Vector( ( p0.x() + p1.x() + m0.x() + m1.x() ), ( p0.y() + p1.y() + m0.y() + m1.y() ) ) );
}

void bezier3( List<Frame> pts ){
  ArrayList< Vector > rpts = new ArrayList< Vector >();
  ArrayList<ArrayList<Vector>> rptsa = new ArrayList<ArrayList<Vector>>();
  for(Frame frame : pts ) {
    rpts.add(frame.position());
  }
  ArrayList<Vector> p1 = new ArrayList<Vector>();
  ArrayList<Vector> p2 = new ArrayList<Vector>();
  for(int i = 0; i < 3; i++){
    p1.add(rpts.get(i));
  }
  for(int i = 3; i < 8; i++){
    p2.add(rpts.get(i));
  }
  rptsa.add(p1);
  rptsa.add(p2);
  
  for( ArrayList p: rptsa ){
    for( float t = 0; t <= 1.0; t += 0.001 ){
      curve( p, t, 3 );
    }
  }
  
  printitlikeitshot( b3pts ); 
}

private Vector getPoint(Vector a, Vector b, float t){
    float x = (1-t)*a.x() + t*b.x();  
    float y = (1-t)*a.y() + t*b.y();  
    float z = (1-t)*a.z() + t*b.z();  
    return new Vector(x,y,z);  
  }

public ArrayList<Vector> curve(ArrayList<Vector> points, float t, int g){
      ArrayList<Vector>rtPoints = new ArrayList<Vector>();

      for(int i=0; i<points.size()-1; i++){
        rtPoints.add(getPoint(points.get(i),points.get(i+1), t));
      }
      if(rtPoints.size() > 1)
        curve(rtPoints, t, g);
      else if(rtPoints.size() == 1){
        if( g == 3 ){
          b3pts.add(rtPoints.get(0));
        }
        else if( g == 7 ){
          b7pts.add(rtPoints.get(0));
        }
      }      
      return rtPoints;      
  }

void printitlikeitshot( ArrayList<Vector> pts ){
  strokeWeight(1);
  stroke(50,100,150);
  for(int i = 0; i < pts.size()-1; i ++){
      Vector Pi = pts.get(i);
      Vector Pj = pts.get(i+1);
      line(Pi.x(),Pi.y(),Pi.z(),Pj.x(), Pj.y(),Pj.z());
  }
}

void bezier7( List<Frame> pts ){
  ArrayList< Vector > rpts = new ArrayList< Vector >();
  ArrayList<ArrayList<Vector>> rptsa = new ArrayList<ArrayList<Vector>>();
  for( Frame frame : pts ) {
    rpts.add(frame.position());
  }
  ArrayList<Vector> ptsb = new ArrayList<Vector>();
    for(int i = 0; i < 8; i++){
      ptsb.add(rpts.get(i));
    }
    rptsa.add(ptsb);
  
  for( ArrayList p: rptsa ){
    for( float t = 0; t <= 1.0; t += 0.001 ){
      curve( p, t, 7 );
    }
  }
  
  printitlikeitshot( b7pts ); 
}