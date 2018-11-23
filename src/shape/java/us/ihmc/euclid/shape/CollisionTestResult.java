package us.ihmc.euclid.shape;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.shape.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class CollisionTestResult implements Clearable
{
   private boolean shapesAreColliding;
   private double depth;
   private double distance;

   private Shape3DReadOnly shapeA;
   private Shape3DReadOnly shapeB;

   private final Point3D pointOnA = new Point3D();
   private final Vector3D normalOnA = new Vector3D();

   private final Point3D pointOnB = new Point3D();
   private final Vector3D normalOnB = new Vector3D();

   public CollisionTestResult()
   {
   }

   @Override
   public void setToZero()
   {
      setShapesAreColliding(false);
      setDepth(0.0);
      setDistance(0.0);
      setShapeA(null);
      setShapeB(null);
      getPointOnA().setToZero();
      getNormalOnA().setToZero();
      getPointOnB().setToZero();
      getNormalOnB().setToZero();
   }

   @Override
   public void setToNaN()
   {
      setShapesAreColliding(false);
      setDepth(Double.NaN);
      setDistance(Double.NaN);
      setShapeA(null);
      setShapeB(null);
      getPointOnA().setToNaN();
      getNormalOnA().setToNaN();
      getPointOnB().setToNaN();
      getNormalOnB().setToNaN();
   }

   @Override
   public boolean containsNaN()
   {
      return Double.isNaN(getDepth()) || Double.isNaN(getDistance()) || getPointOnA().containsNaN() || getNormalOnA().containsNaN()
            || getPointOnB().containsNaN() || getNormalOnB().containsNaN();
   }

   public void swapShapes()
   {
      Shape3DReadOnly tempShape = shapeA;
      shapeA = shapeB;
      shapeB = tempShape;

      double tempX = pointOnA.getX();
      double tempY = pointOnA.getY();
      double tempZ = pointOnA.getZ();

      pointOnA.set(pointOnB);
      pointOnB.set(tempX, tempY, tempZ);

      tempX = normalOnA.getX();
      tempY = normalOnA.getY();
      tempZ = normalOnA.getZ();
      normalOnA.set(normalOnB);
      normalOnB.set(tempX, tempY, tempZ);
   }

   public void setShapesAreColliding(boolean shapesAreColliding)
   {
      this.shapesAreColliding = shapesAreColliding;
   }

   public void setDepth(double depth)
   {
      this.depth = depth;
   }

   public void setDistance(double distance)
   {
      this.distance = distance;
   }

   public void setShapeA(Shape3DReadOnly shapeA)
   {
      this.shapeA = shapeA;
   }

   public void setShapeB(Shape3DReadOnly shapeB)
   {
      this.shapeB = shapeB;
   }

   public boolean areShapesColliding()
   {
      return shapesAreColliding;
   }

   public double getDepth()
   {
      return depth;
   }

   public double getDistance()
   {
      return distance;
   }

   public Shape3DReadOnly getShapeA()
   {
      return shapeA;
   }

   public Shape3DReadOnly getShapeB()
   {
      return shapeB;
   }

   public Point3D getPointOnA()
   {
      return pointOnA;
   }

   public Vector3D getNormalOnA()
   {
      return normalOnA;
   }

   public Point3D getPointOnB()
   {
      return pointOnB;
   }

   public Vector3D getNormalOnB()
   {
      return normalOnB;
   }
}
