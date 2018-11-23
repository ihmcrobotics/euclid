package us.ihmc.euclid.shape;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class CollisionTestResult
{
   private boolean shapesAreColliding;
   private double depth;
   private double distance;

   private final Point3D pointOnA = new Point3D();
   private final Vector3D normalOnA = new Vector3D();

   private final Point3D pointOnB = new Point3D();
   private final Vector3D normalOnB = new Vector3D();

   public CollisionTestResult()
   {
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
