package us.ihmc.euclid.shape.collision;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.GeometricallyComparable;
import us.ihmc.euclid.shape.collision.interfaces.EuclidShape3DCollisionResultBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class EuclidShape3DCollisionResult
      implements EuclidShape3DCollisionResultBasics, EpsilonComparable<EuclidShape3DCollisionResult>, GeometricallyComparable<EuclidShape3DCollisionResult>
{
   private boolean shapesAreColliding;
   private double distance;

   private Shape3DReadOnly shapeA;
   private Shape3DReadOnly shapeB;

   private final Point3D pointOnA = new Point3D();
   private final Vector3D normalOnA = new Vector3D();

   private final Point3D pointOnB = new Point3D();
   private final Vector3D normalOnB = new Vector3D();

   public EuclidShape3DCollisionResult()
   {
   }

   @Override
   public void setShapesAreColliding(boolean shapesAreColliding)
   {
      this.shapesAreColliding = shapesAreColliding;
   }

   @Override
   public void setDistance(double distance)
   {
      this.distance = distance;
   }

   @Override
   public void setShapeA(Shape3DReadOnly shapeA)
   {
      this.shapeA = shapeA;
   }

   @Override
   public void setShapeB(Shape3DReadOnly shapeB)
   {
      this.shapeB = shapeB;
   }

   @Override
   public boolean areShapesColliding()
   {
      return shapesAreColliding;
   }

   @Override
   public double getDistance()
   {
      return distance;
   }

   @Override
   public Shape3DReadOnly getShapeA()
   {
      return shapeA;
   }

   @Override
   public Shape3DReadOnly getShapeB()
   {
      return shapeB;
   }

   @Override
   public Point3D getPointOnA()
   {
      return pointOnA;
   }

   @Override
   public Vector3D getNormalOnA()
   {
      return normalOnA;
   }

   @Override
   public Point3D getPointOnB()
   {
      return pointOnB;
   }

   @Override
   public Vector3D getNormalOnB()
   {
      return normalOnB;
   }

   @Override
   public boolean epsilonEquals(EuclidShape3DCollisionResult other, double epsilon)
   {
      return EuclidShape3DCollisionResultBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean geometricallyEquals(EuclidShape3DCollisionResult other, double epsilon)
   {
      return EuclidShape3DCollisionResultBasics.super.geometricallyEquals(other, epsilon);
   }

   @Override
   public String toString()
   {
      return EuclidShapeIOTools.getEuclidShape3DCollisionResultString(this);
   }
}
