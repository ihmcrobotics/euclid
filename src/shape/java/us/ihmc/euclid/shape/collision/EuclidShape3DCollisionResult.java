package us.ihmc.euclid.shape.collision;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.GeometricallyComparable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class EuclidShape3DCollisionResult
      implements Clearable, EpsilonComparable<EuclidShape3DCollisionResult>, GeometricallyComparable<EuclidShape3DCollisionResult>, Transformable
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
   public void setToZero()
   {
      setShapesAreColliding(false);
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
      return Double.isNaN(getDistance()) || getPointOnA().containsNaN() || getNormalOnA().containsNaN() || getPointOnB().containsNaN()
            || getNormalOnB().containsNaN();
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

   @Override
   public void applyTransform(Transform transform)
   {
      if (!pointOnA.containsNaN())
         pointOnA.applyTransform(transform);
      if (!pointOnB.containsNaN())
         pointOnB.applyTransform(transform);
      if (!normalOnA.containsNaN())
         normalOnA.applyTransform(transform);
      if (!normalOnB.containsNaN())
         normalOnB.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      if (!pointOnA.containsNaN())
         pointOnA.applyInverseTransform(transform);
      if (!pointOnB.containsNaN())
         pointOnB.applyInverseTransform(transform);
      if (!normalOnA.containsNaN())
         normalOnA.applyInverseTransform(transform);
      if (!normalOnB.containsNaN())
         normalOnB.applyInverseTransform(transform);
   }

   @Override
   public boolean epsilonEquals(EuclidShape3DCollisionResult other, double epsilon)
   {
      if (areShapesColliding() != other.areShapesColliding())
         return false;
      if (!EuclidCoreTools.epsilonEquals(getDistance(), other.getDistance(), epsilon))
         return false;
      if (getShapeA() != other.getShapeA())
         return false;
      if (getShapeB() != other.getShapeB())
         return false;
      if (!getPointOnA().geometricallyEquals(other.getPointOnA(), epsilon))
         return false;
      if (!getNormalOnA().geometricallyEquals(other.getNormalOnA(), epsilon))
         return false;
      if (!getPointOnB().geometricallyEquals(other.getPointOnB(), epsilon))
         return false;
      if (!getNormalOnB().geometricallyEquals(other.getNormalOnB(), epsilon))
         return false;

      return true;
   }

   @Override
   public boolean geometricallyEquals(EuclidShape3DCollisionResult other, double epsilon)
   {
      if (areShapesColliding() != other.areShapesColliding())
         return false;

      if (!EuclidCoreTools.epsilonEquals(getDistance(), other.getDistance(), epsilon))
         return false;

      if (getShapeA() == other.getShapeA())
      {
         if (getShapeB() != other.getShapeB())
            return false;
         if (!getPointOnA().epsilonEquals(other.getPointOnA(), epsilon))
            return false;
         if (!getNormalOnA().epsilonEquals(other.getNormalOnA(), epsilon))
            return false;
         if (!getPointOnB().epsilonEquals(other.getPointOnB(), epsilon))
            return false;
         if (!getNormalOnB().epsilonEquals(other.getNormalOnB(), epsilon))
            return false;
      }
      else
      {
         if (getShapeA() != other.getShapeB())
            return false;
         if (getShapeB() != other.getShapeA())
            return false;
         if (!getPointOnA().epsilonEquals(other.getPointOnB(), epsilon))
            return false;
         if (!getNormalOnA().epsilonEquals(other.getNormalOnB(), epsilon))
            return false;
         if (!getPointOnB().epsilonEquals(other.getPointOnA(), epsilon))
            return false;
         if (!getNormalOnB().epsilonEquals(other.getNormalOnA(), epsilon))
            return false;
      }

      return true;
   }

   @Override
   public String toString()
   {
      return EuclidShapeIOTools.getEuclidShape3DCollisionResultString(this);
   }
}
