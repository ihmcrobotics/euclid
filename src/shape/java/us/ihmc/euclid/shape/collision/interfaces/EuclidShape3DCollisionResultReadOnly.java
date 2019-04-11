package us.ihmc.euclid.shape.collision.interfaces;

import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for
 * 
 * @author Sylvain Bertrand
 */
public interface EuclidShape3DCollisionResultReadOnly
{
   boolean areShapesColliding();

   double getDistance();

   Shape3DReadOnly getShapeA();

   Shape3DReadOnly getShapeB();

   Point3DReadOnly getPointOnA();

   Point3DReadOnly getPointOnB();

   Vector3DReadOnly getNormalOnA();

   Vector3DReadOnly getNormalOnB();

   default boolean containsNaN()
   {
      if (Double.isNaN(getDistance()))
         return true;
      if (getPointOnA().containsNaN() || getNormalOnA().containsNaN())
         return true;
      if (getPointOnB().containsNaN() || getNormalOnB().containsNaN())
         return true;
      return false;
   }

   default boolean epsilonEquals(EuclidShape3DCollisionResultReadOnly other, double epsilon)
   {
      if (getShapeA() != other.getShapeA())
         return false;
      if (getShapeB() != other.getShapeB())
         return false;
      if (areShapesColliding() != other.areShapesColliding())
         return false;
      if (!EuclidCoreTools.epsilonEquals(getDistance(), other.getDistance(), epsilon))
         return false;

      if (getPointOnA().containsNaN() != other.getPointOnA().containsNaN())
         return false;
      else if (!getPointOnA().epsilonEquals(other.getPointOnA(), epsilon))
         return false;

      if (getPointOnB().containsNaN() != other.getPointOnB().containsNaN())
         return false;
      else if (!getPointOnB().epsilonEquals(other.getPointOnB(), epsilon))
         return false;

      if (getNormalOnA().containsNaN() != other.getNormalOnA().containsNaN())
         return false;
      else if (!getNormalOnA().epsilonEquals(other.getNormalOnA(), epsilon))
         return false;

      if (getNormalOnB().containsNaN() != other.getNormalOnB().containsNaN())
         return false;
      else if (!getNormalOnB().epsilonEquals(other.getNormalOnB(), epsilon))
         return false;
      return true;
   }

   default boolean geometricallyEquals(EuclidShape3DCollisionResultReadOnly other, double epsilon)
   {
      if (areShapesColliding() != other.areShapesColliding())
         return false;

      if (!EuclidCoreTools.epsilonEquals(getDistance(), other.getDistance(), epsilon))
         return false;

      if (getShapeA() != null || getShapeB() != null || other.getShapeA() != null || other.getShapeB() != null)
      {
         boolean swap = getShapeA() != other.getShapeA();
         Shape3DReadOnly otherShapeA = swap ? other.getShapeB() : other.getShapeA();
         Shape3DReadOnly otherShapeB = swap ? other.getShapeA() : other.getShapeB();
         Point3DReadOnly otherPointOnA = swap ? other.getPointOnB() : other.getPointOnA();
         Point3DReadOnly otherPointOnB = swap ? other.getPointOnA() : other.getPointOnB();
         Vector3DReadOnly otherNormalOnA = swap ? other.getNormalOnB() : other.getNormalOnA();
         Vector3DReadOnly otherNormalOnB = swap ? other.getNormalOnA() : other.getNormalOnB();

         if (getShapeA() != otherShapeA)
            return false;
         if (getShapeB() != otherShapeB)
            return false;

         if (getPointOnA().containsNaN() != otherPointOnA.containsNaN())
            return false;
         else if (!getPointOnA().geometricallyEquals(otherPointOnA, epsilon))
            return false;

         if (getPointOnB().containsNaN() != otherPointOnB.containsNaN())
            return false;
         else if (!getPointOnB().geometricallyEquals(otherPointOnB, epsilon))
            return false;

         if (getNormalOnA().containsNaN() != otherNormalOnA.containsNaN())
            return false;
         else if (!getNormalOnA().geometricallyEquals(otherNormalOnA, epsilon))
            return false;

         if (getNormalOnB().containsNaN() != otherNormalOnB.containsNaN())
            return false;
         else if (!getNormalOnB().geometricallyEquals(otherNormalOnB, epsilon))
            return false;

         return true;
      }
      else
      {
         boolean swap = !getPointOnA().geometricallyEquals(other.getPointOnA(), epsilon);
         Point3DReadOnly otherPointOnA = swap ? other.getPointOnB() : other.getPointOnA();
         Point3DReadOnly otherPointOnB = swap ? other.getPointOnA() : other.getPointOnB();
         Vector3DReadOnly otherNormalOnA = swap ? other.getNormalOnB() : other.getNormalOnA();
         Vector3DReadOnly otherNormalOnB = swap ? other.getNormalOnA() : other.getNormalOnB();

         if (getPointOnA().containsNaN() != otherPointOnA.containsNaN())
            return false;
         else if (!getPointOnA().geometricallyEquals(otherPointOnA, epsilon))
            return false;

         if (getPointOnB().containsNaN() != otherPointOnB.containsNaN())
            return false;
         else if (!getPointOnB().geometricallyEquals(otherPointOnB, epsilon))
            return false;

         if (getNormalOnA().containsNaN() != otherNormalOnA.containsNaN())
            return false;
         else if (!getNormalOnA().geometricallyEquals(otherNormalOnA, epsilon))
            return false;

         if (getNormalOnB().containsNaN() != otherNormalOnB.containsNaN())
            return false;
         else if (!getNormalOnB().geometricallyEquals(otherNormalOnB, epsilon))
            return false;

         return true;
      }
   }
}
