package us.ihmc.euclid.shape.collision.interfaces;

import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface EuclidShape3DCollisionResultReadOnly extends EuclidCollisionResultReadOnly
{
   Shape3DReadOnly getShapeA();

   Shape3DReadOnly getShapeB();

   default boolean epsilonEquals(EuclidShape3DCollisionResultReadOnly other, double epsilon)
   {
      if (getShapeA() != other.getShapeA())
         return false;
      if (getShapeB() != other.getShapeB())
         return false;
      return EuclidCollisionResultReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(EuclidShape3DCollisionResultReadOnly other, double epsilon)
   {
      if (areShapesColliding() != other.areShapesColliding())
         return false;

      if (!EuclidCoreTools.epsilonEquals(getDistance(), other.getDistance(), epsilon))
         return false;

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
}
