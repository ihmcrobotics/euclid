package us.ihmc.euclid.shape.collision.interfaces;

import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for holding the result of a collision query between two shapes.
 *
 * @author Sylvain Bertrand
 */
public interface EuclidShape3DCollisionResultReadOnly extends EuclidGeometry
{
   /**
    * Gets whether the shapes are colliding.
    *
    * @return {@code true} if the shapes are colliding, {@code false} otherwise.
    */
   boolean areShapesColliding();

   /**
    * Gets either the separating distance between the two shapes or the depth of the penetration.
    *
    * @return the value of the distance.
    */
   default double getDistance()
   {
      return Math.abs(getSignedDistance());
   }

   /**
    * Gets either the separating distance between the two shapes or the depth of the penetration.
    * <p>
    * The distance should be signed as follows:
    * <ul>
    * <li>positive in the case the shapes are not colliding, in which case it represents the separating
    * distance.
    * <li>negative in the case the shapes are colliding, in which case it represents the depth of the
    * penetration.
    * </ul>
    * </p>
    *
    * @return the value of the distance.
    */
   double getSignedDistance();

   /**
    * Gets the reference to the first shape.
    *
    * @return the shape A.
    */
   Shape3DReadOnly getShapeA();

   /**
    * Gets the reference to the second shape.
    *
    * @return the shape B.
    */
   Shape3DReadOnly getShapeB();

   /**
    * Gets the key point on the surface of the shape A.
    * <p>
    * In the case the shapes are not colliding, {@code pointOnA} is the closest point to shape B that
    * belongs to shape A.
    * </p>
    * <p>
    * In the case the shapes are colliding, {@code v = pointOnA - pointOnB} is the collision vector of
    * minimum norm that if applied to translate shape A would resolve the collision.
    * </p>
    *
    * @return the key point on shape A.
    */
   Point3DReadOnly getPointOnA();

   /**
    * Gets the key point on the surface of the shape B.
    * <p>
    * In the case the shapes are not colliding, {@code pointOnB} is the closest point to shape A that
    * belongs to shape B.
    * </p>
    * <p>
    * In the case the shapes are colliding, {@code v = pointOnB - pointOnA} is the collision vector of
    * minimum norm that if applied to translate shape B would resolve the collision.
    * </p>
    *
    * @return the key point on shape B.
    */
   Point3DReadOnly getPointOnB();

   /**
    * Gets, if available, the surface normal of shape A at {@code pointOnA}.
    *
    * @return the surface normal at {@code pointOnA}.
    */
   Vector3DReadOnly getNormalOnA();

   /**
    * Gets, if available, the surface normal of shape B at {@code pointOnB}.
    *
    * @return the surface normal at {@code pointOnB}.
    */
   Vector3DReadOnly getNormalOnB();

   /**
    * Tests whether this collision result contains {@link Double#NaN}.
    *
    * @return {@code true} if this contains at least one {@link Double#NaN}, {@code false} otherwise.
    */
   default boolean containsNaN()
   {
      if (Double.isNaN(getSignedDistance()) || getPointOnA().containsNaN() || getNormalOnA().containsNaN())
         return true;
      if (getPointOnB().containsNaN() || getNormalOnB().containsNaN())
         return true;
      return false;
   }

   /**
    * Tests on a per component basis if {@code other} and {@code this} are equal to an {@code epsilon}.
    *
    * @param geometry the other object to compare against this. Not modified.
    * @param epsilon  tolerance to use when comparing each component.
    * @return {@code true} if the two collision results are equal component-wise, {@code false}
    *         otherwise.
    */
   @Override
   default boolean epsilonEquals(EuclidGeometry geometry, double epsilon)
   {
      if (!(geometry instanceof EuclidShape3DCollisionResultReadOnly))
         return false;
      EuclidShape3DCollisionResultReadOnly other = (EuclidShape3DCollisionResultReadOnly) geometry;
      if (getShapeA() != other.getShapeA())
         return false;
      if (getShapeB() != other.getShapeB())
         return false;
      if (areShapesColliding() != other.areShapesColliding())
         return false;
      if (!EuclidCoreTools.epsilonEquals(getSignedDistance(), other.getSignedDistance(), epsilon))
         return false;
      if (getPointOnA().containsNaN() ? !other.getPointOnA().containsNaN() : !getPointOnA().epsilonEquals(other.getPointOnA(), epsilon))
         return false;
      if (getPointOnB().containsNaN() ? !other.getPointOnB().containsNaN() : !getPointOnB().epsilonEquals(other.getPointOnB(), epsilon))
         return false;
      if (getNormalOnA().containsNaN() ? !other.getNormalOnA().containsNaN() : !getNormalOnA().epsilonEquals(other.getNormalOnA(), epsilon))
         return false;
      if (getNormalOnB().containsNaN() ? !other.getNormalOnB().containsNaN() : !getNormalOnB().epsilonEquals(other.getNormalOnB(), epsilon))
         return false;
      return true;
   }

   /**
    * Tests each feature of {@code this} against {@code other} for geometric similarity.
    *
    * @param other   the other collision result to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each feature.
    * @return {@code true} if the two collision results are considered geometrically similar,
    *         {@code false} otherwise.
    */
   @Override
   default boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      if (!(geometry instanceof EuclidShape3DCollisionResultReadOnly))
         return false;
      EuclidShape3DCollisionResultReadOnly other = (EuclidShape3DCollisionResultReadOnly) geometry;
      return geometricallyEquals(other, epsilon, epsilon, epsilon);
   }

   /**
    * Tests each feature of {@code this} against {@code other} for geometric similarity.
    *
    * @param other                  the other collision result to compare against this. Not modified.
    * @param distanceEpsilon        tolerance to use when comparing the distance feature.
    * @param pointTangentialEpsilon tolerance to use when comparing {@code pointOnA} and
    *                               {@code pointOnB} in the plane perpendicular to the collision
    *                               vector, i.e. {@code collisionVector = pointOnA - pointOnB}. The
    *                               {@code distanceEpsilon} is used for comparing the points along the
    *                               collision vector.
    * @param normalEpsilon          tolerance to use when comparing {@code normalOnA} and
    *                               {@code normalOnB}.
    * @return {@code true} if the two collision results are considered geometrically similar,
    *         {@code false} otherwise.
    */
   default boolean geometricallyEquals(EuclidShape3DCollisionResultReadOnly other, double distanceEpsilon, double pointTangentialEpsilon, double normalEpsilon)
   {
      if ((areShapesColliding() != other.areShapesColliding()) || (Double.isNaN(getSignedDistance()) ? !Double.isNaN(other.getSignedDistance())
            : !EuclidCoreTools.epsilonEquals(getSignedDistance(), other.getSignedDistance(), distanceEpsilon)))
         return false;

      boolean swap;

      if (getShapeA() != null || getShapeB() != null || other.getShapeA() != null || other.getShapeB() != null)
         swap = getShapeA() != other.getShapeA();
      else
         swap = !getPointOnA().geometricallyEquals(other.getPointOnA(), pointTangentialEpsilon);

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

      if (getPointOnA().containsNaN() || getPointOnB().containsNaN() || distanceEpsilon == pointTangentialEpsilon)
      {
         if (getPointOnA().containsNaN() ? !otherPointOnA.containsNaN() : !getPointOnA().geometricallyEquals(otherPointOnA, distanceEpsilon))
            return false;

         if (getPointOnB().containsNaN() ? !otherPointOnB.containsNaN() : !getPointOnB().geometricallyEquals(otherPointOnB, distanceEpsilon))
            return false;
      }
      else
      {
         double collisionVectorX = (getPointOnA().getX() - getPointOnB().getX()) / getDistance();
         double collisionVectorY = (getPointOnA().getY() - getPointOnB().getY()) / getDistance();
         double collisionVectorZ = (getPointOnA().getZ() - getPointOnB().getZ()) / getDistance();

         if (!EuclidShapeTools.geometricallyEquals(getPointOnA(),
                                                   otherPointOnA,
                                                   collisionVectorX,
                                                   collisionVectorY,
                                                   collisionVectorZ,
                                                   distanceEpsilon,
                                                   pointTangentialEpsilon))
            return false;
         if (!EuclidShapeTools.geometricallyEquals(getPointOnB(),
                                                   otherPointOnB,
                                                   collisionVectorX,
                                                   collisionVectorY,
                                                   collisionVectorZ,
                                                   distanceEpsilon,
                                                   pointTangentialEpsilon))
            return false;
      }

      if (getNormalOnA().containsNaN() ? !otherNormalOnA.containsNaN() : !getNormalOnA().geometricallyEquals(otherNormalOnA, normalEpsilon))
         return false;

      if (getNormalOnB().containsNaN() ? !otherNormalOnB.containsNaN() : !getNormalOnB().geometricallyEquals(otherNormalOnB, normalEpsilon))
         return false;

      return true;
   }

   /**
    * Tests on a per component basis, if this collision result is exactly equal to {@code other}.
    *
    * @param geometry the EuclidGeometry to compare against this. Not modified.
    * @return {@code true} if the two collision results are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   @Override
   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
      {
         return true;
      }
      else if (geometry == null || !(geometry instanceof EuclidShape3DCollisionResultReadOnly))
      {
         return false;
      }
      else
      {
         EuclidShape3DCollisionResultReadOnly other = (EuclidShape3DCollisionResultReadOnly) geometry;
         if (areShapesColliding() != other.areShapesColliding())
            return false;
         if (Double.compare(getSignedDistance(), other.getSignedDistance()) != 0)
            return false;
         if (getShapeA() != other.getShapeA())
            return false;
         if (getShapeB() != other.getShapeB())
            return false;
         if (!getPointOnA().equals(other.getPointOnA()))
            return false;
         if (!getNormalOnA().equals(other.getNormalOnA()))
            return false;
         if (!getPointOnB().equals(other.getPointOnB()))
            return false;
         if (!getNormalOnB().equals(other.getNormalOnB()))
            return false;
         return true;
      }
   }

   /**
    * Gets the representative {@code String} of {@code euclidShape3DCollisionResult} given a specific
    * format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:<br>
    * When shapes are colliding:
    *
    * <pre>
    * Collision test result: colliding, depth: 0.539
    * Shape A: Box3D, location: ( 0.540,  0.110,  0.319 ), normal: ( 0.540,  0.110,  0.319 )
    * Shape B: Capsule3D, location: ( 0.540,  0.110,  0.319 ), normal: ( 0.540,  0.110,  0.319 )
    * </pre>
    *
    * When shapes are not colliding:
    *
    * <pre>
    * Collision test result: non-colliding, separating distance: 0.539
    * Shape A: Box3D, location: ( 0.540,  0.110,  0.319 ), normal: ( 0.540,  0.110,  0.319 )
    * Shape B: Capsule3D, location: ( 0.540,  0.110,  0.319 ), normal: ( 0.540,  0.110,  0.319 )
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @return the representative {@code String}.
    */
   @Override
   default String toString(String format)
   {
      return EuclidShapeIOTools.getEuclidShape3DCollisionResultString(format, this);
   }
}
