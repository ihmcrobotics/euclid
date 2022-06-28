package us.ihmc.euclid.referenceFrame.collision.interfaces;

import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.shape.collision.interfaces.EuclidShape3DCollisionResultReadOnly;

/**
 * Read-only interface adding {@code ReferenceFrame} support to
 * {@link EuclidShape3DCollisionResultReadOnly}.
 *
 * @author Sylvain Bertrand
 */
public interface EuclidFrameShape3DCollisionResultReadOnly extends EuclidShape3DCollisionResultReadOnly
{
   /** {@inheritDoc} */
   @Override
   FramePoint3DReadOnly getPointOnA();

   /** {@inheritDoc} */
   @Override
   FramePoint3DReadOnly getPointOnB();

   /** {@inheritDoc} */
   @Override
   FrameVector3DReadOnly getNormalOnA();

   /** {@inheritDoc} */
   @Override
   FrameVector3DReadOnly getNormalOnB();

   /** {@inheritDoc} */
   @Override
   FrameShape3DReadOnly getShapeA();

   /** {@inheritDoc} */
   @Override
   FrameShape3DReadOnly getShapeB();

   /** {@inheritDoc} */
   @Override
   default boolean epsilonEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;

      if (geometry instanceof EuclidFrameShape3DCollisionResultReadOnly)
      {
         EuclidFrameShape3DCollisionResultReadOnly other = (EuclidFrameShape3DCollisionResultReadOnly) geometry;
         if (getPointOnA().getReferenceFrame() != other.getPointOnA().getReferenceFrame())
            return false;
         if (getPointOnB().getReferenceFrame() != other.getPointOnB().getReferenceFrame())
            return false;
         if (getNormalOnA().getReferenceFrame() != other.getNormalOnA().getReferenceFrame())
            return false;
         if (getNormalOnB().getReferenceFrame() != other.getNormalOnB().getReferenceFrame())
            return false;
      }

      return EuclidShape3DCollisionResultReadOnly.super.epsilonEquals(geometry, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;

      if (geometry instanceof EuclidFrameShape3DCollisionResultReadOnly)
         return geometricallyEquals((EuclidFrameShape3DCollisionResultReadOnly) geometry, epsilon, epsilon, epsilon);
      else if (geometry instanceof EuclidShape3DCollisionResultReadOnly)
         return geometricallyEquals((EuclidShape3DCollisionResultReadOnly) geometry, epsilon, epsilon, epsilon);
      else
         return false;
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
   default boolean geometricallyEquals(EuclidFrameShape3DCollisionResultReadOnly other,
                                       double distanceEpsilon,
                                       double pointTangentialEpsilon,
                                       double normalEpsilon)
   {
      if (other == this)
         return true;
      if (other == null)
         return false;

      boolean swap;

      if (getShapeA() != null || getShapeB() != null || other.getShapeA() != null || other.getShapeB() != null)
         swap = getShapeA() != other.getShapeA();
      else
         swap = getPointOnA().getReferenceFrame() != other.getPointOnA().getReferenceFrame();

      FramePoint3DReadOnly otherPointOnA = swap ? other.getPointOnB() : other.getPointOnA();
      FramePoint3DReadOnly otherPointOnB = swap ? other.getPointOnA() : other.getPointOnB();
      FrameVector3DReadOnly otherNormalOnA = swap ? other.getNormalOnB() : other.getNormalOnA();
      FrameVector3DReadOnly otherNormalOnB = swap ? other.getNormalOnA() : other.getNormalOnB();

      if (getPointOnA().getReferenceFrame() != otherPointOnA.getReferenceFrame())
         return false;
      if (getPointOnB().getReferenceFrame() != otherPointOnB.getReferenceFrame())
         return false;
      if (getNormalOnA().getReferenceFrame() != otherNormalOnA.getReferenceFrame())
         return false;
      if (getNormalOnB().getReferenceFrame() != otherNormalOnB.getReferenceFrame())
         return false;

      return EuclidShape3DCollisionResultReadOnly.super.geometricallyEquals(other, distanceEpsilon, pointTangentialEpsilon, normalEpsilon);
   }

   /**
    * Tests on a per component basis, if this collision result is exactly equal to {@code other}.
    * <p>
    * Two instances of collision frame results are not considered equal when their respective frames
    * are different.
    * </p>
    *
    * @param other the other collision result to compare against this. Not modified.
    * @return {@code true} if the two collision results are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   @Override
   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
         return true;

      if (geometry instanceof EuclidFrameShape3DCollisionResultReadOnly)
      {
         EuclidFrameShape3DCollisionResultReadOnly other = (EuclidFrameShape3DCollisionResultReadOnly) geometry;
         if (getPointOnA().getReferenceFrame() != other.getPointOnA().getReferenceFrame())
            return false;
         if (getPointOnB().getReferenceFrame() != other.getPointOnB().getReferenceFrame())
            return false;
         if (getNormalOnA().getReferenceFrame() != other.getNormalOnA().getReferenceFrame())
            return false;
         if (getNormalOnB().getReferenceFrame() != other.getNormalOnB().getReferenceFrame())
            return false;
      }

      return EuclidShape3DCollisionResultReadOnly.super.equals(geometry);
   }

   /**
    * Gets the representative {@code String} of this collision result given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:<br>
    * When shapes are colliding:
    *
    * <pre>
    * Collision test result: colliding, depth: 0.539
    * Shape A: Box3D - shapeFrameA, location: ( 0.540,  0.110,  0.319 ) - locationFrameA, normal: ( 0.540,  0.110,  0.319 ) - normalFrameB
    * Shape B: Capsule3D - shapeFrameB, location: ( 0.540,  0.110,  0.319 ) - locationFrameB, normal: ( 0.540,  0.110,  0.319 ) - normalFrameB
    * </pre>
    *
    * When shapes are not colliding:
    *
    * <pre>
    * Collision test result: non-colliding, separating distance: 0.539
    * Shape A: Box3D - shapeFrameA, location: ( 0.540,  0.110,  0.319 ) - locationFrameA, normal: ( 0.540,  0.110,  0.319 ) - normalFrameB
    * Shape B: Capsule3D - shapeFrameB, location: ( 0.540,  0.110,  0.319 ) - locationFrameB, normal: ( 0.540,  0.110,  0.319 ) - normalFrameB
    * </pre>
    * </p>
    *
    * @param format                       the format to use for each number.
    * @param euclidShape3DCollisionResult the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   @Override
   default String toString(String format)
   {
      return EuclidFrameShapeIOTools.getEuclidFrameShape3DCollisionResultString(format, this);
   }
}
