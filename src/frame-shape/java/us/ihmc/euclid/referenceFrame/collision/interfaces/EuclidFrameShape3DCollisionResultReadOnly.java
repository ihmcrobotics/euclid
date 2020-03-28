package us.ihmc.euclid.referenceFrame.collision.interfaces;

import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
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

   @Override
   FrameShape3DReadOnly getShapeA();

   @Override
   FrameShape3DReadOnly getShapeB();

   /**
    * Tests on a per component basis if {@code other} and {@code this} are equal to an {@code epsilon}.
    *
    * @param other   the other collision result to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two collision results are equal component-wise, {@code false}
    *         otherwise.
    */
   default boolean epsilonEquals(EuclidFrameShape3DCollisionResultReadOnly other, double epsilon)
   {
      if (getPointOnA().getReferenceFrame() != other.getPointOnA().getReferenceFrame())
         return false;
      if (getPointOnB().getReferenceFrame() != other.getPointOnB().getReferenceFrame())
         return false;
      if (getNormalOnA().getReferenceFrame() != other.getNormalOnA().getReferenceFrame())
         return false;
      if (getNormalOnB().getReferenceFrame() != other.getNormalOnB().getReferenceFrame())
         return false;
      return EuclidShape3DCollisionResultReadOnly.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests each feature of {@code this} against {@code other} for geometric similarity.
    *
    * @param other   the other collision result to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each feature.
    * @return {@code true} if the two collision results are considered geometrically similar,
    *         {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code other} does not hold the same reference frames
    *                                         as {@code this}.
    */
   default boolean geometricallyEquals(EuclidFrameShape3DCollisionResultReadOnly other, double epsilon)
   {
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
   default boolean geometricallyEquals(EuclidFrameShape3DCollisionResultReadOnly other, double distanceEpsilon, double pointTangentialEpsilon,
                                       double normalEpsilon)
   {
      boolean swap;

      if (getShapeA() != null || getShapeB() != null || other.getShapeA() != null || other.getShapeB() != null)
         swap = getShapeA() != other.getShapeA();
      else
         swap = getPointOnA().getReferenceFrame() != other.getPointOnA().getReferenceFrame();

      FramePoint3DReadOnly otherPointOnA = swap ? other.getPointOnB() : other.getPointOnA();
      FramePoint3DReadOnly otherPointOnB = swap ? other.getPointOnA() : other.getPointOnB();
      FrameVector3DReadOnly otherNormalOnA = swap ? other.getNormalOnB() : other.getNormalOnA();
      FrameVector3DReadOnly otherNormalOnB = swap ? other.getNormalOnA() : other.getNormalOnB();

      getPointOnA().checkReferenceFrameMatch(otherPointOnA);
      getPointOnB().checkReferenceFrameMatch(otherPointOnB);
      getNormalOnA().checkReferenceFrameMatch(otherNormalOnA);
      getNormalOnB().checkReferenceFrameMatch(otherNormalOnB);

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
   default boolean equals(EuclidFrameShape3DCollisionResultReadOnly other)
   {
      if (other == null)
         return false;
      if (getPointOnA().getReferenceFrame() != other.getPointOnA().getReferenceFrame())
         return false;
      if (getPointOnB().getReferenceFrame() != other.getPointOnB().getReferenceFrame())
         return false;
      if (getNormalOnA().getReferenceFrame() != other.getNormalOnA().getReferenceFrame())
         return false;
      if (getNormalOnB().getReferenceFrame() != other.getNormalOnB().getReferenceFrame())
         return false;
      return EuclidShape3DCollisionResultReadOnly.super.equals(other);
   }
}
