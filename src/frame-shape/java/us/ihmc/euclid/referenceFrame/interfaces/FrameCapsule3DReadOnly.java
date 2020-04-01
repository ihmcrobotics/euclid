package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeTools;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;

/**
 * Read-only interface for a capsule 3D expressed in given reference frame.
 * <p>
 * A capsule 3D is represented by its length, i.e. the distance separating the center of the two
 * half-spheres, its radius, the position of its center, and its axis of revolution.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameCapsule3DReadOnly extends Capsule3DReadOnly, FrameShape3DReadOnly
{
   /** {@inheritDoc} */
   @Override
   FramePoint3DReadOnly getPosition();

   /** {@inheritDoc} */
   @Override
   FrameVector3DReadOnly getAxis();

   /**
    * {@inheritDoc}
    * <p>
    * Note that the centroid is also the position of this capsule.
    * </p>
    */
   @Override
   default FramePoint3DReadOnly getCentroid()
   {
      return getPosition();
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint3DReadOnly getTopCenter()
   {
      FramePoint3D topCenter = new FramePoint3D(getReferenceFrame());
      topCenter.scaleAdd(getHalfLength(), getAxis(), getPosition());
      return topCenter;
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint3DReadOnly getBottomCenter()
   {
      FramePoint3D bottomCenter = new FramePoint3D(getReferenceFrame());
      bottomCenter.scaleAdd(-getHalfLength(), getAxis(), getPosition());
      return bottomCenter;
   }

   /** {@inheritDoc} */
   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      FrameShape3DReadOnly.super.getBoundingBox(boundingBoxToPack);
   }

   /** {@inheritDoc} */
   @Override
   default void getBoundingBox(ReferenceFrame destinationFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      EuclidFrameShapeTools.boundingBoxCapsule3D(this, destinationFrame, boundingBoxToPack);
   }

   /**
    * Returns {@code null} as this shape is not defined by a pose.
    */
   @Override
   default FrameShape3DPoseReadOnly getPose()
   {
      return null;
   }

   /** {@inheritDoc} */
   @Override
   FixedFrameCapsule3DBasics copy();

   /**
    * Tests on a per component basis if this capsule and {@code other} are equal to an {@code epsilon}.
    * <p>
    * If the two capsules have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other   the other capsule to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two capsules are equal component-wise and are expressed in the same
    *         reference frame, {@code false} otherwise.
    */
   default boolean epsilonEquals(FrameCapsule3DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return Capsule3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two capsules are geometrically
    * similar.
    *
    * @param other   the other capsule to compare against this. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two capsules represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *                                         the same reference frame.
    */
   default boolean geometricallyEquals(FrameCapsule3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Capsule3DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests on a per component basis, if this capsule 3D is exactly equal to {@code other}.
    * <p>
    * If the two capsules have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other capsule 3D to compare against this. Not modified.
    * @return {@code true} if the two capsules are exactly equal component-wise and are expressed in
    *         the same reference frame, {@code false} otherwise.
    */
   default boolean equals(FrameCapsule3DReadOnly other)
   {
      if (other == this)
      {
         return true;
      }
      else if (other == null)
      {
         return false;
      }
      else
      {
         if (getReferenceFrame() != other.getReferenceFrame())
            return false;
         if (getLength() != other.getLength())
            return false;
         if (getRadius() != other.getRadius())
            return false;
         if (!getPosition().equals(other.getPosition()))
            return false;
         if (!getAxis().equals(other.getAxis()))
            return false;
         return true;
      }
   }
}
