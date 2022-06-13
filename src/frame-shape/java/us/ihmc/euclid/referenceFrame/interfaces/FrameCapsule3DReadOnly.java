package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
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
   FrameUnitVector3DReadOnly getAxis();

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
    * @param object  the other object to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two capsules are equal component-wise and are expressed in the same
    *         reference frame, {@code false} otherwise.
    */
   @Override
   default boolean epsilonEquals(Object object, double epsilon)
   {
      if (!(object instanceof FrameCapsule3DReadOnly))
         return false;
      FrameCapsule3DReadOnly other = (FrameCapsule3DReadOnly) object;
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      return Capsule3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two capsules are geometrically
    * similar.
    *
    * @param object  the other object to compare against this. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two capsules represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *                                         the same reference frame.
    */
   @Override
   default boolean geometricallyEquals(Object object, double epsilon)
   {
      if (!(object instanceof FrameCapsule3DReadOnly))
         return false;
      FrameCapsule3DReadOnly other = (FrameCapsule3DReadOnly) object;
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
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
         if ((getReferenceFrame() != other.getReferenceFrame()) || (getLength() != other.getLength()) || (getRadius() != other.getRadius())
               || !getPosition().equals(other.getPosition()))
            return false;
         if (!getAxis().equals(other.getAxis()))
            return false;
         return true;
      }
   }

   /**
    * Gets a representative {@code String} of {@code capsule3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Capsule 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), length:  0.170, radius:  0.906] - worldFrame
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @return the representative {@code String}.
    */
   @Override
   default String toString(String format)
   {
      return EuclidFrameShapeIOTools.getFrameCapsule3DString(format, this);
   }
}
