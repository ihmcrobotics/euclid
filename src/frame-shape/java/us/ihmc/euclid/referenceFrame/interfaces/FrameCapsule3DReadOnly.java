package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeTools;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;

public interface FrameCapsule3DReadOnly extends Capsule3DReadOnly, FrameShape3DReadOnly
{
   @Override
   FramePoint3DReadOnly getPosition();

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

   @Override
   default FramePoint3DReadOnly getTopCenter()
   {
      FramePoint3D topCenter = new FramePoint3D(getReferenceFrame());
      topCenter.scaleAdd(getHalfLength(), getAxis(), getPosition());
      return topCenter;
   }

   @Override
   default FramePoint3DReadOnly getBottomCenter()
   {
      FramePoint3D bottomCenter = new FramePoint3D(getReferenceFrame());
      bottomCenter.scaleAdd(-getHalfLength(), getAxis(), getPosition());
      return bottomCenter;
   }

   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      FrameShape3DReadOnly.super.getBoundingBox(boundingBoxToPack);
   }

   @Override
   default FrameShape3DPoseReadOnly getPose()
   {
      return null;
   }

   @Override
   FixedFrameCapsule3DBasics copy();

   default boolean epsilonEquals(FrameCapsule3DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return Capsule3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(FrameCapsule3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Capsule3DReadOnly.super.geometricallyEquals(other, epsilon);
   }

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

   @Override
   default void getBoundingBox(ReferenceFrame destinationFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      EuclidFrameShapeTools.boundingBoxCapsule3D(this, destinationFrame, boundingBoxToPack);
   }
}
