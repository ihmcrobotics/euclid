package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.shape.primitives.interfaces.Shape3DPoseReadOnly;

public interface FrameShape3DPoseReadOnly extends Shape3DPoseReadOnly, ReferenceFrameHolder
{
   @Override
   FrameRotationMatrixReadOnly getShapeOrientation();

   @Override
   FramePoint3DReadOnly getShapePosition();

   @Override
   FrameVector3DReadOnly getXAxis();

   @Override
   FrameVector3DReadOnly getYAxis();

   @Override
   FrameVector3DReadOnly getZAxis();

   default boolean equals(FrameShape3DPoseReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return getShapePosition().equals(other.getShapePosition()) && getShapeOrientation().equals(other.getShapeOrientation());
   }

   default boolean epsilonEquals(FrameShape3DPoseReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return Shape3DPoseReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(FrameShape3DPoseReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Shape3DPoseReadOnly.super.geometricallyEquals(other, epsilon);
   }
}
