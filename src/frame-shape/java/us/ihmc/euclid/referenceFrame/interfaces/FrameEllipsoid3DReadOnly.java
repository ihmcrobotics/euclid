package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeTools;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;

public interface FrameEllipsoid3DReadOnly extends Ellipsoid3DReadOnly, FrameShape3DReadOnly
{
   FrameVector3DReadOnly getRadii();

   @Override
   FrameShape3DPoseReadOnly getPose();

   @Override
   default FrameRotationMatrixReadOnly getOrientation()
   {
      return getPose().getShapeOrientation();
   }

   @Override
   default FramePoint3DReadOnly getPosition()
   {
      return getPose().getShapePosition();
   }

   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      FrameShape3DReadOnly.super.getBoundingBox(boundingBoxToPack);
   }

   @Override
   default void getBoundingBox(ReferenceFrame destinationFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      EuclidFrameShapeTools.boundingBoxEllipsoid3D(this, destinationFrame, boundingBoxToPack);
   }

   default boolean epsilonEquals(FrameEllipsoid3DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return Ellipsoid3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(FrameEllipsoid3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Ellipsoid3DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   default boolean equals(FrameEllipsoid3DReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return getPose().equals(other.getPose()) && getRadii().equals(other.getRadii());
   }
}
