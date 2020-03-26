package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface FixedFrameSphere3DBasics extends Sphere3DBasics, FrameSphere3DReadOnly, FixedFrameShape3DBasics
{
   @Override
   FixedFramePoint3DBasics getPosition();

   default void set(ReferenceFrame referenceFrame, Sphere3DReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(other);
   }

   default void set(FrameSphere3DReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   default void set(ReferenceFrame referenceFrame, double centerX, double centerY, double centerZ, double radius)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(centerX, centerY, centerZ, radius);
   }

   default void set(ReferenceFrame referenceFrame, Point3DReadOnly center, double radius)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(center, radius);
   }

   default void set(FramePoint3DReadOnly center, double radius)
   {
      set(center.getReferenceFrame(), center, radius);
   }

   default void setMatchingFrame(ReferenceFrame referenceFrame, Sphere3DReadOnly other)
   {
      Sphere3DBasics.super.set(other);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setMatchingFrame(FrameSphere3DReadOnly other)
   {
      setMatchingFrame(other.getReferenceFrame(), other);
   }

   default void setMatchingFrame(ReferenceFrame referenceFrame, double centerX, double centerY, double centerZ, double radius)
   {
      set(centerX, centerY, centerZ, radius);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setMatchingFrame(ReferenceFrame referenceFrame, Point3DReadOnly center, double radius)
   {
      set(center, radius);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setMatchingFrame(FramePoint3DReadOnly center, double radius)
   {
      setMatchingFrame(center.getReferenceFrame(), center, radius);
   }

   @Override
   default FixedFrameShape3DPoseBasics getPose()
   {
      return null;
   }

   @Override
   FixedFrameSphere3DBasics copy();
}
