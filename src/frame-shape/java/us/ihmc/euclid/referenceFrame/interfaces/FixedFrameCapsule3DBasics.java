package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface FixedFrameCapsule3DBasics extends Capsule3DBasics, FrameCapsule3DReadOnly, FixedFrameShape3DBasics
{
   @Override
   FixedFramePoint3DBasics getPosition();

   @Override
   FixedFrameVector3DBasics getAxis();

   default void setAxis(ReferenceFrame referenceFrame, Vector3DReadOnly axis)
   {
      checkReferenceFrameMatch(referenceFrame);
      Capsule3DBasics.super.setAxis(axis);
   }

   default void setAxis(FrameVector3DReadOnly axis)
   {
      setAxis(axis.getReferenceFrame(), axis);
   }

   default void set(ReferenceFrame referenceFrame, Capsule3DReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      Capsule3DBasics.super.set(other);
   }

   default void set(FrameCapsule3DReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   default void set(ReferenceFrame referenceFrame, Point3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      checkReferenceFrameMatch(referenceFrame);
      Capsule3DBasics.super.set(position, axis, length, radius);
   }

   default void set(FramePoint3DReadOnly position, FrameVector3DReadOnly axis, double length, double radius)
   {
      position.checkReferenceFrameMatch(axis);
      set(position.getReferenceFrame(), position, axis, length, radius);
   }

   default void setMatchingFrame(ReferenceFrame referenceFrame, Capsule3DReadOnly other)
   {
      Capsule3DBasics.super.set(other);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setMatchingFrame(FrameCapsule3DReadOnly other)
   {
      setMatchingFrame(other.getReferenceFrame(), other);
   }

   default void setMatchingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      Capsule3DBasics.super.set(position, axis, length, radius);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setMatchingFrame(FramePoint3DReadOnly position, FrameVector3DReadOnly axis, double length, double radius)
   {
      position.checkReferenceFrameMatch(axis);
      setMatchingFrame(position.getReferenceFrame(), position, axis, length, radius);
   }
}
