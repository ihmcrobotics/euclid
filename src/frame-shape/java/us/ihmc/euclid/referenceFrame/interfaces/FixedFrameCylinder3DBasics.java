package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface FixedFrameCylinder3DBasics extends Cylinder3DBasics, FrameCylinder3DReadOnly, FixedFrameShape3DBasics
{
   @Override
   FixedFramePoint3DBasics getPosition();

   @Override
   FixedFrameVector3DBasics getAxis();

   default void setAxis(ReferenceFrame referenceFrame, Vector3DReadOnly axis)
   {
      checkReferenceFrameMatch(referenceFrame);
      Cylinder3DBasics.super.setAxis(axis);
   }

   default void setAxis(FrameVector3DReadOnly axis)
   {
      setAxis(axis.getReferenceFrame(), axis);
   }

   default void set(ReferenceFrame referenceFrame, Cylinder3DReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      Cylinder3DBasics.super.set(other);
   }

   default void set(FrameCylinder3DReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   default void set(ReferenceFrame referenceFrame, Point3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      checkReferenceFrameMatch(referenceFrame);
      Cylinder3DBasics.super.set(position, axis, length, radius);
   }

   default void set(FramePoint3DReadOnly position, FrameVector3DReadOnly axis, double length, double radius)
   {
      position.checkReferenceFrameMatch(axis);
      set(position.getReferenceFrame(), position, axis, length, radius);
   }

   default void setMatchingFrame(ReferenceFrame referenceFrame, Cylinder3DReadOnly other)
   {
      Cylinder3DBasics.super.set(other);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setMatchingFrame(FrameCylinder3DReadOnly other)
   {
      setMatchingFrame(other.getReferenceFrame(), other);
   }

   default void setMatchingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      Cylinder3DBasics.super.set(position, axis, length, radius);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setMatchingFrame(FramePoint3DReadOnly position, FrameVector3DReadOnly axis, double length, double radius)
   {
      position.checkReferenceFrameMatch(axis);
      setMatchingFrame(position.getReferenceFrame(), position, axis, length, radius);
   }
}
