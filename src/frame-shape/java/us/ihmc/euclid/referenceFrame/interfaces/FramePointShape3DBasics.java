package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public interface FramePointShape3DBasics extends FixedFramePointShape3DBasics, FrameShape3DBasics, FramePoint3DBasics
{
   @Override
   default void setToZero(ReferenceFrame referenceFrame)
   {
      FrameShape3DBasics.super.setToZero(referenceFrame);
   }

   @Override
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      FrameShape3DBasics.super.setToNaN(referenceFrame);
   }
}
