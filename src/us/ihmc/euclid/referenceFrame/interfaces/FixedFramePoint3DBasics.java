package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

public interface FixedFramePoint3DBasics extends FramePoint3DReadOnly, FixedFrameTuple3DBasics, Point3DBasics
{
   /**
    * Sets this point coordinate to the given {@code referenceFrame}'s origin coordinate in this
    * frame tuple current frame.
    *
    * @param referenceFrame the reference frame of interest.
    */
   default void setFromReferenceFrame(ReferenceFrame referenceFrame)
   {
      setToZero();
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }
}
