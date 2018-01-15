package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;

public interface FixedFramePoint2DBasics extends FramePoint2DReadOnly, FixedFrameTuple2DBasics, Point2DBasics
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
