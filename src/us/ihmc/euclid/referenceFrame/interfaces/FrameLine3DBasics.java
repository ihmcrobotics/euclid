package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface FrameLine3DBasics extends FixedFrameLine3DBasics, FrameChangeable
{
   /**
    * Sets the reference frame of this line 3D without updating or modifying its point or direction.
    *
    * @param referenceFrame the new reference frame for this frame line 3D.
    */
   @Override
   void setReferenceFrame(ReferenceFrame referenceFrame);

   default void setIncludingFrame(ReferenceFrame referenceFrame, Line2DReadOnly line2DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(line2DReadOnly);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Line3DReadOnly line3DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(line3DReadOnly);
   }

   /**
    * Sets the point and direction parts of this line 3D to zero and sets the current reference
    * frame to {@code referenceFrame}.
    * 
    * @param referenceFrame the new reference frame to be associated with this line 3D.
    */
   default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToZero();
   }

   /**
    * Sets the point and direction parts of this line 3D to {@link Double#NaN} and sets the current
    * reference frame to {@code referenceFrame}.
    * 
    * @param referenceFrame the new reference frame to be associated with this line 3D.
    */
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToNaN();
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, LineSegment2DReadOnly lineSegment2DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(lineSegment2DReadOnly);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, LineSegment3DReadOnly lineSegment3DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(lineSegment3DReadOnly);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      setReferenceFrame(referenceFrame);
      set(pointOnLine, lineDirection);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      setReferenceFrame(referenceFrame);
      set(pointOnLine, lineDirection);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      setReferenceFrame(referenceFrame);
      set(firstPointOnLine, secondPointOnLine);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      setReferenceFrame(referenceFrame);
      set(firstPointOnLine, secondPointOnLine);
   }

   default void setIncludingFrame(FrameLine2DReadOnly frameLine2DReadOnly)
   {
      setIncludingFrame(frameLine2DReadOnly.getReferenceFrame(), frameLine2DReadOnly);
   }

   default void setIncludingFrame(FrameLine3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   default void setIncludingFrame(FrameLineSegment2DReadOnly frameLineSegment2DReadOnly)
   {
      setIncludingFrame(frameLineSegment2DReadOnly.getReferenceFrame(), frameLineSegment2DReadOnly);
   }

   default void setIncludingFrame(FrameLineSegment3DReadOnly frameLineSegment3DReadOnly)
   {
      setIncludingFrame(frameLineSegment3DReadOnly.getReferenceFrame(), frameLineSegment3DReadOnly);
   }

   default void setIncludingFrame(FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection);
      setIncludingFrame(pointOnLine.getReferenceFrame(), pointOnLine, lineDirection);
   }

   default void setIncludingFrame(FramePoint3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection);
      setIncludingFrame(pointOnLine.getReferenceFrame(), pointOnLine, lineDirection);
   }

   default void setIncludingFrame(FramePoint2DReadOnly firstPointOnLine, FramePoint2DReadOnly secondPointOnLine)
   {
      firstPointOnLine.checkReferenceFrameMatch(secondPointOnLine);
      setIncludingFrame(firstPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }

   default void setIncludingFrame(FramePoint3DReadOnly firstPointOnLine, FramePoint3DReadOnly secondPointOnLine)
   {
      firstPointOnLine.checkReferenceFrameMatch(secondPointOnLine);
      setIncludingFrame(firstPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }
}
