package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Line3DBasics;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface FixedFrameLine3DBasics extends FrameLine3DReadOnly, Line3DBasics
{
   /** {@inheritDoc} */
   @Override
   FixedFramePoint3DBasics getPoint();

   /** {@inheritDoc} */
   @Override
   FixedFrameVector3DBasics getDirection();

   /**
    * Changes the point through which this line has to go.
    *
    * @param pointOnLine new point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointOnLine} are not
    *            expressed in the same reference frame.
    */
   default void setPoint(FramePoint3DReadOnly pointOnLine)
   {
      checkReferenceFrameMatch(pointOnLine);
      Line3DBasics.super.setPoint(pointOnLine);
   }

   /**
    * Changes the direction of this line by setting it to the normalized value of the given vector.
    *
    * @param lineDirection new direction of this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code lineDirection} are not
    *            expressed in the same reference frame.
    */
   default void setDirection(FrameVector3DReadOnly lineDirection)
   {
      checkReferenceFrameMatch(lineDirection);
      Line3DBasics.super.setDirection(lineDirection);
   }

   default void set(ReferenceFrame referenceFrame, Line2DReadOnly line2DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(line2DReadOnly);
   }

   default void set(ReferenceFrame referenceFrame, Line3DReadOnly line3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(line3DReadOnly);
   }

   default void set(ReferenceFrame referenceFrame, LineSegment2DReadOnly lineSegment2DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(lineSegment2DReadOnly);
   }

   default void set(ReferenceFrame referenceFrame, LineSegment3DReadOnly lineSegment3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(lineSegment3DReadOnly);
   }

   default void set(ReferenceFrame referenceFrame, Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pointOnLine, lineDirection);
   }

   default void set(ReferenceFrame referenceFrame, Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pointOnLine, lineDirection);
   }

   default void set(ReferenceFrame referenceFrame, Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(firstPointOnLine, secondPointOnLine);
   }

   default void set(ReferenceFrame referenceFrame, Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(firstPointOnLine, secondPointOnLine);
   }

   default void set(FrameLine2DReadOnly frameLine2DReadOnly)
   {
      set(frameLine2DReadOnly.getReferenceFrame(), frameLine2DReadOnly);
   }

   default void set(FrameLine3DReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   default void set(FrameLineSegment2DReadOnly frameLineSegment2DReadOnly)
   {
      set(frameLineSegment2DReadOnly.getReferenceFrame(), frameLineSegment2DReadOnly);
   }

   default void set(FrameLineSegment3DReadOnly frameLineSegment3DReadOnly)
   {
      set(frameLineSegment3DReadOnly.getReferenceFrame(), frameLineSegment3DReadOnly);
   }

   default void set(Point2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection)
   {
      set(lineDirection.getReferenceFrame(), pointOnLine, lineDirection);
   }

   default void set(FramePoint2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      set(pointOnLine.getReferenceFrame(), pointOnLine, lineDirection);
   }

   default void set(FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection);
      set(pointOnLine.getReferenceFrame(), pointOnLine, lineDirection);
   }

   default void set(Point3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection)
   {
      set(lineDirection.getReferenceFrame(), pointOnLine, lineDirection);
   }

   default void set(FramePoint3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      set(pointOnLine.getReferenceFrame(), pointOnLine, lineDirection);
   }

   default void set(FramePoint3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection);
      set(pointOnLine.getReferenceFrame(), pointOnLine, lineDirection);
   }

   default void set(Point2DReadOnly firstPointOnLine, FramePoint2DReadOnly secondPointOnLine)
   {
      set(secondPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }

   default void set(FramePoint2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      set(firstPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }

   default void set(FramePoint2DReadOnly firstPointOnLine, FramePoint2DReadOnly secondPointOnLine)
   {
      firstPointOnLine.checkReferenceFrameMatch(secondPointOnLine);
      set(firstPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }

   default void set(Point3DReadOnly firstPointOnLine, FramePoint3DReadOnly secondPointOnLine)
   {
      set(secondPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }

   default void set(FramePoint3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      set(firstPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }

   default void set(FramePoint3DReadOnly firstPointOnLine, FramePoint3DReadOnly secondPointOnLine)
   {
      firstPointOnLine.checkReferenceFrameMatch(secondPointOnLine);
      set(firstPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }
}
