package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Line2DBasics;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Write and read interface for a line 2D expressed in a changeable reference frame, i.e. the
 * reference frame in which this line is expressed can be changed.
 * <p>
 * A line 2D represents an infinitely long line in the XY-plane and defined by a point and a
 * direction.
 * </p>
 * <p>
 * In addition to representing a {@link Line2DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FrameLine2DBasics}. This allows, for instance, to enforce, at runtime, that operations on
 * lines occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameLine2DBasics} extends {@code Line2DBasics}, it is compatible with methods
 * only requiring {@code Line2DBasics}. However, these methods do NOT assert that the operation
 * occur in the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FrameLine2DBasics}.
 * </p>
 */
public interface FrameLine2DBasics extends FixedFrameLine2DBasics, FrameChangeable
{
   /**
    * Sets the reference frame of this line 2D without updating or modifying its point or direction.
    *
    * @param referenceFrame the new reference frame for this frame line 2D.
    */
   @Override
   void setReferenceFrame(ReferenceFrame referenceFrame);

   /**
    * Sets the point and direction parts of this line 2D to zero and sets the current reference
    * frame to {@code referenceFrame}.
    * 
    * @param referenceFrame the new reference frame to be associated with this line 2D.
    */
   default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToZero();
   }

   /**
    * Sets the point and direction parts of this line 2D to {@link Double#NaN} and sets the current
    * reference frame to {@code referenceFrame}.
    * 
    * @param referenceFrame the new reference frame to be associated with this line 2D.
    */
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToNaN();
   }

   /**
    * Redefines this line with a new point, a new direction vector, and a new reference frame.
    *
    * @param referenceFrame the new reference frame for this frame line.
    * @param pointOnLineX the new x-coordinate of the point on this line.
    * @param pointOnLineY the new y-coordinate of the point on this line.
    * @param lineDirectionX the new x-component of the direction of this line.
    * @param lineDirectionY the new y-component of the direction of this line.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, double pointOnLineX, double pointOnLineY, double lineDirectionX, double lineDirectionY)
   {
      setReferenceFrame(referenceFrame);
      set(pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY);
   }

   /**
    * Sets this line to be the same as the given line.
    *
    * @param referenceFrame the new reference frame for this frame line.
    * @param other the other line to copy. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Line2DReadOnly line2DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(line2DReadOnly);
   }

   /**
    * Sets this line to be the same as the given line.
    *
    * @param referenceFrame the new reference frame for this frame line.
    * @param other the other line to copy. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Line3DReadOnly line3DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(line3DReadOnly);
   }

   /**
    * Sets this line to go through the endpoints of the given line segment.
    *
    * @param referenceFrame the new reference frame for this frame line.
    * @param lineSegment2DReadOnly the line segment to copy. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, LineSegment2DReadOnly lineSegment2DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(lineSegment2DReadOnly);
   }

   /**
    * Sets this line to go through the endpoints of the given line segment projected on the XY-plane.
    *
    * @param referenceFrame the new reference frame for this frame line.
    * @param lineSegment3DReadOnly the line segment to copy. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, LineSegment3DReadOnly lineSegment3DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(lineSegment3DReadOnly);
   }

   /**
    * Redefines this line with a new point, a new direction vector, and a new reference frame.
    *
    * @param referenceFrame the new reference frame for this frame line.
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      setReferenceFrame(referenceFrame);
      set(pointOnLine, lineDirection);
   }

   /**
    * Redefines this line with a new point, a new direction vector, and a new reference frame.
    *
    * @param referenceFrame the new reference frame for this frame line.
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      setReferenceFrame(referenceFrame);
      set(pointOnLine, lineDirection);
   }

   /**
    * Redefines this line such that it goes through the two given points in the given reference
    * frame.
    *
    * @param referenceFrame the new reference frame for this frame line.
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      setReferenceFrame(referenceFrame);
      set(firstPointOnLine, secondPointOnLine);
   }

   /**
    * Redefines this line such that it goes through the two given points in the given reference
    * frame.
    *
    * @param referenceFrame the new reference frame for this frame line.
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      setReferenceFrame(referenceFrame);
      set(firstPointOnLine, secondPointOnLine);
   }

   /**
    * Sets this line to be the same as the given line.
    *
    * @param other the other line to copy. Not modified.
    */
   default void setIncludingFrame(FrameLine2DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   /**
    * Sets this line to be the same as the given line.
    *
    * @param other the other line to copy. Not modified.
    */
   default void setIncludingFrame(FrameLine3DReadOnly frameLine3DReadOnly)
   {
      setIncludingFrame(frameLine3DReadOnly.getReferenceFrame(), frameLine3DReadOnly);
   }

   /**
    * Sets this line to go through the endpoints of the given line segment.
    *
    * @param referenceFrame the new reference frame for this frame line.
    * @param frameLineSegment2DReadOnly the line segment to copy. Not modified.
    */
   default void setIncludingFrame(FrameLineSegment2DReadOnly frameLineSegment2DReadOnly)
   {
      setIncludingFrame(frameLineSegment2DReadOnly.getReferenceFrame(), frameLineSegment2DReadOnly);
   }

   /**
    * Sets this line to go through the endpoints of the given line segment projected on the XY-plane.
    *
    * @param referenceFrame the new reference frame for this frame line.
    * @param frameLineSegment3DReadOnly the line segment to copy. Not modified.
    */
   default void setIncludingFrame(FrameLineSegment3DReadOnly frameLineSegment3DReadOnly)
   {
      setIncludingFrame(frameLineSegment3DReadOnly.getReferenceFrame(), frameLineSegment3DReadOnly);
   }

   /**
    * Redefines this line with a new point, a new direction vector, and a new reference frame.
    *
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code pointOnLine} and {@code lineDirection} are
    *            not expressed in the same reference frame
    */
   default void setIncludingFrame(FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection);
      setIncludingFrame(pointOnLine.getReferenceFrame(), pointOnLine, lineDirection);
   }

   /**
    * Redefines this line with a new point, a new direction vector, and a new reference frame.
    *
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    */
   default void setIncludingFrame(FramePoint2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      setIncludingFrame(pointOnLine.getReferenceFrame(), pointOnLine, lineDirection);
   }

   /**
    * Redefines this line with a new point, a new direction vector, and a new reference frame.
    *
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    */
   default void setIncludingFrame(Point2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection)
   {
      setIncludingFrame(lineDirection.getReferenceFrame(), pointOnLine, lineDirection);
   }

   /**
    * Redefines this line with a new point, a new direction vector, and a new reference frame.
    *
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code pointOnLine} and {@code lineDirection} are
    *            not expressed in the same reference frame
    */
   default void setIncludingFrame(FramePoint3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection);
      setIncludingFrame(pointOnLine.getReferenceFrame(), pointOnLine, lineDirection);
   }

   /**
    * Redefines this line with a new point, a new direction vector, and a new reference frame.
    *
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    */
   default void setIncludingFrame(FramePoint3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      setIncludingFrame(pointOnLine.getReferenceFrame(), pointOnLine, lineDirection);
   }

   /**
    * Redefines this line with a new point, a new direction vector, and a new reference frame.
    *
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    */
   default void setIncludingFrame(Point3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection)
   {
      setIncludingFrame(lineDirection.getReferenceFrame(), pointOnLine, lineDirection);
   }

   /**
    * Redefines this line such that it goes through the two given points in the given reference
    * frame.
    *
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code firstPointOnLine} and
    *            {@code secondPointOnLine} are not expressed in the same reference frame
    */
   default void setIncludingFrame(FramePoint2DReadOnly firstPointOnLine, FramePoint2DReadOnly secondPointOnLine)
   {
      firstPointOnLine.checkReferenceFrameMatch(secondPointOnLine);
      setIncludingFrame(firstPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }

   /**
    * Redefines this line such that it goes through the two given points in the given reference
    * frame.
    *
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    */
   default void setIncludingFrame(FramePoint2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      setIncludingFrame(firstPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }

   /**
    * Redefines this line such that it goes through the two given points in the given reference
    * frame.
    *
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    */
   default void setIncludingFrame(Point2DReadOnly firstPointOnLine, FramePoint2DReadOnly secondPointOnLine)
   {
      setIncludingFrame(secondPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }

   /**
    * Redefines this line such that it goes through the two given points in the given reference
    * frame.
    *
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code firstPointOnLine} and
    *            {@code secondPointOnLine} are not expressed in the same reference frame
    */
   default void setIncludingFrame(FramePoint3DReadOnly firstPointOnLine, FramePoint3DReadOnly secondPointOnLine)
   {
      firstPointOnLine.checkReferenceFrameMatch(secondPointOnLine);
      setIncludingFrame(firstPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }

   /**
    * Redefines this line such that it goes through the two given points in the given reference
    * frame.
    *
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    */
   default void setIncludingFrame(FramePoint3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      setIncludingFrame(firstPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }

   /**
    * Redefines this line such that it goes through the two given points in the given reference
    * frame.
    *
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    */
   default void setIncludingFrame(Point3DReadOnly firstPointOnLine, FramePoint3DReadOnly secondPointOnLine)
   {
      setIncludingFrame(secondPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }

   /**
    * Performs a transformation of the line such that it is expressed in a new frame
    * {@code desireFrame}.
    * <p>
    * Because the transformation between two reference frames is a 3D transformation, the result of
    * transforming this line 2D can result in a line 3D. This method projects the result of the
    * transformation onto the XY-plane.
    * </p>
    *
    * @param desiredFrame the reference frame in which the line is to be expressed.
    */
   void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame);
}
