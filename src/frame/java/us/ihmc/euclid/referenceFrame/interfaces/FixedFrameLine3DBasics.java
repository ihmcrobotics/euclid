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

/**
 * Write and read interface for a line 3D expressed in a constant reference frame, i.e. the
 * reference frame of this object cannot be changed via this interface.
 * <p>
 * A line 3D represents an infinitely long line in the XY-plane and defined by a point and a
 * direction.
 * </p>
 * <p>
 * In addition to representing a {@link Line3DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FixedFrameLine3DBasics}. This allows, for instance, to enforce, at runtime, that
 * operations on lines occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FixedFrameLine3DBasics} extends {@code Line3DBasics}, it is compatible with
 * methods only requiring {@code Line3DBasics}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FixedFrameLine3DBasics}.
 * </p>
 */
public interface FixedFrameLine3DBasics extends FrameLine3DReadOnly, Line3DBasics
{
   /** {@inheritDoc} */
   @Override
   FixedFramePoint3DBasics getPoint();

   /** {@inheritDoc} */
   @Override
   FixedFrameUnitVector3DBasics getDirection();

   /**
    * Changes the point through which this line has to go.
    *
    * @param pointOnLine new point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointOnLine} are not expressed
    *                                         in the same reference frame.
    * @deprecated Use {@code this.getPoint().set(pointOnLine)} instead.
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
    *                                         expressed in the same reference frame.
    * @deprecated Use {@code this.getDirection().set(lineDirection)} instead.
    */
   default void setDirection(FrameVector3DReadOnly lineDirection)
   {
      checkReferenceFrameMatch(lineDirection);
      Line3DBasics.super.setDirection(lineDirection);
   }

   /**
    * Sets this line to be the same as the given line.
    * <p>
    * After calling this method this line is in the XY-plane.
    * </p>
    *
    * @param referenceFrame the reference frame in which the given line is expressed.
    * @param line2DReadOnly the line to copy. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame()} and
    *                                         {@code referenceFrame} are not the same reference frame.
    */
   default void set(ReferenceFrame referenceFrame, Line2DReadOnly line2DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(line2DReadOnly);
   }

   /**
    * Sets this line to be the same as the given line.
    *
    * @param referenceFrame the reference frame in which the given line is expressed.
    * @param line3DReadOnly the line to copy. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame()} and
    *                                         {@code referenceFrame} are not the same reference frame.
    */
   default void set(ReferenceFrame referenceFrame, Line3DReadOnly line3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(line3DReadOnly);
   }

   /**
    * Sets this line to go through the endpoints of the given line segment.
    * <p>
    * After calling this method this line is in the XY-plane.
    * </p>
    *
    * @param referenceFrame        the reference frame in which the given line segment is expressed.
    * @param lineSegment2DReadOnly the line segment to copy. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame()} and
    *                                         {@code referenceFrame} are not the same reference frame.
    */
   default void set(ReferenceFrame referenceFrame, LineSegment2DReadOnly lineSegment2DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(lineSegment2DReadOnly);
   }

   /**
    * Sets this line to go through the endpoints of the given line segment.
    *
    * @param referenceFrame        the reference frame in which the given line segment is expressed.
    * @param lineSegment3DReadOnly the line segment to copy. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame()} and
    *                                         {@code referenceFrame} are not the same reference frame.
    */
   default void set(ReferenceFrame referenceFrame, LineSegment3DReadOnly lineSegment3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(lineSegment3DReadOnly);
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    * <p>
    * After calling this method this line is in the XY-plane.
    * </p>
    *
    * @param referenceFrame the reference frame in which the given arguments are expressed.
    * @param pointOnLine    new point on this line. Not modified.
    * @param lineDirection  new direction of this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame()} and
    *                                         {@code referenceFrame} are not the same reference frame.
    */
   default void set(ReferenceFrame referenceFrame, Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pointOnLine, lineDirection);
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    *
    * @param referenceFrame the reference frame in which the given arguments are expressed.
    * @param pointOnLine    new point on this line. Not modified.
    * @param lineDirection  new direction of this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame()} and
    *                                         {@code referenceFrame} are not the same reference frame.
    */
   default void set(ReferenceFrame referenceFrame, Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pointOnLine, lineDirection);
   }

   /**
    * Redefines this line such that it goes through the two given points.
    * <p>
    * After calling this method this line is in the XY-plane.
    * </p>
    *
    * @param referenceFrame    the reference frame in which the given arguments are expressed.
    * @param firstPointOnLine  first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame()} and
    *                                         {@code referenceFrame} are not the same reference frame.
    */
   default void set(ReferenceFrame referenceFrame, Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(firstPointOnLine, secondPointOnLine);
   }

   /**
    * Redefines this line such that it goes through the two given points.
    *
    * @param referenceFrame    the reference frame in which the given arguments are expressed.
    * @param firstPointOnLine  first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame()} and
    *                                         {@code referenceFrame} are not the same reference frame.
    */
   default void set(ReferenceFrame referenceFrame, Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(firstPointOnLine, secondPointOnLine);
   }

   /**
    * Sets this line to be the same as the given line.
    * <p>
    * After calling this method this line is in the XY-plane.
    * </p>
    *
    * @param referenceFrame      the reference frame in which the given line is expressed.
    * @param frameLine2DReadOnly the line to copy. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code frameLine2DReadOnly} are not
    *                                         expressed in the same reference frame.
    */
   default void set(FrameLine2DReadOnly frameLine2DReadOnly)
   {
      set(frameLine2DReadOnly.getReferenceFrame(), frameLine2DReadOnly);
   }

   /**
    * Sets this line to be the same as the given line.
    *
    * @param referenceFrame the reference frame in which the given line is expressed.
    * @param other          the line to copy. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *                                         the same reference frame.
    */
   default void set(FrameLine3DReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   /**
    * Sets this line to be the same as the given line expressed in the reference frame of this.
    * <p>
    * If {@code other} is expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #set(FrameLine3DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param other the other frame line to set this to. Not modified.
    */
   default void setMatchingFrame(FrameLine3DReadOnly other)
   {
      Line3DBasics.super.set(other);
      other.getReferenceFrame().transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this line from the given {@code frameLineSegment2DReadOnly}.
    * <p>
    * After calling this method this line is in the XY-plane.
    * </p>
    *
    * @param frameLineSegment2DReadOnly the line segment to copy. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code frameLineSegment2DReadOnly}
    *                                         are not expressed in the same reference frame.
    */
   default void set(FrameLineSegment2DReadOnly frameLineSegment2DReadOnly)
   {
      set(frameLineSegment2DReadOnly.getReferenceFrame(), frameLineSegment2DReadOnly);
   }

   /**
    * Sets this line from the given {@code frameLineSegment3DReadOnly}.
    *
    * @param frameLineSegment3DReadOnly the line segment to copy. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code frameLineSegment3DReadOnly}
    *                                         are not expressed in the same reference frame.
    */
   default void set(FrameLineSegment3DReadOnly frameLineSegment3DReadOnly)
   {
      set(frameLineSegment3DReadOnly.getReferenceFrame(), frameLineSegment3DReadOnly);
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    * <p>
    * After calling this method this line is in the XY-plane.
    * </p>
    *
    * @param pointOnLine   new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code lineDirection} are not
    *                                         expressed in the same reference frame.
    */
   default void set(Point2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection)
   {
      set(lineDirection.getReferenceFrame(), pointOnLine, lineDirection);
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    * <p>
    * After calling this method this line is in the XY-plane.
    * </p>
    *
    * @param pointOnLine   new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointOnLine} are not expressed
    *                                         in the same reference frame.
    */
   default void set(FramePoint2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      set(pointOnLine.getReferenceFrame(), pointOnLine, lineDirection);
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    * <p>
    * After calling this method this line is in the XY-plane.
    * </p>
    *
    * @param pointOnLine   new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code pointOnLine},
    *                                         {@code lineDirection} are not expressed in the same
    *                                         reference frame.
    */
   default void set(FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection);
      set(pointOnLine.getReferenceFrame(), pointOnLine, lineDirection);
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    *
    * @param pointOnLine   new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code lineDirection} are not
    *                                         expressed in the same reference frame.
    */
   default void set(Point3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection)
   {
      set(lineDirection.getReferenceFrame(), pointOnLine, lineDirection);
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    *
    * @param pointOnLine   new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointOnLine} are not expressed
    *                                         in the same reference frame.
    */
   default void set(FramePoint3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      set(pointOnLine.getReferenceFrame(), pointOnLine, lineDirection);
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    *
    * @param pointOnLine   new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code pointOnLine},
    *                                         {@code lineDirection} are not expressed in the same
    *                                         reference frame.
    */
   default void set(FramePoint3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection);
      set(pointOnLine.getReferenceFrame(), pointOnLine, lineDirection);
   }

   /**
    * Redefines this line such that it goes through the two given points.
    * <p>
    * After calling this method this line is in the XY-plane.
    * </p>
    *
    * @param firstPointOnLine  first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondPointOnLine} are not
    *                                         expressed in the same reference frame.
    */
   default void set(Point2DReadOnly firstPointOnLine, FramePoint2DReadOnly secondPointOnLine)
   {
      set(secondPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }

   /**
    * Redefines this line such that it goes through the two given points.
    * <p>
    * After calling this method this line is in the XY-plane.
    * </p>
    *
    * @param firstPointOnLine  first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code firstPointOnLine} are not
    *                                         expressed in the same reference frame.
    */
   default void set(FramePoint2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      set(firstPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }

   /**
    * Redefines this line such that it goes through the two given points.
    * <p>
    * After calling this method this line is in the XY-plane.
    * </p>
    *
    * @param firstPointOnLine  first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code firstPointOnLine},
    *                                         {@code secondPointOnLine} are not expressed in the same
    *                                         reference frame.
    */
   default void set(FramePoint2DReadOnly firstPointOnLine, FramePoint2DReadOnly secondPointOnLine)
   {
      firstPointOnLine.checkReferenceFrameMatch(secondPointOnLine);
      set(firstPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }

   /**
    * Redefines this line such that it goes through the two given points.
    *
    * @param firstPointOnLine  first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondPointOnLine} are not
    *                                         expressed in the same reference frame.
    */
   default void set(Point3DReadOnly firstPointOnLine, FramePoint3DReadOnly secondPointOnLine)
   {
      set(secondPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }

   /**
    * Redefines this line such that it goes through the two given points.
    *
    * @param firstPointOnLine  first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code firstPointOnLine} are not
    *                                         expressed in the same reference frame.
    */
   default void set(FramePoint3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      set(firstPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }

   /**
    * Redefines this line such that it goes through the two given points.
    *
    * @param firstPointOnLine  first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondPointOnLine} are not
    *                                         expressed in the same reference frame.
    */
   default void set(FramePoint3DReadOnly firstPointOnLine, FramePoint3DReadOnly secondPointOnLine)
   {
      firstPointOnLine.checkReferenceFrameMatch(secondPointOnLine);
      set(firstPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }
}
