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
 * Write and read interface for a line 2D expressed in a constant reference frame, i.e. this line is
 * always expressed in the same reference frame.
 * <p>
 * A line 2D represents an infinitely long line in the XY-plane and defined by a point and a
 * direction.
 * </p>
 * <p>
 * In addition to representing a {@link Line2DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FixedFrameLine2DBasics}. This allows, for instance, to enforce, at runtime, that
 * operations on lines occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FixedFrameLine2DBasics} extends {@code Line2DBasics}, it is compatible with
 * methods only requiring {@code Line2DBasics}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FixedFrameLine2DBasics}.
 * </p>
 */
public interface FixedFrameLine2DBasics extends FrameLine2DReadOnly, Line2DBasics
{
   /**
    * Gets the reference to the point through which this line is going.
    *
    * @return the reference to the point.
    */
   @Override
   FixedFramePoint2DBasics getPoint();

   /**
    * Gets the reference to the direction of this line.
    *
    * @return the reference to the direction.
    */
   @Override
   FixedFrameVector2DBasics getDirection();

   /**
    * Changes the point through which this line has to go.
    *
    * @param pointOnLine new point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointOnLine} are not expressed
    *            in the same reference frame.
    */
   default void setPoint(FramePoint2DReadOnly pointOnLine)
   {
      checkReferenceFrameMatch(pointOnLine);
      Line2DBasics.super.setPoint(pointOnLine);
   }

   /**
    * Changes the point through which this line has to go.
    *
    * @param pointOnLine new point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointOnLine} are not expressed
    *            in the same reference frame.
    */
   default void setPoint(FramePoint3DReadOnly pointOnLine)
   {
      checkReferenceFrameMatch(pointOnLine);
      Line2DBasics.super.setPoint(pointOnLine);
   }

   /**
    * Changes the direction of this line by setting it to the normalized value of the given vector.
    *
    * @param lineDirection new direction of this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code lineDirection} are not
    *            expressed in the same reference frame.
    */
   default void setDirection(FrameVector2DReadOnly lineDirection)
   {
      checkReferenceFrameMatch(lineDirection);
      Line2DBasics.super.setDirection(lineDirection);
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
      Line2DBasics.super.setDirection(lineDirection);
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    *
    * @param pointOnLineX the new x-coordinate of the point on this line.
    * @param pointOnLineY the new y-coordinate of the point on this line.
    * @param lineDirectionX the new x-component of the direction of this line.
    * @param lineDirectionY the new y-component of the direction of this line.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame()} and
    *            {@code referenceFrame} are not the same reference frame.
    */
   default void set(ReferenceFrame referenceFrame, double pointOnLineX, double pointOnLineY, double lineDirectionX, double lineDirectionY)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY);
   }

   /**
    * Sets this line to be the same as the given line.
    *
    * @param referenceFrame the reference frame in which the given line is expressed.
    * @param line2DReadOnly the line to copy. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame()} and
    *            {@code referenceFrame} are not the same reference frame.
    */
   default void set(ReferenceFrame referenceFrame, Line2DReadOnly line2DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(line2DReadOnly);
   }

   /**
    * Sets this line to be the same as the given line projected onto the XY-plane.
    *
    * @param referenceFrame the reference frame in which the given line is expressed.
    * @param line3DReadOnly the line to copy. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame()} and
    *            {@code referenceFrame} are not the same reference frame.
    */
   default void set(ReferenceFrame referenceFrame, Line3DReadOnly line3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(line3DReadOnly);
   }

   /**
    * Sets this line to go through the endpoints of the given line segment.
    *
    * @param referenceFrame the reference frame in which the given line segment is expressed.
    * @param lineSegment2DReadOnly the line segment to copy. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame()} and
    *            {@code referenceFrame} are not the same reference frame.
    */
   default void set(ReferenceFrame referenceFrame, LineSegment2DReadOnly lineSegment2DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(lineSegment2DReadOnly);
   }

   /**
    * Sets this line to go through the endpoints of the given line segment projected on the XY-plane.
    *
    * @param referenceFrame the reference frame in which the given line segment is expressed.
    * @param lineSegment3DReadOnly the line segment to copy. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame()} and
    *            {@code referenceFrame} are not the same reference frame.
    */
   default void set(ReferenceFrame referenceFrame, LineSegment3DReadOnly lineSegment3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(lineSegment3DReadOnly);
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    *
    * @param referenceFrame the reference frame in which the given arguments are expressed.
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame()} and
    *            {@code referenceFrame} are not the same reference frame.
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
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame()} and
    *            {@code referenceFrame} are not the same reference frame.
    */
   default void set(ReferenceFrame referenceFrame, Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pointOnLine, lineDirection);
   }

   /**
    * Redefines this line such that it goes through the two given points.
    *
    * @param referenceFrame the reference frame in which the given arguments are expressed.
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame()} and
    *            {@code referenceFrame} are not the same reference frame.
    */
   default void set(ReferenceFrame referenceFrame, Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(firstPointOnLine, secondPointOnLine);
   }

   /**
    * Redefines this line such that it goes through the two given points.
    *
    * @param referenceFrame the reference frame in which the given arguments are expressed.
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame()} and
    *            {@code referenceFrame} are not the same reference frame.
    */
   default void set(ReferenceFrame referenceFrame, Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(firstPointOnLine, secondPointOnLine);
   }

   /**
    * Sets this line to be the same as the given line.
    *
    * @param referenceFrame the reference frame in which the given line is expressed.
    * @param other the line to copy. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
    */
   default void set(FrameLine2DReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   /**
    * Sets this line to be the same as the given line projected onto the XY-plane.
    *
    * @param frameLine3DReadOnly the line to copy. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code frameLine3DReadOnly} are not
    *            expressed in the same reference frame.
    */
   default void set(FrameLine3DReadOnly frameLine3DReadOnly)
   {
      set(frameLine3DReadOnly.getReferenceFrame(), frameLine3DReadOnly);
   }

   /**
    * Sets this line from the given {@code frameLineSegment2DReadOnly}.
    *
    * @param frameLineSegment2DReadOnly the line segment to copy. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code frameLineSegment2DReadOnly}
    *            are not expressed in the same reference frame.
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
    *            are not expressed in the same reference frame.
    */
   default void set(FrameLineSegment3DReadOnly frameLineSegment3DReadOnly)
   {
      set(frameLineSegment3DReadOnly.getReferenceFrame(), frameLineSegment3DReadOnly);
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    *
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code lineDirection} are not
    *            expressed in the same reference frame.
    */
   default void set(Point2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection)
   {
      set(lineDirection.getReferenceFrame(), pointOnLine, lineDirection);
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    *
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointOnLine} are not expressed
    *            in the same reference frame.
    */
   default void set(FramePoint2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      set(pointOnLine.getReferenceFrame(), pointOnLine, lineDirection);
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    *
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code pointOnLine},
    *            {@code lineDirection} are not expressed in the same reference frame.
    */
   default void set(FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection);
      set(pointOnLine.getReferenceFrame(), pointOnLine, lineDirection);
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    *
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code lineDirection} are not
    *            expressed in the same reference frame.
    */
   default void set(Point3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection)
   {
      set(lineDirection.getReferenceFrame(), pointOnLine, lineDirection);
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    *
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointOnLine} are not expressed
    *            in the same reference frame.
    */
   default void set(FramePoint3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      set(pointOnLine.getReferenceFrame(), pointOnLine, lineDirection);
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    *
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code pointOnLine},
    *            {@code lineDirection} are not expressed in the same reference frame.
    */
   default void set(FramePoint3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection);
      set(pointOnLine.getReferenceFrame(), pointOnLine, lineDirection);
   }

   /**
    * Redefines this line such that it goes through the two given points.
    *
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondPointOnLine} are not
    *            expressed in the same reference frame.
    */
   default void set(Point2DReadOnly firstPointOnLine, FramePoint2DReadOnly secondPointOnLine)
   {
      set(secondPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }

   /**
    * Redefines this line such that it goes through the two given points.
    *
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code firstPointOnLine} are not
    *            expressed in the same reference frame.
    */
   default void set(FramePoint2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      set(firstPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }

   /**
    * Redefines this line such that it goes through the two given points.
    *
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code firstPointOnLine},
    *            {@code secondPointOnLine} are not expressed in the same reference frame.
    */
   default void set(FramePoint2DReadOnly firstPointOnLine, FramePoint2DReadOnly secondPointOnLine)
   {
      firstPointOnLine.checkReferenceFrameMatch(secondPointOnLine);
      set(firstPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }

   /**
    * Redefines this line such that it goes through the two given points.
    *
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondPointOnLine} are not
    *            expressed in the same reference frame.
    */
   default void set(Point3DReadOnly firstPointOnLine, FramePoint3DReadOnly secondPointOnLine)
   {
      set(secondPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }

   /**
    * Redefines this line such that it goes through the two given points.
    *
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code firstPointOnLine} are not
    *            expressed in the same reference frame.
    */
   default void set(FramePoint3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      set(firstPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }

   /**
    * Redefines this line such that it goes through the two given points.
    *
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondPointOnLine} are not
    *            expressed in the same reference frame.
    */
   default void set(FramePoint3DReadOnly firstPointOnLine, FramePoint3DReadOnly secondPointOnLine)
   {
      firstPointOnLine.checkReferenceFrameMatch(secondPointOnLine);
      set(firstPointOnLine.getReferenceFrame(), firstPointOnLine, secondPointOnLine);
   }
}
