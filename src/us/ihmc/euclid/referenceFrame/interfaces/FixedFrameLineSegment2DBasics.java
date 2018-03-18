package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.LineSegment2DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Write and read interface for a line segment 2D expressed in a constant reference frame, i.e. this
 * line segment is always expressed in the same reference frame.
 * <p>
 * A line segment 2D is a finite-length line defined in the XY-plane by its two 2D endpoints.
 * </p>
 * <p>
 * In addition to representing a {@link LineSegment2DBasics}, a {@link ReferenceFrame} is associated
 * to a {@code FrameLineSegment2DBasics}. This allows, for instance, to enforce, at runtime, that
 * operations on lines occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameLineSegment2DBasics} extends {@code LineSegment2DBasics}, it is compatible
 * with methods only requiring {@code LineSegment2DBasics}. However, these methods do NOT assert
 * that the operation occur in the proper coordinate system. Use this feature carefully and always
 * prefer using methods requiring {@code FrameLineSegment2DBasics}.
 * </p>
 */
public interface FixedFrameLineSegment2DBasics extends FrameLineSegment2DReadOnly, LineSegment2DBasics
{
   /**
    * Gets the reference to the first endpoint of this line segment.
    *
    * @return the reference to the first endpoint of this line segment.
    */
   @Override
   FixedFramePoint2DBasics getFirstEndpoint();

   /**
    * Gets the reference to the second endpoint of this line segment.
    *
    * @return the reference to the second endpoint of this line segment.
    */
   @Override
   FixedFramePoint2DBasics getSecondEndpoint();

   /**
    * Changes the first endpoint of this line segment.
    *
    * @param firstEndpointX x-coordinate of the new first endpoint.
    * @param firstEndpointY y-coordinate of the new first endpoint.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame() != referenceFrame}.
    */
   default void setFirstEndpoint(ReferenceFrame referenceFrame, double firstEndpointX, double firstEndpointY)
   {
      checkReferenceFrameMatch(referenceFrame);
      LineSegment2DBasics.super.setFirstEndpoint(firstEndpointX, firstEndpointY);
   }

   /**
    * Changes the first endpoint of this line segment.
    *
    * @param firstEndpoint new endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame() != referenceFrame}.
    */
   default void setFirstEndpoint(ReferenceFrame referenceFrame, Point2DReadOnly firstEndpoint)
   {
      checkReferenceFrameMatch(referenceFrame);
      LineSegment2DBasics.super.setFirstEndpoint(firstEndpoint);
   }

   /**
    * Changes the first endpoint of this line segment.
    *
    * @param firstEndpoint new endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame() != referenceFrame}.
    */
   default void setFirstEndpoint(ReferenceFrame referenceFrame, Point3DReadOnly firstEndpoint)
   {
      checkReferenceFrameMatch(referenceFrame);
      LineSegment2DBasics.super.setFirstEndpoint(firstEndpoint);
   }

   /**
    * Changes the second endpoint of this line segment.
    *
    * @param secondEndpointX x-coordinate of the new second endpoint.
    * @param secondEndpointY y-coordinate of the new second endpoint.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame() != referenceFrame}.
    */
   default void setSecondEndpoint(ReferenceFrame referenceFrame, double secondEndpointX, double secondEndpointY)
   {
      checkReferenceFrameMatch(referenceFrame);
      LineSegment2DBasics.super.setSecondEndpoint(secondEndpointX, secondEndpointY);
   }

   /**
    * Changes the second endpoint of this line segment.
    *
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame() != referenceFrame}.
    */
   default void setSecondEndpoint(ReferenceFrame referenceFrame, Point2DReadOnly secondEndpoint)
   {
      checkReferenceFrameMatch(referenceFrame);
      LineSegment2DBasics.super.setSecondEndpoint(secondEndpoint);
   }

   /**
    * Changes the second endpoint of this line segment.
    *
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame() != referenceFrame}.
    */
   default void setSecondEndpoint(ReferenceFrame referenceFrame, Point3DReadOnly secondEndpoint)
   {
      checkReferenceFrameMatch(referenceFrame);
      LineSegment2DBasics.super.setSecondEndpoint(secondEndpoint);
   }

   /**
    * Changes the first endpoint of this line segment.
    *
    * @param firstEndpoint new endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code firstEndpoint} are not
    *            expressed in the same reference frame.
    */
   default void setFirstEndpoint(FramePoint2DReadOnly firstEndpoint)
   {
      setFirstEndpoint(firstEndpoint.getReferenceFrame(), firstEndpoint);
   }

   /**
    * Changes the first endpoint of this line segment.
    *
    * @param firstEndpoint new endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code firstEndpoint} are not
    *            expressed in the same reference frame.
    */
   default void setFirstEndpoint(FramePoint3DReadOnly firstEndpoint)
   {
      setFirstEndpoint(firstEndpoint.getReferenceFrame(), firstEndpoint);
   }

   /**
    * Changes the second endpoint of this line segment.
    *
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondEndpoint} are not
    *            expressed in the same reference frame.
    */
   default void setSecondEndpoint(FramePoint2DReadOnly secondEndpoint)
   {
      setSecondEndpoint(secondEndpoint.getReferenceFrame(), secondEndpoint);
   }

   /**
    * Changes the second endpoint of this line segment.
    *
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondEndpoint} are not
    *            expressed in the same reference frame.
    */
   default void setSecondEndpoint(FramePoint3DReadOnly secondEndpoint)
   {
      setSecondEndpoint(secondEndpoint.getReferenceFrame(), secondEndpoint);
   }

   /**
    * Redefines this line segments with new endpoints.
    *
    * @param firstEndpointX x-coordinate of the new first endpoint.
    * @param firstEndpointY y-coordinate of the new first endpoint.
    * @param secondEndpointX x-coordinate of the new second endpoint.
    * @param secondEndpointY y-coordinate of the new second endpoint.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame() != referenceFrame}.
    */
   default void set(ReferenceFrame referenceFrame, double firstEndpointX, double firstEndpointY, double secondEndpointX, double secondEndpointY)
   {
      checkReferenceFrameMatch(referenceFrame);
      LineSegment2DBasics.super.set(firstEndpointX, firstEndpointY, secondEndpointX, secondEndpointY);
   }

   /**
    * Redefines this line segment with new endpoints.
    *
    * @param firstEndpoint new endpoint of this line segment. Not modified
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code referenceFrame} are not the
    *            same reference frame.
    */
   default void set(ReferenceFrame referenceFrame, Point2DReadOnly firstEndpoint, Point2DReadOnly secondEndpoint)
   {
      checkReferenceFrameMatch(referenceFrame);
      LineSegment2DBasics.super.set(firstEndpoint, secondEndpoint);
   }

   /**
    * Sets this line segment to be same as the given line segment.
    *
    * @param other the other line segment to copy. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
    */
   default void set(FrameLineSegment2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      LineSegment2DBasics.super.set(other);
   }

   /**
    * Sets this line segment to be same as the given line segment.
    *
    * @param other the other line segment to copy. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
    */
   default void set(FrameLineSegment3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      LineSegment2DBasics.super.set(other);
   }

   /**
    * Redefines this line segment with new endpoints.
    *
    * @param firstEndpoint new endpoint of this line segment. Not modified
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code firstEndpoint} are not
    *            expressed in the same reference frame.
    */
   default void set(FramePoint2DReadOnly firstEndpoint, Point2DReadOnly secondEndpoint)
   {
      checkReferenceFrameMatch(firstEndpoint);
      LineSegment2DBasics.super.set(firstEndpoint, secondEndpoint);
   }

   /**
    * Redefines this line segment with new endpoints.
    *
    * @param firstEndpoint new endpoint of this line segment. Not modified
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondEndpoint} are not
    *            expressed in the same reference frame.
    */
   default void set(Point2DReadOnly firstEndpoint, FramePoint2DReadOnly secondEndpoint)
   {
      checkReferenceFrameMatch(secondEndpoint);
      LineSegment2DBasics.super.set(firstEndpoint, secondEndpoint);
   }

   /**
    * Redefines this line segment with new endpoints.
    *
    * @param firstEndpoint new endpoint of this line segment. Not modified
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code firstEndpoint}, and
    *            {@code secondEndpoint} are not expressed in the same reference frame.
    */
   default void set(FramePoint2DReadOnly firstEndpoint, FramePoint2DReadOnly secondEndpoint)
   {
      checkReferenceFrameMatch(firstEndpoint);
      checkReferenceFrameMatch(secondEndpoint);
      LineSegment2DBasics.super.set(firstEndpoint, secondEndpoint);
   }

   /**
    * Redefines this line segment with new endpoints.
    *
    * @param firstEndpoint new endpoint of this line segment. Not modified
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code firstEndpoint} are not
    *            expressed in the same reference frame.
    */
   default void set(FramePoint3DReadOnly firstEndpoint, Point3DReadOnly secondEndpoint)
   {
      checkReferenceFrameMatch(firstEndpoint);
      LineSegment2DBasics.super.set(firstEndpoint, secondEndpoint);
   }

   /**
    * Redefines this line segment with new endpoints.
    *
    * @param firstEndpoint new endpoint of this line segment. Not modified
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondEndpoint} are not
    *            expressed in the same reference frame.
    */
   default void set(Point3DReadOnly firstEndpoint, FramePoint3DReadOnly secondEndpoint)
   {
      checkReferenceFrameMatch(secondEndpoint);
      LineSegment2DBasics.super.set(firstEndpoint, secondEndpoint);
   }

   /**
    * Redefines this line segment with new endpoints.
    *
    * @param firstEndpoint new endpoint of this line segment. Not modified
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code firstEndpoint}, and
    *            {@code secondEndpoint} are not expressed in the same reference frame.
    */
   default void set(FramePoint3DReadOnly firstEndpoint, FramePoint3DReadOnly secondEndpoint)
   {
      checkReferenceFrameMatch(firstEndpoint);
      checkReferenceFrameMatch(secondEndpoint);
      LineSegment2DBasics.super.set(firstEndpoint, secondEndpoint);
   }

   /**
    * Redefines this line segment with a new first endpoint and a vector going from the first to the
    * second endpoint.
    *
    * @param firstEndpoint new first endpoint. Not modified.
    * @param fromFirstToSecondEndpoint vector going from the first to the second endpoint. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code firstEndpoint} are not
    *            expressed in the same reference frame.
    */
   default void set(FramePoint2DReadOnly firstEndpoint, Vector2DReadOnly fromFirstToSecondEndpoint)
   {
      checkReferenceFrameMatch(firstEndpoint);
      LineSegment2DBasics.super.set(firstEndpoint, fromFirstToSecondEndpoint);
   }

   /**
    * Redefines this line segment with a new first endpoint and a vector going from the first to the
    * second endpoint.
    *
    * @param firstEndpoint new first endpoint. Not modified.
    * @param fromFirstToSecondEndpoint vector going from the first to the second endpoint. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code fromFirstToSecondEndpoint} are
    *            not expressed in the same reference frame.
    */
   default void set(Point2DReadOnly firstEndpoint, FrameVector2DReadOnly fromFirstToSecondEndpoint)
   {
      checkReferenceFrameMatch(fromFirstToSecondEndpoint);
      LineSegment2DBasics.super.set(firstEndpoint, fromFirstToSecondEndpoint);
   }

   /**
    * Redefines this line segment with a new first endpoint and a vector going from the first to the
    * second endpoint.
    *
    * @param firstEndpoint new first endpoint. Not modified.
    * @param fromFirstToSecondEndpoint vector going from the first to the second endpoint. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code firstEndpoint}, and
    *            {@code fromFirstToSecondEndpoint} are not expressed in the same reference frame.
    */
   default void set(FramePoint2DReadOnly firstEndpoint, FrameVector2DReadOnly fromFirstToSecondEndpoint)
   {
      checkReferenceFrameMatch(firstEndpoint);
      checkReferenceFrameMatch(fromFirstToSecondEndpoint);
      LineSegment2DBasics.super.set(firstEndpoint, fromFirstToSecondEndpoint);
   }

   /**
    * Redefines this line segment with a new first endpoint and a vector going from the first to the
    * second endpoint.
    *
    * @param firstEndpoint new first endpoint. Not modified.
    * @param fromFirstToSecondEndpoint vector going from the first to the second endpoint. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code firstEndpoint} are not
    *            expressed in the same reference frame.
    */
   default void set(FramePoint3DReadOnly firstEndpoint, Vector3DReadOnly fromFirstToSecondEndpoint)
   {
      checkReferenceFrameMatch(firstEndpoint);
      LineSegment2DBasics.super.set(firstEndpoint, fromFirstToSecondEndpoint);
   }

   /**
    * Redefines this line segment with a new first endpoint and a vector going from the first to the
    * second endpoint.
    *
    * @param firstEndpoint new first endpoint. Not modified.
    * @param fromFirstToSecondEndpoint vector going from the first to the second endpoint. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code fromFirstToSecondEndpoint} are
    *            not expressed in the same reference frame.
    */
   default void set(Point3DReadOnly firstEndpoint, FrameVector3DReadOnly fromFirstToSecondEndpoint)
   {
      checkReferenceFrameMatch(fromFirstToSecondEndpoint);
      LineSegment2DBasics.super.set(firstEndpoint, fromFirstToSecondEndpoint);
   }

   /**
    * Redefines this line segment with a new first endpoint and a vector going from the first to the
    * second endpoint.
    *
    * @param firstEndpoint new first endpoint. Not modified.
    * @param fromFirstToSecondEndpoint vector going from the first to the second endpoint. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code firstEndpoint}, and
    *            {@code fromFirstToSecondEndpoint} are not expressed in the same reference frame.
    */
   default void set(FramePoint3DReadOnly firstEndpoint, FrameVector3DReadOnly fromFirstToSecondEndpoint)
   {
      checkReferenceFrameMatch(firstEndpoint);
      checkReferenceFrameMatch(fromFirstToSecondEndpoint);
      LineSegment2DBasics.super.set(firstEndpoint, fromFirstToSecondEndpoint);
   }

   /**
    * Translates this line segment by the given (x, y, z) contained in {@code translation}.
    * <p>
    * Note that the length and direction of this line segment remains unchanged.
    * </p>
    *
    * @param translation the translation to add to each endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code translation} are not expressed
    *            in the same reference frame.
    */
   default void translate(FrameTuple2DReadOnly translation)
   {
      checkReferenceFrameMatch(translation);
      LineSegment2DBasics.super.translate(translation);
   }
}
