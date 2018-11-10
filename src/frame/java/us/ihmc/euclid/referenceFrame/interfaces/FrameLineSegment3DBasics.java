package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.LineSegment3DBasics;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Write and read interface for a line segment 3D expressed in a changeable reference frame, i.e.
 * the reference frame in which this line is expressed can be changed.
 * <p>
 * A line segment 3D is a finite-length line defined in the XY-plane by its two 3D endpoints.
 * </p>
 * <p>
 * In addition to representing a {@link LineSegment3DBasics}, a {@link ReferenceFrame} is associated
 * to a {@code FrameLineSegment3DBasics}. This allows, for instance, to enforce, at runtime, that
 * operations on lines occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameLineSegment3DBasics} extends {@code LineSegment3DBasics}, it is compatible
 * with methods only requiring {@code LineSegment3DBasics}. However, these methods do NOT assert
 * that the operation occur in the proper coordinate system. Use this feature carefully and always
 * prefer using methods requiring {@code FrameLineSegment3DBasics}.
 * </p>
 */
public interface FrameLineSegment3DBasics extends FixedFrameLineSegment3DBasics, FrameChangeable
{
   /**
    * Sets the reference frame of this line segment 3D without updating or modifying either of its
    * endpoints.
    *
    * @param referenceFrame the new reference frame for this frame line segment 3D.
    */
   @Override
   void setReferenceFrame(ReferenceFrame referenceFrame);

   /**
    * Sets the point and direction parts of this line segment 3D to zero and sets the current reference
    * frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame to be associated with this line segment 3D.
    */
   default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToZero();
   }

   /**
    * Sets the point and direction parts of this line segment 3D to {@link Double#NaN} and sets the
    * current reference frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame to be associated with this line segment 3D.
    */
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToNaN();
   }

   /**
    * Sets this line segment to be same as the given line segment including its reference frame.
    *
    * @param other the other line segment to copy. Not modified.
    */
   default void setIncludingFrame(FrameLineSegment3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   /**
    * Sets this line segment to be same as the given line segment including its reference frame.
    *
    * @param referenceFrame the new reference frame to be associated with this line segment 3D.
    * @param lineSegment3DReadOnly the line segment to copy. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, LineSegment3DReadOnly lineSegment3DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(lineSegment3DReadOnly);
   }

   /**
    * Redefines this line segment with new endpoints.
    *
    * @param firstEndpoint new endpoint of this line segment. Not modified
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code firstEndpoint} and {@code secondEndpoint} are
    *            not expressed in the same reference frame.
    */
   default void setIncludingFrame(FramePoint3DReadOnly firstEndpoint, FramePoint3DReadOnly secondEndpoint)
   {
      firstEndpoint.checkReferenceFrameMatch(secondEndpoint);
      setIncludingFrame(firstEndpoint.getReferenceFrame(), firstEndpoint, secondEndpoint);
   }

   /**
    * Redefines this line segment with a new first endpoint and a vector going from the first to the
    * second endpoint.
    *
    * @param referenceFrame the new reference frame to be associated with this line segment 3D.
    * @param firstEndpoint new first endpoint. Not modified.
    * @param secondEndpoint new second endpoint. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly firstEndpoint, Point3DReadOnly secondEndpoint)
   {
      setReferenceFrame(referenceFrame);
      set(firstEndpoint, secondEndpoint);
   }

   /**
    * Redefines this line segment with a new first endpoint and a vector going from the first to the
    * second endpoint.
    *
    * @param firstEndpoint new first endpoint. Not modified.
    * @param fromFirstToSecondEndpoint vector going from the first to the second endpoint. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if {@code firstEndpoint} and
    *            {@code fromFirstToSecondEndpoint} are not expressed in the same reference frame.
    */
   default void setIncludingFrame(FramePoint3DReadOnly firstEndpoint, FrameVector3DReadOnly fromFirstToSecondEndpoint)
   {
      firstEndpoint.checkReferenceFrameMatch(fromFirstToSecondEndpoint);
      setIncludingFrame(firstEndpoint.getReferenceFrame(), firstEndpoint, fromFirstToSecondEndpoint);
   }

   /**
    * Redefines this line segment with a new first endpoint and a vector going from the first to the
    * second endpoint.
    *
    * @param referenceFrame the new reference frame to be associated with this line segment 3D.
    * @param firstEndpoint new first endpoint. Not modified.
    * @param fromFirstToSecondEndpoint vector going from the first to the second endpoint. Not
    *           modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly firstEndpoint, Vector3DReadOnly fromFirstToSecondEndpoint)
   {
      setReferenceFrame(referenceFrame);
      set(firstEndpoint, fromFirstToSecondEndpoint);
   }
}
