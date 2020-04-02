package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Write and read interface for a 3D axis-aligned bounding box expressed in a changeable reference
 * frame, i.e. the reference frame in which this object is expressed can be changed.
 */
public interface FrameBoundingBox3DBasics extends FixedFrameBoundingBox3DBasics
{
   /**
    * Sets the reference frame of this bounding box 3D without updating or modifying its min or max
    * coordinates.
    *
    * @param referenceFrame the new reference frame for this frame bounding box 3D.
    */
   void setReferenceFrame(ReferenceFrame referenceFrame);

   /**
    * Sets the min and max coordinates of this bounding box 3D to zero and sets the current reference
    * frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame to be associated with this bounding box 3D.
    */
   default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToZero();
   }

   /**
    * Sets the min and max coordinates of this bounding box 3D to {@link Double#NaN} and sets the
    * current reference frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame to be associated with this bounding box 3D.
    */
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToNaN();
   }

   /**
    * Redefines this bounding box with new minimum and maximum coordinates.
    *
    * @param referenceFrame the new reference frame to be associated with this bounding box 3D.
    * @param min            the new minimum coordinates for this bounding box. Not modified.
    * @param max            the new maximum coordinates for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *                          coordinate on the same axis.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly min, Point3DReadOnly max)
   {
      setReferenceFrame(referenceFrame);
      set(min, max);
   }

   /**
    * Redefines this bounding box with new minimum and maximum coordinates and reference frame from the
    * arguments.
    *
    * @param min the new minimum coordinates for this bounding box. Not modified.
    * @param max the new maximum coordinates for this bounding box. Not modified.
    * @throws RuntimeException                if any of the minimum coordinates is strictly greater
    *                                         than the maximum coordinate on the same axis.
    * @throws ReferenceFrameMismatchException if {@code min} and {@code max} are not expressed in the
    *                                         same reference frame
    */
   default void setIncludingFrame(FramePoint3DReadOnly min, FramePoint3DReadOnly max)
   {
      min.checkReferenceFrameMatch(max);
      setIncludingFrame(min.getReferenceFrame(), min, max);
   }

   /**
    * Redefines this bounding box given its {@code center} location and half its size along each axis
    * {@code halfSize}.
    *
    * @param referenceFrame the new reference frame to be associated with this bounding box 3D.
    * @param center         the new center location of this bounding box. Not modified.
    * @param halfSize       half the size of this bounding box. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly center, Vector3DReadOnly halfSize)
   {
      setReferenceFrame(referenceFrame);
      set(center, halfSize);
   }

   /**
    * Redefines this bounding box given its {@code center} location and half its size along each axis
    * {@code halfSize} and its reference frame obtained from the arguments.
    *
    * @param center   the new center location of this bounding box. Not modified.
    * @param halfSize half the size of this bounding box. Not modified.
    * @throws ReferenceFrameMismatchException if {@code center} and {@code halfSize} are not expressed
    *                                         in the same reference frame
    */
   default void setIncludingFrame(FramePoint3DReadOnly center, FrameVector3DReadOnly halfSize)
   {
      center.checkReferenceFrameMatch(halfSize);
      setIncludingFrame(center.getReferenceFrame(), center, halfSize);
   }

   /**
    * Redefines this bounding box to be the same as the given {@code other}.
    *
    * @param referenceFrame the new reference frame to be associated with this bounding box 3D.
    * @param other          the bounding box used to redefine this bounding box. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, BoundingBox3DReadOnly other)
   {
      setReferenceFrame(referenceFrame);
      set(other);
   }

   /**
    * Redefines this bounding box to be the same as the given {@code other}.
    *
    * @param other the bounding box used to redefine this bounding box. Not modified.
    */
   default void setIncludingFrame(FrameBoundingBox3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }
}
