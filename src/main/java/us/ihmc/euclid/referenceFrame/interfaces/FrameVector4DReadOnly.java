package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

/**
 * Read-only interface for a 4D vector expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Vector4DReadOnly}, a {@link ReferenceFrame} is associated to
 * a {@code FrameVector4DReadOnly}. This allows, for instance, to enforce, at runtime, that
 * operations on vectors occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameVector4DReadOnly} extends {@code Vector4DReadOnly}, it is compatible with
 * methods only requiring {@code Vector4DReadOnly}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameVector4DReadOnly}.
 * </p>
 */
public interface FrameVector4DReadOnly extends Vector4DReadOnly, FrameTuple4DReadOnly
{
   /**
    * Compares {@code this} to {@code other} to determine if the two frame vectors are geometrically
    * similar, i.e. the length of the distance between them is less than or equal to {@code epsilon}.
    *
    * @param other the frame vector to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two frame vectors represent the same geometry, {@code false}
    *         otherwise.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default boolean geometricallyEquals(FrameVector4DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Vector4DReadOnly.super.geometricallyEquals(other, epsilon);
   }
}
