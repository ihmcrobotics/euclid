package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for a 3D vector expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Vector3DReadOnly}, a {@link ReferenceFrame} is associated to
 * a {@code FrameVector3DReadOnly}. This allows, for instance, to enforce, at runtime, that
 * operations on vectors occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameVector3DReadOnly} extends {@code Vector3DReadOnly}, it is compatible with
 * methods only requiring {@code Vector3DReadOnly}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameVector3DReadOnly}.
 * </p>
 */
public interface FrameVector3DReadOnly extends Vector3DReadOnly, FrameTuple3DReadOnly
{

   /**
    * Calculates and returns the value of the dot product of this frame vector with {@code other}.
    * <p>
    * For instance, the dot product of two vectors p and q is defined as: <br>
    * p . q = &sum;<sub>i=1:3</sub>(p<sub>i</sub> * q<sub>i</sub>)
    * </p>
    *
    * @param other the other frame vector used for the dot product. Not modified.
    * @return the value of the dot product.
    */
   default double dot(FrameVector3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return Vector3DReadOnly.super.dot(other);
   }

   /**
    * Calculates and returns the angle in radians from this frame vector to {@code other}.
    * <p>
    * The computed angle is in the range [0; <i>pi</i>].
    * </p>
    *
    * @param other the other frame vector used to compute the angle. Not modified.
    * @return the value of the angle from this frame vector to {@code other}.
    */
   default double angle(FrameVector3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return Vector3DReadOnly.super.angle(other);
   }

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
   default boolean geometricallyEquals(FrameVector3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Vector3DReadOnly.super.geometricallyEquals(other, epsilon);
   }
}
