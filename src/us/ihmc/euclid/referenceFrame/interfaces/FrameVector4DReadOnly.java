package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
    * Calculates and returns the value of the dot product of this frame vector with {@code other}.
    * <p>
    * For instance, the dot product of two vectors p and q is defined as: <br>
    * p . q = &sum;<sub>i=1:3</sub>(p<sub>i</sub> * q<sub>i</sub>)
    * </p>
    *
    * @param other the other frame vector used for the dot product. Not modified.
    * @return the value of the dot product.
    */
   default double dot(FrameVector4DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return Vector4DReadOnly.super.dot(other);
   }
}
