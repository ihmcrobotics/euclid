package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

/**
 * Write and read interface for generic matrix 3D expressed in a changeable reference frame, i.e.
 * the reference frame in which this matrix is expressed can be changed.
 * <p>
 * In addition to representing a {@link Matrix3DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FrameMatrix3DBasics}. This allows, for instance, to enforce, at runtime, that operations
 * on matrices occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameMatrix3DBasics} extends {@code Matrix3DBasics}, it is compatible with
 * methods only requiring {@code Matrix3DBasics}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameMatrix3DBasics}.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameMatrix3DBasics extends FixedFrameMatrix3DBasics, FrameChangeable, FrameCommonMatrix3DBasics
{
   /**
    * Sets the reference frame of this matrix without updating or modifying any of its coefficients.
    *
    * @param referenceFrame the new reference frame for this frame matrix.
    */
   @Override
   void setReferenceFrame(ReferenceFrame referenceFrame);
}
