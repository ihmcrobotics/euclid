package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

/**
 * Write and read interface used for a 3-by-3 rotation matrix expressed in a changeable reference
 * frame, i.e. the reference frame in which this rotation matrix is expressed can be changed.
 * <p>
 * A rotation matrix is used to represent a 3D orientation through its 9 coefficients. A rotation
 * matrix has to comply to several constraints:
 * <ul>
 * <li>each column of the matrix represents a unitary vector,
 * <li>each row of the matrix represents a unitary vector,
 * <li>every pair of columns of the matrix represents two orthogonal vectors,
 * <li>every pair of rows of the matrix represents two orthogonal vectors,
 * <li>the matrix determinant is equal to {@code 1}.
 * </ul>
 * A rotation matrix has the nice property <i>R<sup>T</sup> = R<sup>-1</sup></i>.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameRotationMatrixBasics extends FixedFrameRotationMatrixBasics, FrameCommonMatrix3DBasics, FrameOrientation3DBasics
{
   /** {@inheritDoc} */
   @Override
   default void setIncludingFrame(ReferenceFrame referenceFrame, Orientation3DReadOnly orientation3dReadOnly)
   {
      FrameOrientation3DBasics.super.setIncludingFrame(referenceFrame, orientation3dReadOnly);
   }

   /** {@inheritDoc} */
   @Override
   default void setIncludingFrame(FrameOrientation3DReadOnly other)
   {
      FrameOrientation3DBasics.super.setIncludingFrame(other);
   }

   /** {@inheritDoc} */
   @Override
   default void setAndInvert(FrameOrientation3DReadOnly orientation)
   {
      FixedFrameRotationMatrixBasics.super.setAndInvert(orientation);
   }

   /** {@inheritDoc} */
   @Override
   default void setAndNormalize(FrameOrientation3DReadOnly orientation)
   {
      FixedFrameRotationMatrixBasics.super.setAndNormalize(orientation);
   }
}
