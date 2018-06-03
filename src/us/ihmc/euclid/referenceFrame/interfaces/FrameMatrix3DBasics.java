package us.ihmc.euclid.referenceFrame.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public interface FrameMatrix3DBasics extends FixedFrameMatrix3DBasics, FrameChangeable
{
   @Override
   void setReferenceFrame(ReferenceFrame referenceFrame);

   default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToZero();
   }

   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToNaN();
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Matrix3DReadOnly matrix3DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(matrix3DReadOnly);
   }

   default void setIncludingFrame(FrameMatrix3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, double[] matrixArray)
   {
      setReferenceFrame(referenceFrame);
      set(matrixArray);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, int startIndex, double[] matrixArray)
   {
      setReferenceFrame(referenceFrame);
      set(startIndex, matrixArray);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, DenseMatrix64F matrix)
   {
      setReferenceFrame(referenceFrame);
      set(matrix);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, int startRow, int startColumn, DenseMatrix64F matrix)
   {
      setReferenceFrame(referenceFrame);
      set(startRow, startColumn, matrix);
   }
}
