package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public interface FixedFrameMatrix3DBasics extends FrameMatrix3DReadOnly, Matrix3DBasics, Transformable
{
   default void set(ReferenceFrame referenceFrame, double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   default void set(ReferenceFrame referenceFrame, Matrix3DReadOnly matrix3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(matrix3DReadOnly);
   }

   default void set(FrameMatrix3DReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   default void setMatchingFrame(FrameMatrix3DReadOnly other)
   {
      Matrix3DBasics.super.set(other);
      other.getReferenceFrame().transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setToTildeForm(FrameTuple3DReadOnly frameTuple)
   {
      checkReferenceFrameMatch(frameTuple);
      Matrix3DBasics.super.setToTildeForm(frameTuple);
   }

   default void setToDiagonal(FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      checkReferenceFrameMatch(frameTuple3DReadOnly);
      Matrix3DBasics.super.setToDiagonal(frameTuple3DReadOnly);
   }

   default void setAndMultiplyOuter(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.setAndMultiplyOuter(other);
   }

   default void setAndInvert(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.setAndInvert(other);
   }

   default void setAndNormalize(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.setAndNormalize(other);
   }

   default void setAndTranspose(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.setAndTranspose(other);
   }

   default void setAndNegate(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.setAndNegate(other);
   }

   default void setRow(int row, FrameTuple3DReadOnly rowValues)
   {
      checkReferenceFrameMatch(rowValues);
      Matrix3DBasics.super.setRow(row, rowValues);
   }

   default void setColumn(int column, FrameTuple3DReadOnly columnValues)
   {
      checkReferenceFrameMatch(columnValues);
      Matrix3DBasics.super.setColumn(column, columnValues);
   }

   default void add(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.add(other);
   }

   default void add(FrameMatrix3DReadOnly matrix1, Matrix3DReadOnly matrix2)
   {
      checkReferenceFrameMatch(matrix1);
      Matrix3DBasics.super.add(matrix1, matrix2);
   }

   default void add(Matrix3DReadOnly matrix1, FrameMatrix3DReadOnly matrix2)
   {
      checkReferenceFrameMatch(matrix2);
      Matrix3DBasics.super.add(matrix1, matrix2);
   }

   default void add(FrameMatrix3DReadOnly matrix1, FrameMatrix3DReadOnly matrix2)
   {
      checkReferenceFrameMatch(matrix1);
      checkReferenceFrameMatch(matrix2);
      Matrix3DBasics.super.add(matrix1, matrix2);
   }

   default void sub(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.sub(other);
   }

   default void sub(FrameMatrix3DReadOnly matrix1, Matrix3DReadOnly matrix2)
   {
      checkReferenceFrameMatch(matrix1);
      Matrix3DBasics.super.sub(matrix1, matrix2);
   }

   default void sub(Matrix3DReadOnly matrix1, FrameMatrix3DReadOnly matrix2)
   {
      checkReferenceFrameMatch(matrix2);
      Matrix3DBasics.super.sub(matrix1, matrix2);
   }

   default void sub(FrameMatrix3DReadOnly matrix1, FrameMatrix3DReadOnly matrix2)
   {
      checkReferenceFrameMatch(matrix1);
      checkReferenceFrameMatch(matrix2);
      Matrix3DBasics.super.sub(matrix1, matrix2);
   }

   default void multiply(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.multiply(other);
   }

   default void multiplyTransposeThis(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.multiplyTransposeThis(other);
   }

   default void multiplyTransposeOther(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.multiplyTransposeOther(other);
   }

   default void multiplyTransposeBoth(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.multiplyTransposeBoth(other);
   }

   default void multiplyInvertThis(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.multiplyInvertThis(other);
   }

   default void multiplyInvertOther(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.multiplyInvertOther(other);
   }

   default void preMultiply(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.preMultiply(other);
   }

   default void preMultiplyTransposeThis(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.preMultiplyTransposeThis(other);
   }

   default void preMultiplyTransposeOther(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.preMultiplyTransposeOther(other);
   }

   default void preMultiplyTransposeBoth(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.preMultiplyTransposeBoth(other);
   }

   default void preMultiplyInvertThis(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.preMultiplyInvertThis(other);
   }

   default void preMultiplyInvertOther(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.preMultiplyInvertOther(other);
   }

}
