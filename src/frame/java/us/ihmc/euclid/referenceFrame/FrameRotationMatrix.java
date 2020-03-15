package us.ihmc.euclid.referenceFrame;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameMatrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRotationMatrixBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class FrameRotationMatrix implements FrameRotationMatrixBasics, GeometryObject<FrameRotationMatrix>
{
   private ReferenceFrame referenceFrame;
   private final RotationMatrix rotationMatrix = new RotationMatrix();

   public FrameRotationMatrix()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   public FrameRotationMatrix(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   public FrameRotationMatrix(ReferenceFrame referenceFrame, double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21,
                              double m22)
   {
      setIncludingFrame(referenceFrame, m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   public FrameRotationMatrix(ReferenceFrame referenceFrame, double[] matrixArray)
   {
      setIncludingFrame(referenceFrame, matrixArray);
   }

   public FrameRotationMatrix(ReferenceFrame referenceFrame, DenseMatrix64F matrix)
   {
      setIncludingFrame(referenceFrame, matrix);
   }

   public FrameRotationMatrix(ReferenceFrame referenceFrame, RotationMatrixReadOnly rotationMatrixReadOnly)
   {
      setIncludingFrame(referenceFrame, rotationMatrixReadOnly);
   }

   public FrameRotationMatrix(ReferenceFrame referenceFrame, Orientation3DReadOnly orientation3DReadOnly)
   {
      setIncludingFrame(referenceFrame, orientation3DReadOnly);
   }

   public FrameRotationMatrix(ReferenceFrame referenceFrame, Vector3DReadOnly rotationVector)
   {
      setRotationVectorIncludingFrame(referenceFrame, rotationVector);
   }

   public FrameRotationMatrix(ReferenceFrame referenceFrame, double yaw, double pitch, double roll)
   {
      setYawPitchRollIncludingFrame(referenceFrame, yaw, pitch, roll);
   }

   public FrameRotationMatrix(FrameRotationMatrixReadOnly other)
   {
      setIncludingFrame(other);
   }

   public FrameRotationMatrix(FrameOrientation3DReadOnly orientation)
   {
      setIncludingFrame(orientation);
   }

   @Override
   public void set(FrameRotationMatrix other)
   {
      FrameRotationMatrixBasics.super.set(other);
   }

   @Override
   public void set(RotationMatrixReadOnly other)
   {
      rotationMatrix.set(other);
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   @Override
   public void setUnsafe(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public boolean isDirty()
   {
      return rotationMatrix.isDirty();
   }

   /** {@inheritDoc} */
   @Override
   public double getM00()
   {
      return rotationMatrix.getM00();
   }

   /** {@inheritDoc} */
   @Override
   public double getM01()
   {
      return rotationMatrix.getM01();
   }

   /** {@inheritDoc} */
   @Override
   public double getM02()
   {
      return rotationMatrix.getM02();
   }

   /** {@inheritDoc} */
   @Override
   public double getM10()
   {
      return rotationMatrix.getM10();
   }

   /** {@inheritDoc} */
   @Override
   public double getM11()
   {
      return rotationMatrix.getM11();
   }

   /** {@inheritDoc} */
   @Override
   public double getM12()
   {
      return rotationMatrix.getM12();
   }

   /** {@inheritDoc} */
   @Override
   public double getM20()
   {
      return rotationMatrix.getM20();
   }

   /** {@inheritDoc} */
   @Override
   public double getM21()
   {
      return rotationMatrix.getM21();
   }

   /** {@inheritDoc} */
   @Override
   public double getM22()
   {
      return rotationMatrix.getM22();
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameMatrix3DReadOnly)
         return FrameRotationMatrixBasics.super.equals((FrameMatrix3DReadOnly) object);
      else
         return false;
   }

   @Override
   public boolean epsilonEquals(FrameRotationMatrix other, double epsilon)
   {
      return FrameRotationMatrixBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean geometricallyEquals(FrameRotationMatrix other, double epsilon)
   {
      return FrameRotationMatrixBasics.super.geometricallyEquals(other, epsilon);
   }

   @Override
   public String toString()
   {
      return EuclidFrameIOTools.getFrameMatrix3DString(this);
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(rotationMatrix, referenceFrame);
   }
}
