package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameMatrix3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameMatrix3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class FrameMatrix3D implements FrameMatrix3DBasics, GeometryObject<FrameMatrix3D>
{
   private ReferenceFrame referenceFrame;
   private final Matrix3D matrix3D = new Matrix3D();

   private final RigidBodyTransform transformToDesiredFrame = new RigidBodyTransform();

   public FrameMatrix3D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public FrameMatrix3D(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   public FrameMatrix3D(FrameMatrix3DReadOnly frameMatrix3D)
   {
      setIncludingFrame(frameMatrix3D);
   }

   public FrameMatrix3D(ReferenceFrame referenceFrame, Matrix3DReadOnly matrix)
   {
      setIncludingFrame(referenceFrame, matrix);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public void set(FrameMatrix3D other)
   {
      FrameMatrix3DBasics.super.set(other);
   }

   @Override
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      if (desiredFrame == referenceFrame)
         return;

      referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, desiredFrame);
      applyTransform(transformToDesiredFrame);
      setReferenceFrame(desiredFrame);
   }

   @Override
   public void setM00(double m00)
   {
      matrix3D.setM00(m00);
   }

   @Override
   public void setM01(double m01)
   {
      matrix3D.setM01(m01);
   }

   @Override
   public void setM02(double m02)
   {
      matrix3D.setM02(m02);
   }

   @Override
   public void setM10(double m10)
   {
      matrix3D.setM10(m10);
   }

   @Override
   public void setM11(double m11)
   {
      matrix3D.setM11(m11);
   }

   @Override
   public void setM12(double m12)
   {
      matrix3D.setM12(m12);
   }

   @Override
   public void setM20(double m20)
   {
      matrix3D.setM20(m20);
   }

   @Override
   public void setM21(double m21)
   {
      matrix3D.setM21(m21);
   }

   @Override
   public void setM22(double m22)
   {
      matrix3D.setM22(m22);
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   @Override
   public double getM00()
   {
      return matrix3D.getM00();
   }

   @Override
   public double getM01()
   {
      return matrix3D.getM01();
   }

   @Override
   public double getM02()
   {
      return matrix3D.getM02();
   }

   @Override
   public double getM10()
   {
      return matrix3D.getM10();
   }

   @Override
   public double getM11()
   {
      return matrix3D.getM11();
   }

   @Override
   public double getM12()
   {
      return matrix3D.getM12();
   }

   @Override
   public double getM20()
   {
      return matrix3D.getM20();
   }

   @Override
   public double getM21()
   {
      return matrix3D.getM21();
   }

   @Override
   public double getM22()
   {
      return matrix3D.getM22();
   }

   /**
    * Two 3D matrices are considered geometrically equal if they are epsilon equal.
    * <p>
    * This method is equivalent to {@link #epsilonEquals(FrameMatrix3D, double)}.
    * </p>
    *
    * @param other the other matrix to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two matrices are equal, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(FrameMatrix3D other, double epsilon)
   {
      return epsilonEquals(other, epsilon);
   }

   @Override
   public boolean epsilonEquals(FrameMatrix3D other, double epsilon)
   {
      return FrameMatrix3DBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameMatrix3DReadOnly)
         return FrameMatrix3DBasics.super.equals((FrameMatrix3DReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return matrix3D.toString() + "\n" + referenceFrame;
   }

   @Override
   public int hashCode()
   {
      return matrix3D.hashCode();
   }
}
