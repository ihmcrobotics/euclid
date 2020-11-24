package us.ihmc.euclid.transform;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.rotationConversion.QuaternionConversion;
import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tools.Matrix3DFeatures;
import us.ihmc.euclid.tools.SingularValueDecomposition3D;
import us.ihmc.euclid.tools.SingularValueDecomposition3D.SVD3DOutput;
import us.ihmc.euclid.transform.interfaces.LinearTransform3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class LinearTransform3D implements LinearTransform3DBasics
{
   private double m00, m01, m02, m10, m11, m12, m20, m21, m22;

   private boolean isRotation = true;
   private boolean rotationDirty = true;
   private boolean isIdentity = true;
   private boolean identityDirty = true;

   private boolean svdDirty = true;
   private final SingularValueDecomposition3D svd3D = new SingularValueDecomposition3D();
   private final SVD3DOutput svdOutput = svd3D.getOutput();
   private final QuaternionReadOnly U = EuclidCoreFactories.newObservableQuaternionReadOnly(i -> updateSVD(), svdOutput.getU());
   private final QuaternionReadOnly Vt = EuclidCoreFactories.newObservableQuaternionReadOnly(i -> updateSVD(),
                                                                                             EuclidCoreFactories.newConjugateLinkedQuaternion(svdOutput.getV()));

   private boolean quaternionViewDirty = true;
   private final QuaternionReadOnly quaternionView = new QuaternionReadOnly()
   {
      private double x, y, z, s;

      private void update()
      {
         if (quaternionViewDirty)
         {
            quaternionViewDirty = false;
            updateSVD();

            double ux = U.getX(), uy = U.getY(), uz = U.getZ(), us = U.getS();

            if (isRotationMatrix())
            {
               x = ux;
               y = uy;
               z = uz;
               s = us;
            }
            else
            {
               double vx = Vt.getX(), vy = Vt.getY(), vz = Vt.getZ(), vs = Vt.getS();
               x = us * vx + ux * vs + uy * vz - uz * vy;
               y = us * vy - ux * vz + uy * vs + uz * vx;
               z = us * vz + ux * vy - uy * vx + uz * vs;
               s = us * vs - ux * vx - uy * vy - uz * vz;
            }
         }
      }

      @Override
      public double getX()
      {
         update();
         return x;
      }

      @Override
      public double getY()
      {
         update();
         return y;
      }

      @Override
      public double getZ()
      {
         update();
         return z;
      }

      @Override
      public double getS()
      {
         update();
         return s;
      }

      @Override
      public int hashCode()
      {
         return EuclidHashCodeTools.toIntHashCode(getX(), getY(), getZ(), getS());
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof QuaternionReadOnly)
            return equals((QuaternionReadOnly) object);
         else
            return false;
      }

      @Override
      public String toString()
      {
         return EuclidCoreIOTools.getTuple4DString(this);
      }
   };

   private final Vector3DReadOnly scaleView = EuclidCoreFactories.newObservableVector3DReadOnly(a -> updateSVD(), svdOutput.getW());

   public LinearTransform3D()
   {
      setIdentity();
   }

   public LinearTransform3D(Matrix3DReadOnly matrix3D)
   {
      set(matrix3D);
   }

   public LinearTransform3D(DMatrix matrix)
   {
      set(matrix);
   }

   public LinearTransform3D(Orientation3DReadOnly orientation)
   {
      set(orientation);
   }

   public LinearTransform3D(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   private void updateSVD()
   {
      if (svdDirty)
      {
         if (isRotationMatrix())
         { // Abusing the SVD output to store this matrix as a quaternion
            svdDirty = false;
            QuaternionConversion.convertMatrixToQuaternion(m00, m01, m02, m10, m11, m12, m20, m21, m22, svdOutput.getU());
            svdOutput.getW().set(1.0, 1.0, 1.0);
            svdOutput.getV().setToZero();
         }
         else
         {
            svdDirty = false;
            svd3D.decompose(this);
         }
      }
   }

   @Override
   public double determinant()
   {
      if (svdDirty)
         return LinearTransform3DBasics.super.determinant();
      else
         return svdOutput.getW().getX() * svdOutput.getW().getY() * svdOutput.getW().getZ();
   }

   @Override
   public void setIdentity()
   {
      m00 = 1.0;
      m01 = 0.0;
      m02 = 0.0;
      m10 = 0.0;
      m11 = 1.0;
      m12 = 0.0;
      m20 = 0.0;
      m21 = 0.0;
      m22 = 1.0;
      isRotation = true;
      rotationDirty = false;
      isIdentity = true;
      identityDirty = false;
      quaternionViewDirty = true;
      svdDirty = false;
      svdOutput.setIdentity();
   }

   @Override
   public void setToNaN()
   {
      m00 = Double.NaN;
      m01 = Double.NaN;
      m02 = Double.NaN;
      m10 = Double.NaN;
      m11 = Double.NaN;
      m12 = Double.NaN;
      m20 = Double.NaN;
      m21 = Double.NaN;
      m22 = Double.NaN;
      isRotation = false;
      rotationDirty = false;
      isIdentity = false;
      identityDirty = false;
      quaternionViewDirty = true;
      svdDirty = false;
      svdOutput.setToNaN();
   }

   @Override
   public void resetScale()
   {
      if (isIdentity() || isRotationMatrix())
         return;

      updateSVD();
      svdOutput.getW().set(1.0, 1.0, 1.0);
      svdOutput.getU().appendInvertOther(svdOutput.getV());
      svdOutput.getV().setToZero();
      svdOutput.getU().get(this);
      rotationDirty = false;
      isRotation = true;
      quaternionViewDirty = true;
   }

   @Override
   public boolean isIdentity()
   {
      if (identityDirty)
      {
         identityDirty = false;
         isIdentity = LinearTransform3DBasics.super.isIdentity();
         if (isIdentity)
            isRotation = true;
      }
      return isIdentity;
   }

   @Override
   public void transpose()
   {
      double temp;

      temp = m01;
      m01 = m10;
      m10 = temp;

      temp = m02;
      m02 = m20;
      m20 = temp;

      temp = m12;
      m12 = m21;
      m21 = temp;

      quaternionViewDirty = true;
      if (!svdDirty)
         svd3D.transpose();
   }

   @Override
   public void invert()
   {
      if (isIdentity())
         return;

      boolean invertSVDOutput = !svdDirty;

      if (isRotationMatrix())
      {
         double temp;

         temp = m01;
         m01 = m10;
         m10 = temp;

         temp = m02;
         m02 = m20;
         m20 = temp;

         temp = m12;
         m12 = m21;
         m21 = temp;
      }
      else
      {
         LinearTransform3DBasics.super.invert();
      }
      quaternionViewDirty = true;

      if (invertSVDOutput)
      {
         svdOutput.invert();
         svdDirty = false;
      }
   }

   @Override
   public void set(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      this.m00 = m00;
      this.m01 = m01;
      this.m02 = m02;
      this.m10 = m10;
      this.m11 = m11;
      this.m12 = m12;
      this.m20 = m20;
      this.m21 = m21;
      this.m22 = m22;

      svdDirty = true;
      identityDirty = true;
      rotationDirty = true;
      quaternionViewDirty = true;
   }

   @Override
   public void set(Matrix3DReadOnly matrix3D)
   {
      if (matrix3D instanceof RotationMatrixReadOnly)
         set((RotationMatrixReadOnly) matrix3D);
      else if (matrix3D instanceof LinearTransform3D)
         set((LinearTransform3D) matrix3D);
      else
         LinearTransform3DBasics.super.set(matrix3D);
   }

   @Override
   public void set(RotationMatrixReadOnly rotationMatrix)
   {
      m00 = rotationMatrix.getM00();
      m01 = rotationMatrix.getM01();
      m02 = rotationMatrix.getM02();
      m10 = rotationMatrix.getM10();
      m11 = rotationMatrix.getM11();
      m12 = rotationMatrix.getM12();
      m20 = rotationMatrix.getM20();
      m21 = rotationMatrix.getM21();
      m22 = rotationMatrix.getM22();

      isRotation = true;
      rotationDirty = false;
      identityDirty = rotationMatrix.isDirty();
      isIdentity = identityDirty ? false : rotationMatrix.isIdentity();
      quaternionViewDirty = true;
      svdDirty = true;
   }

   public void set(LinearTransform3D other)
   {
      m00 = other.m00;
      m01 = other.m01;
      m02 = other.m02;
      m10 = other.m10;
      m11 = other.m11;
      m12 = other.m12;
      m20 = other.m20;
      m21 = other.m21;
      m22 = other.m22;

      isRotation = other.isRotation;
      rotationDirty = other.rotationDirty;
      isIdentity = other.isIdentity;
      identityDirty = other.identityDirty;
      quaternionViewDirty = true;
      svdDirty = other.svdDirty;
      if (!svdDirty)
         svdOutput.set(other.svdOutput);
   }

   @Override
   public void set(Orientation3DReadOnly orientation3D)
   {
      LinearTransform3DBasics.super.set(orientation3D);
      isRotation = true;
      rotationDirty = false;
      quaternionViewDirty = true;
   }

   @Override
   public void setRotationVector(Vector3DReadOnly rotationVector)
   {
      LinearTransform3DBasics.super.setRotationVector(rotationVector);
      isRotation = true;
      rotationDirty = false;
      quaternionViewDirty = true;
   }

   @Override
   public void setEuler(Tuple3DReadOnly eulerAngles)
   {
      LinearTransform3DBasics.super.setEuler(eulerAngles);
      isRotation = true;
      rotationDirty = false;
      quaternionViewDirty = true;
   }

   /** {@inheritDoc} */
   @Override
   public void setM00(double m00)
   {
      if (this.m00 != m00)
      {
         svdDirty = true;
         identityDirty = true;
         rotationDirty = true;
         quaternionViewDirty = true;
         this.m00 = m00;
      }
   }

   /** {@inheritDoc} */
   @Override
   public void setM01(double m01)
   {
      if (this.m01 != m01)
      {
         svdDirty = true;
         identityDirty = true;
         rotationDirty = true;
         quaternionViewDirty = true;
         this.m01 = m01;
      }
   }

   /** {@inheritDoc} */
   @Override
   public void setM02(double m02)
   {
      if (this.m02 != m02)
      {
         svdDirty = true;
         identityDirty = true;
         rotationDirty = true;
         quaternionViewDirty = true;
         this.m02 = m02;
      }
   }

   /** {@inheritDoc} */
   @Override
   public void setM10(double m10)
   {
      if (this.m10 != m10)
      {
         svdDirty = true;
         identityDirty = true;
         rotationDirty = true;
         quaternionViewDirty = true;
         this.m10 = m10;
      }
   }

   /** {@inheritDoc} */
   @Override
   public void setM11(double m11)
   {
      if (this.m11 != m11)
      {
         svdDirty = true;
         identityDirty = true;
         rotationDirty = true;
         quaternionViewDirty = true;
         this.m11 = m11;
      }
   }

   /** {@inheritDoc} */
   @Override
   public void setM12(double m12)
   {
      if (this.m12 != m12)
      {
         svdDirty = true;
         identityDirty = true;
         rotationDirty = true;
         quaternionViewDirty = true;
         this.m12 = m12;
      }
   }

   /** {@inheritDoc} */
   @Override
   public void setM20(double m20)
   {
      if (this.m20 != m20)
      {
         svdDirty = true;
         identityDirty = true;
         rotationDirty = true;
         quaternionViewDirty = true;
         this.m20 = m20;
      }
   }

   /** {@inheritDoc} */
   @Override
   public void setM21(double m21)
   {
      if (this.m21 != m21)
      {
         svdDirty = true;
         identityDirty = true;
         rotationDirty = true;
         quaternionViewDirty = true;
         this.m21 = m21;
      }
   }

   /** {@inheritDoc} */
   @Override
   public void setM22(double m22)
   {
      if (this.m22 != m22)
      {
         svdDirty = true;
         identityDirty = true;
         rotationDirty = true;
         quaternionViewDirty = true;
         this.m22 = m22;
      }
   }

   @Override
   public boolean isRotationMatrix()
   {
      if (rotationDirty)
      {
         rotationDirty = false;
         if (!svdDirty)
         {
            isRotation = EuclidCoreTools.epsilonEquals(1.0, getScaleX(), Matrix3DFeatures.EPS_CHECK_ROTATION)
                  && EuclidCoreTools.epsilonEquals(1.0, getScaleY(), Matrix3DFeatures.EPS_CHECK_ROTATION)
                  && EuclidCoreTools.epsilonEquals(1.0, getScaleZ(), Matrix3DFeatures.EPS_CHECK_ROTATION);
         }
         else
         {
            isRotation = LinearTransform3DBasics.super.isRotationMatrix();
         }
      }
      return isRotation;
   }

   @Override
   public QuaternionReadOnly getAsQuaternion()
   {
      return quaternionView;
   }

   @Override
   public QuaternionReadOnly getPreScaleQuaternion()
   {
      updateSVD();
      return U;
   }

   @Override
   public Vector3DReadOnly getScaleVector()
   {
      return scaleView;
   }

   @Override
   public QuaternionReadOnly getPostScaleQuaternion()
   {
      updateSVD();
      return Vt;
   }

   /** {@inheritDoc} */
   @Override
   public double getM00()
   {
      return m00;
   }

   /** {@inheritDoc} */
   @Override
   public double getM01()
   {
      return m01;
   }

   /** {@inheritDoc} */
   @Override
   public double getM02()
   {
      return m02;
   }

   /** {@inheritDoc} */
   @Override
   public double getM10()
   {
      return m10;
   }

   /** {@inheritDoc} */
   @Override
   public double getM11()
   {
      return m11;
   }

   /** {@inheritDoc} */
   @Override
   public double getM12()
   {
      return m12;
   }

   /** {@inheritDoc} */
   @Override
   public double getM20()
   {
      return m20;
   }

   /** {@inheritDoc} */
   @Override
   public double getM21()
   {
      return m21;
   }

   /** {@inheritDoc} */
   @Override
   public double getM22()
   {
      return m22;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(Matrix3DReadOnly)}, it returns {@code false} otherwise or if the {@code object} is
    * {@code null}.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Matrix3DReadOnly)
         return equals((Matrix3DReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this matrix as follows:
    *
    * <pre>
    * /m00, m01, m02 \
    * |m10, m11, m12 |
    * \m20, m21, m22 /
    * </pre>
    *
    * @return the {@code String} representing this matrix.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getMatrix3DString(this);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this matrix.
    *
    * @return the hash code value for this matrix.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }
}