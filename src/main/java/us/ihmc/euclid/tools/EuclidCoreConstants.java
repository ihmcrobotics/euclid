package us.ihmc.euclid.tools;

import us.ihmc.euclid.Axis2D;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.interfaces.AffineTransformBasics;
import us.ihmc.euclid.transform.interfaces.AffineTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollBasics;

class EuclidCoreConstants
{
   /**
    * Constant representing the coordinates (0, 0) of the origin in the 2D plane.
    */
   public static final Point2DReadOnly origin2D = new Origin2D();

   /**
    * Constant representing the coordinates (0, 0, 0) of the origin in 3D.
    */
   public static final Point3DReadOnly origin3D = new Origin3D();

   /**
    * Constant representing the zero vector 2D: (0, 0).
    */
   public static final Vector2DReadOnly zeroVector2D = new ZeroVector2D();

   /**
    * Constant representing the zero vector 3D: (0, 0, 0).
    */
   public static final Vector3DReadOnly zeroVector3D = new ZeroVector3D();

   /**
    * Constant representing the neutral quaternion: (x=0, y=0, z=0, s=1).
    */
   public static final QuaternionReadOnly neutralQuaternion = new NeutralQuaternion();

   public static final RotationMatrixReadOnly identityRotationMatrix = new IdentityRotationMatrix();

   /** Constant representing the zero-matrix 3D, i.e. the 9 elements are zero. */
   public static final Matrix3DReadOnly zeroMatrix3D = new ZeroMatrix3D();

   /**
    * Constant representing the identity matrix 3D, i.e. the 3 diagonal elements are equal to one and
    * the other equal to zero.
    */
   public static final Matrix3DReadOnly identityMatrix3D = new IdentityMatrix3D();

   public static final RigidBodyTransformReadOnly identityTransform = new IdentityRigidBodyTransform();

   private static final class Origin2D implements Point2DReadOnly
   {
      @Override
      public double getX()
      {
         return 0.0;
      }

      @Override
      public double getY()
      {
         return 0.0;
      }

      @Override
      public double dot(Tuple2DReadOnly other)
      {
         return 0.0;
      }

      @Override
      public double distanceFromOrigin()
      {
         return 0.0;
      }

      @Override
      public double distanceFromOriginSquared()
      {
         return 0.0;
      }

      @Override
      public double distance(Point2DReadOnly other)
      {
         return other.distanceFromOrigin();
      }

      @Override
      public double distanceSquared(Point2DReadOnly other)
      {
         return other.distanceFromOriginSquared();
      }

      @Override
      public double norm()
      {
         return 0.0;
      }

      @Override
      public double normSquared()
      {
         return 0.0;
      }

      @Override
      public double differenceNorm(Tuple2DReadOnly other)
      {
         return other.norm();
      }

      @Override
      public double differenceNormSquared(Tuple2DReadOnly other)
      {
         return other.normSquared();
      }

      @Override
      public double getElement(Axis2D axis)
      {
         return 0.0;
      }

      @Override
      public double getElement(int index)
      {
         return 0.0;
      }

      @Override
      public float getElement32(int index)
      {
         return 0.0f;
      }

      @Override
      public float getX32()
      {
         return 0.0f;
      }

      @Override
      public float getY32()
      {
         return 0.0f;
      }

      @Override
      public boolean containsNaN()
      {
         return false;
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof Tuple2DReadOnly)
            return Point2DReadOnly.super.equals((Tuple2DReadOnly) object);
         else
            return false;
      }

      @Override
      public int hashCode()
      {
         return 1;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }
   }

   private static final class Origin3D implements Point3DReadOnly
   {
      @Override
      public double getX()
      {
         return 0.0;
      }

      @Override
      public double getY()
      {
         return 0.0;
      }

      @Override
      public double getZ()
      {
         return 0.0;
      }

      @Override
      public double dot(Tuple3DReadOnly other)
      {
         return 0.0;
      }

      @Override
      public double distanceFromOrigin()
      {
         return 0.0;
      }

      @Override
      public double distanceFromOriginSquared()
      {
         return 0.0;
      }

      @Override
      public double distance(Point3DReadOnly other)
      {
         return other.distanceFromOrigin();
      }

      @Override
      public double distanceSquared(Point3DReadOnly other)
      {
         return other.distanceFromOriginSquared();
      }

      @Override
      public double norm()
      {
         return 0.0;
      }

      @Override
      public double normSquared()
      {
         return 0.0;
      }

      @Override
      public double differenceNorm(Tuple3DReadOnly other)
      {
         return other.norm();
      }

      @Override
      public double differenceNormSquared(Tuple3DReadOnly other)
      {
         return other.normSquared();
      }

      @Override
      public double getElement(Axis3D axis)
      {
         return 0.0;
      }

      @Override
      public double getElement(int index)
      {
         return 0.0;
      }

      @Override
      public float getElement32(int index)
      {
         return 0.0f;
      }

      @Override
      public float getX32()
      {
         return 0.0f;
      }

      @Override
      public float getY32()
      {
         return 0.0f;
      }

      @Override
      public float getZ32()
      {
         return 0.0f;
      }

      @Override
      public boolean containsNaN()
      {
         return false;
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof Tuple3DReadOnly)
            return Point3DReadOnly.super.equals((Tuple3DReadOnly) object);
         else
            return false;
      }

      @Override
      public int hashCode()
      {
         return 1;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }
   }

   private static final class ZeroVector2D implements Vector2DReadOnly
   {
      @Override
      public double getX()
      {
         return 0.0;
      }

      @Override
      public double getY()
      {
         return 0.0;
      }

      @Override
      public double cross(Tuple2DReadOnly tuple)
      {
         return 0.0;
      }

      @Override
      public double dot(Tuple2DReadOnly other)
      {
         return 0.0;
      }

      @Override
      public double angle(Vector2DReadOnly other)
      {
         return Double.NaN;
      }

      @Override
      public double norm()
      {
         return 0.0;
      }

      @Override
      public double normSquared()
      {
         return 0.0;
      }

      @Override
      public double differenceNorm(Tuple2DReadOnly other)
      {
         return other.norm();
      }

      @Override
      public double differenceNormSquared(Tuple2DReadOnly other)
      {
         return other.normSquared();
      }

      @Override
      public double getElement(Axis2D axis)
      {
         return 0.0;
      }

      @Override
      public double getElement(int index)
      {
         return 0.0;
      }

      @Override
      public float getElement32(int index)
      {
         return 0.0f;
      }

      @Override
      public float getX32()
      {
         return 0.0f;
      }

      @Override
      public float getY32()
      {
         return 0.0f;
      }

      @Override
      public boolean containsNaN()
      {
         return false;
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof Tuple2DReadOnly)
            return Vector2DReadOnly.super.equals((Tuple2DReadOnly) object);
         else
            return false;
      }

      @Override
      public int hashCode()
      {
         return 1;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }
   }

   private static final class ZeroVector3D implements Vector3DReadOnly
   {
      @Override
      public double getX()
      {
         return 0.0;
      }

      @Override
      public double getY()
      {
         return 0.0;
      }

      @Override
      public double getZ()
      {
         return 0.0;
      }

      @Override
      public double dot(Tuple3DReadOnly other)
      {
         return 0.0;
      }

      @Override
      public double angle(Vector3DReadOnly other)
      {
         return Double.NaN;
      }

      @Override
      public double norm()
      {
         return 0.0;
      }

      @Override
      public double normSquared()
      {
         return 0.0;
      }

      @Override
      public double differenceNorm(Tuple3DReadOnly other)
      {
         return other.norm();
      }

      @Override
      public double differenceNormSquared(Tuple3DReadOnly other)
      {
         return other.normSquared();
      }

      @Override
      public double getElement(Axis3D axis)
      {
         return 0.0;
      }

      @Override
      public double getElement(int index)
      {
         return 0.0;
      }

      @Override
      public float getElement32(int index)
      {
         return 0.0f;
      }

      @Override
      public float getX32()
      {
         return 0.0f;
      }

      @Override
      public float getY32()
      {
         return 0.0f;
      }

      @Override
      public float getZ32()
      {
         return 0.0f;
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof Tuple3DReadOnly)
            return Vector3DReadOnly.super.equals((Tuple3DReadOnly) object);
         else
            return false;
      }

      @Override
      public int hashCode()
      {
         return 1;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }
   }

   private static final class NeutralQuaternion implements QuaternionReadOnly
   {
      @Override
      public double getX()
      {
         return 0;
      }

      @Override
      public double getY()
      {
         return 0.0;
      }

      @Override
      public double getZ()
      {
         return 0.0;
      }

      @Override
      public double getS()
      {
         return 1.0;
      }

      @Override
      public double angle()
      {
         return 0.0;
      }

      @Override
      public double angle(boolean limitToPi)
      {
         return 0.0;
      }

      @Override
      public double norm()
      {
         return 1.0;
      }

      @Override
      public double normSquared()
      {
         return 1.0;
      }

      @Override
      public double differenceNorm(Tuple4DReadOnly other)
      {
         return other.norm();
      }

      @Override
      public double differenceNormSquared(Tuple4DReadOnly other)
      {
         return other.normSquared();
      }

      @Override
      public double distance(Orientation3DReadOnly other)
      {
         return other.angle();
      }

      @Override
      public double distance(Orientation3DReadOnly other, boolean limitToPi)
      {
         return other.angle(limitToPi);
      }

      @Override
      public double dot(Tuple4DReadOnly other)
      {
         return other.getS();
      }

      @Override
      public void get(AxisAngleBasics axisAngleToPack)
      {
         axisAngleToPack.setToZero();
      }

      @Override
      public void get(CommonMatrix3DBasics rotationMatrixToPack)
      {
         rotationMatrixToPack.setIdentity();
      }

      @Override
      public void get(QuaternionBasics quaternionToPack)
      {
         quaternionToPack.setToZero();
      }

      @Override
      public void get(YawPitchRollBasics yawPitchRollToPack)
      {
         yawPitchRollToPack.setToZero();
      }

      @Override
      public void getEuler(Tuple3DBasics eulerAnglesToPack)
      {
         eulerAnglesToPack.setToZero();
      }

      @Override
      public void getRotationVector(Vector3DBasics rotationVectorToPack)
      {
         rotationVectorToPack.setToZero();
      }

      @Override
      public double getYaw()
      {
         return 0.0;
      }

      @Override
      public double getPitch()
      {
         return 0.0;
      }

      @Override
      public double getRoll()
      {
         return 0.0;
      }

      @Override
      public float getElement32(int index)
      {
         return index == 3 ? 1.0f : 0.0f;
      }

      @Override
      public double getElement(int index)
      {
         return index == 3 ? 1.0 : 0.0;
      }

      @Override
      public float getX32()
      {
         return 0.0f;
      }

      @Override
      public float getY32()
      {
         return 0.0f;
      }

      @Override
      public float getZ32()
      {
         return 0.0f;
      }

      @Override
      public float getS32()
      {
         return 0.0f;
      }

      @Override
      public boolean isOrientation2D()
      {
         return true;
      }

      @Override
      public boolean isOrientation2D(double epsilon)
      {
         return true;
      }

      @Override
      public boolean isUnitary(double epsilon)
      {
         return true;
      }

      @Override
      public boolean isZeroOrientation()
      {
         return true;
      }

      @Override
      public boolean isZeroOrientation(double epsilon)
      {
         return true;
      }

      @Override
      public void checkIfOrientation2D()
      {
      }

      @Override
      public void checkIfOrientation2D(double epsilon)
      {
      }

      @Override
      public void checkIfUnitary()
      {
      }

      @Override
      public void checkIfUnitary(double epsilon)
      {
      }

      @Override
      public void addTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
      {
         tupleTransformed.add(tupleOriginal);
      }

      @Override
      public void subTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
      {
         tupleTransformed.sub(tupleOriginal);
      }

      @Override
      public void transform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
      {
         if (matrixOriginal != matrixTransformed)
            matrixTransformed.set(matrixOriginal);
      }

      @Override
      public void transform(Orientation3DReadOnly orientationOriginal, Orientation3DBasics orientationTransformed)
      {
         if (orientationOriginal != orientationTransformed)
            orientationTransformed.set(orientationOriginal);
      }

      @Override
      public void transform(RotationMatrixReadOnly matrixOriginal, RotationMatrixBasics matrixTransformed)
      {
         if (matrixOriginal != matrixTransformed)
            matrixTransformed.set(matrixOriginal);
      }

      @Override
      public void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfOrientation2D)
      {
         if (tupleOriginal != tupleTransformed)
            tupleTransformed.set(tupleOriginal);
      }

      @Override
      public void transform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
      {
         if (tupleOriginal != tupleTransformed)
            tupleTransformed.set(tupleOriginal);
      }

      @Override
      public void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
      {
         if (vectorOriginal != vectorTransformed)
            vectorTransformed.set(vectorOriginal);
      }

      @Override
      public void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
      {
         if (matrixOriginal != matrixTransformed)
            matrixTransformed.set(matrixOriginal);
      }

      @Override
      public void inverseTransform(Orientation3DReadOnly orientationOriginal, Orientation3DBasics orientationTransformed)
      {
         if (orientationOriginal != orientationTransformed)
            orientationTransformed.set(orientationOriginal);
      }

      @Override
      public void inverseTransform(RotationMatrixReadOnly matrixOriginal, RotationMatrixBasics matrixTransformed)
      {
         if (matrixOriginal != matrixTransformed)
            matrixTransformed.set(matrixOriginal);
      }

      @Override
      public void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfOrientation2D)
      {
         if (tupleOriginal != tupleTransformed)
            tupleTransformed.set(tupleOriginal);
      }

      @Override
      public void inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
      {
         if (tupleOriginal != tupleTransformed)
            tupleTransformed.set(tupleOriginal);
      }

      @Override
      public void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
      {
         if (vectorOriginal != vectorTransformed)
            vectorTransformed.set(vectorOriginal);
      }

      @Override
      public boolean containsNaN()
      {
         return false;
      }

      @Override
      public int hashCode()
      {
         return -1106247679;
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
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }
   }

   private static final class IdentityRotationMatrix implements RotationMatrixReadOnly
   {
      @Override
      public double getM00()
      {
         return 1;
      }

      @Override
      public double getM01()
      {
         return 0;
      }

      @Override
      public double getM02()
      {
         return 0;
      }

      @Override
      public double getM10()
      {
         return 0;
      }

      @Override
      public double getM11()
      {
         return 1;
      }

      @Override
      public double getM12()
      {
         return 0;
      }

      @Override
      public double getM20()
      {
         return 0;
      }

      @Override
      public double getM21()
      {
         return 0;
      }

      @Override
      public double getM22()
      {
         return 1;
      }

      @Override
      public double angle()
      {
         return 0.0;
      }

      @Override
      public double angle(boolean limitToPi)
      {
         return 0.0;
      }

      @Override
      public void get(AxisAngleBasics axisAngleToPack)
      {
         axisAngleToPack.setToZero();
      }

      @Override
      public void get(CommonMatrix3DBasics rotationMatrixToPack)
      {
         rotationMatrixToPack.setIdentity();
      }

      @Override
      public void get(QuaternionBasics quaternionToPack)
      {
         quaternionToPack.setToZero();
      }

      @Override
      public void get(YawPitchRollBasics yawPitchRollToPack)
      {
         yawPitchRollToPack.setToZero();
      }

      @Override
      public void getEuler(Tuple3DBasics eulerAnglesToPack)
      {
         eulerAnglesToPack.setToZero();
      }

      @Override
      public void getRotationVector(Vector3DBasics rotationVectorToPack)
      {
         rotationVectorToPack.setToZero();
      }

      @Override
      public double getYaw()
      {
         return 0.0;
      }

      @Override
      public double getPitch()
      {
         return 0.0;
      }

      @Override
      public double getRoll()
      {
         return 0.0;
      }

      @Override
      public boolean isIdentity()
      {
         return true;
      }

      @Override
      public boolean isIdentity(double epsilon)
      {
         return true;
      }

      @Override
      public boolean isDirty()
      {
         return false;
      }

      @Override
      public boolean isMatrix2D()
      {
         return true;
      }

      @Override
      public boolean isMatrix2D(double epsilon)
      {
         return true;
      }

      @Override
      public boolean isMatrixSkewSymmetric()
      {
         return false;
      }

      @Override
      public boolean isMatrixSkewSymmetric(double epsilon)
      {
         return false;
      }

      @Override
      public boolean isMatrixSymmetric()
      {
         return true;
      }

      @Override
      public boolean isMatrixSymmetric(double epsilon)
      {
         return true;
      }

      @Override
      public boolean isOrientation2D()
      {
         return true;
      }

      @Override
      public boolean isOrientation2D(double epsilon)
      {
         return true;
      }

      @Override
      public boolean isPositiveDefiniteMatrix()
      {
         return true;
      }

      @Override
      public boolean isRotationMatrix()
      {
         return true;
      }

      @Override
      public boolean isRotationMatrix(double epsilon)
      {
         return true;
      }

      @Override
      public boolean isZero()
      {
         return false;
      }

      @Override
      public boolean isZero(double epsilon)
      {
         return false;
      }

      @Override
      public boolean isZeroOrientation()
      {
         return true;
      }

      @Override
      public boolean isZeroOrientation(double epsilon)
      {
         return true;
      }

      @Override
      public void checkIfMatrix2D()
      {
      }

      @Override
      public void checkIfOrientation2D()
      {
      }

      @Override
      public void checkIfOrientation2D(double epsilon)
      {
      }

      @Override
      public void checkIfPositiveDefiniteMatrix()
      {
      }

      @Override
      public void checkIfRotationMatrix()
      {
      }

      @Override
      public void addTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
      {
         tupleTransformed.add(tupleOriginal);
      }

      @Override
      public void subTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
      {
         tupleTransformed.sub(tupleOriginal);
      }

      @Override
      public void transform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
      {
         if (matrixOriginal != matrixTransformed)
            matrixTransformed.set(matrixOriginal);
      }

      @Override
      public void transform(Orientation3DReadOnly orientationOriginal, Orientation3DBasics orientationTransformed)
      {
         if (orientationOriginal != orientationTransformed)
            orientationTransformed.set(orientationOriginal);
      }

      @Override
      public void transform(RotationMatrixReadOnly matrixOriginal, RotationMatrixBasics matrixTransformed)
      {
         if (matrixOriginal != matrixTransformed)
            matrixTransformed.set(matrixOriginal);
      }

      @Override
      public void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfOrientation2D)
      {
         if (tupleOriginal != tupleTransformed)
            tupleTransformed.set(tupleOriginal);
      }

      @Override
      public void transform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
      {
         if (tupleOriginal != tupleTransformed)
            tupleTransformed.set(tupleOriginal);
      }

      @Override
      public void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
      {
         if (vectorOriginal != vectorTransformed)
            vectorTransformed.set(vectorOriginal);
      }

      @Override
      public void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
      {
         if (matrixOriginal != matrixTransformed)
            matrixTransformed.set(matrixOriginal);
      }

      @Override
      public void inverseTransform(Orientation3DReadOnly orientationOriginal, Orientation3DBasics orientationTransformed)
      {
         if (orientationOriginal != orientationTransformed)
            orientationTransformed.set(orientationOriginal);
      }

      @Override
      public void inverseTransform(RotationMatrixReadOnly matrixOriginal, RotationMatrixBasics matrixTransformed)
      {
         if (matrixOriginal != matrixTransformed)
            matrixTransformed.set(matrixOriginal);
      }

      @Override
      public void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfOrientation2D)
      {
         if (tupleOriginal != tupleTransformed)
            tupleTransformed.set(tupleOriginal);
      }

      @Override
      public void inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
      {
         if (tupleOriginal != tupleTransformed)
            tupleTransformed.set(tupleOriginal);
      }

      @Override
      public void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
      {
         if (vectorOriginal != vectorTransformed)
            vectorTransformed.set(vectorOriginal);
      }

      @Override
      public boolean containsNaN()
      {
         return false;
      }

      @Override
      public double determinant()
      {
         return 1.0;
      }

      @Override
      public double distance(Orientation3DReadOnly other)
      {
         return other.angle();
      }

      @Override
      public double distance(Orientation3DReadOnly other, boolean limitToPi)
      {
         return other.angle(limitToPi);
      }

      @Override
      public double getElement(int row, int column)
      {
         return row == column ? 1.0 : 0.0;
      }

      @Override
      public double maxElement()
      {
         return 1.0;
      }

      @Override
      public double maxAbsElement()
      {
         return 1.0;
      }

      @Override
      public double minElement()
      {
         return 0.0;
      }

      @Override
      public double minAbsElement()
      {
         return 0.0;
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof RotationMatrixReadOnly)
            return RotationMatrixReadOnly.super.equals((RotationMatrixReadOnly) object);
         else
            return false;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }

      @Override
      public int hashCode()
      {
         return 976224257;
      }
   }

   private static final class ZeroMatrix3D implements Matrix3DReadOnly
   {
      @Override
      public double getM00()
      {
         return 0;
      }

      @Override
      public double getM01()
      {
         return 0;
      }

      @Override
      public double getM02()
      {
         return 0;
      }

      @Override
      public double getM10()
      {
         return 0;
      }

      @Override
      public double getM11()
      {
         return 0;
      }

      @Override
      public double getM12()
      {
         return 0;
      }

      @Override
      public double getM20()
      {
         return 0;
      }

      @Override
      public double getM21()
      {
         return 0;
      }

      @Override
      public double getM22()
      {
         return 0;
      }

      @Override
      public boolean isIdentity()
      {
         return false;
      }

      @Override
      public boolean isIdentity(double epsilon)
      {
         return false;
      }

      @Override
      public boolean isMatrix2D()
      {
         return false;
      }

      @Override
      public boolean isMatrix2D(double epsilon)
      {
         return false;
      }

      @Override
      public boolean isMatrixSkewSymmetric()
      {
         return true;
      }

      @Override
      public boolean isMatrixSkewSymmetric(double epsilon)
      {
         return true;
      }

      @Override
      public boolean isMatrixSymmetric()
      {
         return true;
      }

      @Override
      public boolean isMatrixSymmetric(double epsilon)
      {
         return true;
      }

      @Override
      public boolean isPositiveDefiniteMatrix()
      {
         return false;
      }

      @Override
      public boolean isRotationMatrix()
      {
         return false;
      }

      @Override
      public boolean isRotationMatrix(double epsilon)
      {
         return false;
      }

      @Override
      public boolean isZero()
      {
         return true;
      }

      @Override
      public boolean isZero(double epsilon)
      {
         return true;
      }

      @Override
      public void addTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
      {
      }

      @Override
      public void subTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
      {
      }

      @Override
      public void transform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
      {
         matrixTransformed.setToZero();
      }

      @Override
      public void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfRotationInXYPlane)
      {
         tupleTransformed.setToZero();
      }

      @Override
      public void transform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
      {
         tupleTransformed.setToZero();
      }

      @Override
      public void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
      {
         vectorTransformed.setToZero();
      }

      @Override
      public void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
      {
         matrixTransformed.setToZero();
      }

      @Override
      public void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
      {
         tupleTransformed.setToZero();
      }

      @Override
      public void inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
      {
         tupleTransformed.setToZero();
      }

      @Override
      public void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
      {
         vectorTransformed.setToZero();
      }

      @Override
      public double determinant()
      {
         return 0.0;
      }

      @Override
      public double getElement(int row, int column)
      {
         return 0.0;
      }

      @Override
      public boolean containsNaN()
      {
         return false;
      }

      @Override
      public double maxElement()
      {
         return 0.0;
      }

      @Override
      public double maxAbsElement()
      {
         return 0.0;
      }

      @Override
      public double minElement()
      {
         return 0.0;
      }

      @Override
      public double minAbsElement()
      {
         return 0.0;
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof Matrix3DReadOnly)
            return Matrix3DReadOnly.super.equals((Matrix3DReadOnly) object);
         else
            return false;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }

      @Override
      public int hashCode()
      {
         return 1;
      }
   }

   private static final class IdentityMatrix3D implements Matrix3DReadOnly
   {
      @Override
      public boolean isIdentity()
      {
         return true;
      }

      @Override
      public boolean isIdentity(double epsilon)
      {
         return true;
      }

      @Override
      public boolean isMatrix2D()
      {
         return true;
      }

      @Override
      public boolean isMatrix2D(double epsilon)
      {
         return true;
      }

      @Override
      public boolean isMatrixSkewSymmetric()
      {
         return false;
      }

      @Override
      public boolean isMatrixSkewSymmetric(double epsilon)
      {
         return false;
      }

      @Override
      public boolean isMatrixSymmetric()
      {
         return true;
      }

      @Override
      public boolean isMatrixSymmetric(double epsilon)
      {
         return true;
      }

      @Override
      public boolean isPositiveDefiniteMatrix()
      {
         return true;
      }

      @Override
      public boolean isRotationMatrix()
      {
         return true;
      }

      @Override
      public boolean isRotationMatrix(double epsilon)
      {
         return true;
      }

      @Override
      public boolean isZero()
      {
         return false;
      }

      @Override
      public boolean isZero(double epsilon)
      {
         return false;
      }

      @Override
      public double getM00()
      {
         return 1;
      }

      @Override
      public double getM01()
      {
         return 0;
      }

      @Override
      public double getM02()
      {
         return 0;
      }

      @Override
      public double getM10()
      {
         return 0;
      }

      @Override
      public double getM11()
      {
         return 1;
      }

      @Override
      public double getM12()
      {
         return 0;
      }

      @Override
      public double getM20()
      {
         return 0;
      }

      @Override
      public double getM21()
      {
         return 0;
      }

      @Override
      public double getM22()
      {
         return 1;
      }

      @Override
      public void addTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
      {
         tupleTransformed.add(tupleOriginal);
      }

      @Override
      public void subTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
      {
         tupleTransformed.sub(tupleOriginal);
      }

      @Override
      public void transform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
      {
         if (matrixOriginal != matrixTransformed)
            matrixTransformed.set(matrixOriginal);
      }

      @Override
      public void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfRotationInXYPlane)
      {
         if (tupleOriginal != tupleTransformed)
            tupleTransformed.set(tupleOriginal);
      }

      @Override
      public void transform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
      {
         if (tupleOriginal != tupleTransformed)
            tupleTransformed.set(tupleOriginal);
      }

      @Override
      public void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
      {
         if (vectorOriginal != vectorTransformed)
            vectorTransformed.set(vectorOriginal);
      }

      @Override
      public void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
      {
         if (matrixOriginal != matrixTransformed)
            matrixTransformed.set(matrixOriginal);
      }

      @Override
      public void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
      {
         if (tupleOriginal != tupleTransformed)
            tupleTransformed.set(tupleOriginal);
      }

      @Override
      public void inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
      {
         if (tupleOriginal != tupleTransformed)
            tupleTransformed.set(tupleOriginal);
      }

      @Override
      public void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
      {
         if (vectorOriginal != vectorTransformed)
            vectorTransformed.set(vectorOriginal);
      }

      @Override
      public double determinant()
      {
         return 1.0;
      }

      @Override
      public double getElement(int row, int column)
      {
         return row == column ? 1.0 : 0.0;
      }

      @Override
      public boolean containsNaN()
      {
         return false;
      }

      @Override
      public double maxElement()
      {
         return 1.0;
      }

      @Override
      public double maxAbsElement()
      {
         return 1.0;
      }

      @Override
      public double minElement()
      {
         return 0.0;
      }

      @Override
      public double minAbsElement()
      {
         return 0.0;
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof Matrix3DReadOnly)
            return Matrix3DReadOnly.super.equals((Matrix3DReadOnly) object);
         else
            return false;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }

      @Override
      public int hashCode()
      {
         return 976224257;
      }
   }

   private static final class IdentityRigidBodyTransform implements RigidBodyTransformReadOnly
   {
      @Override
      public Vector3DReadOnly getTranslation()
      {
         return zeroVector3D;
      }

      @Override
      public RotationMatrixReadOnly getRotation()
      {
         return identityRotationMatrix;
      }

      @Override
      public String toString(String format)
      {
         return "Identity";
      }

      @Override
      public void checkIfRotation2D()
      {
      }

      @Override
      public void get(Orientation3DBasics orientationToPack, Tuple3DBasics translationToPack)
      {
         orientationToPack.setToZero();
         translationToPack.setToZero();
      }

      @Override
      public void get(RotationMatrixBasics rotationMatrixToPack, Tuple3DBasics translationToPack)
      {
         rotationMatrixToPack.setToZero();
         translationToPack.setToZero();
      }

      @Override
      public void get(Vector3DBasics rotationVectorToPack, Tuple3DBasics translationToPack)
      {
         rotationVectorToPack.setToZero();
         translationToPack.setToZero();
      }

      @Override
      public boolean hasRotation()
      {
         return false;
      }

      @Override
      public boolean hasTranslation()
      {
         return false;
      }

      @Override
      public void transform(AffineTransformReadOnly original, AffineTransformBasics transformed)
      {
         if (transformed != original)
            transformed.set(original);
      }

      @Override
      public void transform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
      {
         if (matrixTransformed != matrixOriginal)
            matrixTransformed.set(matrixOriginal);
      }

      @Override
      public void transform(Orientation3DReadOnly orientationOriginal, Orientation3DBasics orientationTransformed)
      {
         if (orientationTransformed != orientationOriginal)
            orientationTransformed.set(orientationOriginal);
      }

      @Override
      public void transform(Point2DReadOnly point2DOriginal, Point2DBasics point2DTransformed, boolean checkIfTransformInXYPlane)
      {
         if (point2DTransformed != point2DOriginal)
            point2DTransformed.set(point2DOriginal);
      }

      @Override
      public void transform(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed)
      {
         if (pointTransformed != pointOriginal)
            pointTransformed.set(pointOriginal);
      }

      @Override
      public void transform(RigidBodyTransformReadOnly original, RigidBodyTransformBasics transformed)
      {
         if (transformed != original)
            transformed.set(original);
      }

      @Override
      public void transform(RotationMatrixReadOnly matrixOriginal, RotationMatrixBasics matrixTransformed)
      {
         if (matrixTransformed != matrixOriginal)
            matrixTransformed.set(matrixOriginal);
      }

      @Override
      public void transform(Vector2DReadOnly vector2DOriginal, Vector2DBasics vector2DTransformed, boolean checkIfTransformInXYPlane)
      {
         if (vector2DTransformed != vector2DOriginal)
            vector2DTransformed.set(vector2DOriginal);
      }

      @Override
      public void transform(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed)
      {
         if (vectorTransformed != vectorOriginal)
            vectorTransformed.set(vectorOriginal);
      }

      @Override
      public void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
      {
         if (vectorTransformed != vectorOriginal)
            vectorTransformed.set(vectorOriginal);
      }

      @Override
      public void inverseTransform(AffineTransformReadOnly original, AffineTransformBasics transformed)
      {
         if (transformed != original)
            transformed.set(original);
      }

      @Override
      public void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
      {
         if (matrixTransformed != matrixOriginal)
            matrixTransformed.set(matrixOriginal);
      }

      @Override
      public void inverseTransform(Orientation3DReadOnly orientationOriginal, Orientation3DBasics orientationTransformed)
      {
         if (orientationTransformed != orientationOriginal)
            orientationTransformed.set(orientationOriginal);
      }

      @Override
      public void inverseTransform(Point2DReadOnly point2DOriginal, Point2DBasics point2DTransformed, boolean checkIfTransformInXYPlane)
      {
         if (point2DTransformed != point2DOriginal)
            point2DTransformed.set(point2DOriginal);
      }

      @Override
      public void inverseTransform(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed)
      {
         if (pointTransformed != pointOriginal)
            pointTransformed.set(pointOriginal);
      }

      @Override
      public void inverseTransform(RigidBodyTransformReadOnly original, RigidBodyTransformBasics transformed)
      {
         if (transformed != original)
            transformed.set(original);
      }

      @Override
      public void inverseTransform(RotationMatrixReadOnly matrixOriginal, RotationMatrixBasics matrixTransformed)
      {
         if (matrixTransformed != matrixOriginal)
            matrixTransformed.set(matrixOriginal);
      }

      @Override
      public void inverseTransform(Vector2DReadOnly vector2DOriginal, Vector2DBasics vector2DTransformed, boolean checkIfTransformInXYPlane)
      {
         if (vector2DTransformed != vector2DOriginal)
            vector2DTransformed.set(vector2DOriginal);
      }

      @Override
      public void inverseTransform(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed)
      {
         if (vectorTransformed != vectorOriginal)
            vectorTransformed.set(vectorOriginal);
      }

      @Override
      public void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
      {
         if (vectorTransformed != vectorOriginal)
            vectorTransformed.set(vectorOriginal);
      }

      @Override
      public boolean isRotation2D()
      {
         return true;
      }

      @Override
      public boolean containsNaN()
      {
         return false;
      }

      @Override
      public boolean equals(EuclidGeometry geometry)
      {
         if (geometry == this)
            return true;
         if (geometry == null)
            return false;
         if (!(geometry instanceof RigidBodyTransformReadOnly))
            return false;
         RigidBodyTransformReadOnly other = (RigidBodyTransformReadOnly) geometry;
         return other.getRotation().isZeroOrientation() && getTranslation().equals(other.getTranslation());
      }

      @Override
      public boolean epsilonEquals(EuclidGeometry geometry, double epsilon)
      {
         if (geometry == this)
            return true;
         if (geometry == null)
            return false;
         if (!(geometry instanceof RigidBodyTransformReadOnly))
            return false;
         RigidBodyTransformReadOnly other = (RigidBodyTransformReadOnly) geometry;
         return other.getRotation().isZeroOrientation(epsilon) && getTranslation().epsilonEquals(other.getTranslation(), epsilon);
      }
   }
}
