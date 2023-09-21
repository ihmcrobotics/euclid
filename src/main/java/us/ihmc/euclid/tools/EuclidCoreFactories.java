package us.ihmc.euclid.tools;

import java.util.function.DoubleSupplier;

import us.ihmc.euclid.Axis2D;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.Axis4D;
import us.ihmc.euclid.Matrix3DElements;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.UnitVector2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.UnitVector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.UnitVector2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.UnitVector4DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

/**
 * This class provides a varieties of factories to create Euclid types.
 *
 * @author Sylvain Bertrand
 */
public class EuclidCoreFactories
{
   private EuclidCoreFactories()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Creates a new point that is linked to the {@code originalTuple} as follows:
    *
    * <pre>
    * linkedPoint = scale * originalTuple
    * </pre>
    *
    * where the scale is obtained from the given {@code scaleSupplier}.
    *
    * @param scaleSupplier the supplier to get the scale.
    * @param originalTuple the reference tuple to scale. Not modified.
    * @return the new point linked to {@code originalTuple}.
    */
   public static Point2DReadOnly newLinkedPoint2DReadOnly(DoubleSupplier scaleSupplier, Tuple2DReadOnly originalTuple)
   {
      DoubleSupplier xSupplier = () -> scaleSupplier.getAsDouble() * originalTuple.getX();
      DoubleSupplier ySupplier = () -> scaleSupplier.getAsDouble() * originalTuple.getY();
      return newLinkedPoint2DReadOnly(xSupplier, ySupplier);
   }

   /**
    * Creates a new vector that is linked to the {@code originalTuple} as follows:
    *
    * <pre>
    * linkedVector = scale * originalTuple
    * </pre>
    *
    * where the scale is obtained from the given {@code scaleSupplier}.
    *
    * @param scaleSupplier the supplier to get the scale.
    * @param originalTuple the reference tuple to scale. Not modified.
    * @return the new vector linked to {@code originalTuple}.
    */
   public static Vector2DReadOnly newLinkedVector2DReadOnly(DoubleSupplier scaleSupplier, Tuple2DReadOnly originalTuple)
   {
      DoubleSupplier xSupplier = () -> scaleSupplier.getAsDouble() * originalTuple.getX();
      DoubleSupplier ySupplier = () -> scaleSupplier.getAsDouble() * originalTuple.getY();
      return newLinkedVector2DReadOnly(xSupplier, ySupplier);
   }

   /**
    * Creates a new point that is linked to the {@code originalTuple} as follows:
    *
    * <pre>
    * linkedPoint = scale * originalTuple
    * </pre>
    *
    * where the scale is obtained from the given {@code scaleSupplier}.
    *
    * @param scaleSupplier the supplier to get the scale.
    * @param originalTuple the reference tuple to scale. Not modified.
    * @return the new point linked to {@code originalTuple}.
    */
   public static Point3DReadOnly newLinkedPoint3DReadOnly(DoubleSupplier scaleSupplier, Tuple3DReadOnly originalTuple)
   {
      DoubleSupplier xSupplier = () -> scaleSupplier.getAsDouble() * originalTuple.getX();
      DoubleSupplier ySupplier = () -> scaleSupplier.getAsDouble() * originalTuple.getY();
      DoubleSupplier zSupplier = () -> scaleSupplier.getAsDouble() * originalTuple.getZ();
      return newLinkedPoint3DReadOnly(xSupplier, ySupplier, zSupplier);
   }

   /**
    * Creates a new vector that is linked to the {@code originalTuple} as follows:
    *
    * <pre>
    * linkedVector = scale * originalTuple
    * </pre>
    *
    * where the scale is obtained from the given {@code scaleSupplier}.
    *
    * @param scaleSupplier the supplier to get the scale.
    * @param originalTuple the reference tuple to scale. Not modified.
    * @return the new vector linked to {@code originalTuple}.
    */
   public static Vector3DReadOnly newLinkedVector3DReadOnly(DoubleSupplier scaleSupplier, Tuple3DReadOnly originalTuple)
   {
      DoubleSupplier xSupplier = () -> scaleSupplier.getAsDouble() * originalTuple.getX();
      DoubleSupplier ySupplier = () -> scaleSupplier.getAsDouble() * originalTuple.getY();
      DoubleSupplier zSupplier = () -> scaleSupplier.getAsDouble() * originalTuple.getZ();
      return newLinkedVector3DReadOnly(xSupplier, ySupplier, zSupplier);
   }

   /**
    * Creates a new point 2D that is a read-only view of the coordinate suppliers.
    *
    * @param xSupplier the x-coordinate supplier.
    * @param ySupplier the y-coordinate supplier.
    * @return the new read-only point 2D.
    */
   public static Point2DReadOnly newLinkedPoint2DReadOnly(DoubleSupplier xSupplier, DoubleSupplier ySupplier)
   {
      return new LinkedPoint2DReadOnly(ySupplier, xSupplier);
   }

   private static final class LinkedPoint2DReadOnly implements Point2DReadOnly
   {
      private final DoubleSupplier ySupplier;
      private final DoubleSupplier xSupplier;

      private LinkedPoint2DReadOnly(DoubleSupplier ySupplier, DoubleSupplier xSupplier)
      {
         this.ySupplier = ySupplier;
         this.xSupplier = xSupplier;
      }

      @Override
      public double getX()
      {
         return xSupplier.getAsDouble();
      }

      @Override
      public double getY()
      {
         return ySupplier.getAsDouble();
      }

      @Override
      public int hashCode()
      {
         return EuclidHashCodeTools.toIntHashCode(getX(), getY());
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof Point2DReadOnly)
            return equals((Point2DReadOnly) object);
         else
            return false;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }
   }

   /**
    * Creates a new vector 2D that is a read-only view of the component suppliers.
    *
    * @param xSupplier the x-component supplier.
    * @param ySupplier the y-component supplier.
    * @return the new read-only vector 2D.
    */
   public static Vector2DReadOnly newLinkedVector2DReadOnly(DoubleSupplier xSupplier, DoubleSupplier ySupplier)
   {
      return new LinkedVector2DReadOnly(xSupplier, ySupplier);
   }

   private static final class LinkedVector2DReadOnly implements Vector2DReadOnly
   {
      private final DoubleSupplier xSupplier;
      private final DoubleSupplier ySupplier;

      private LinkedVector2DReadOnly(DoubleSupplier xSupplier, DoubleSupplier ySupplier)
      {
         this.xSupplier = xSupplier;
         this.ySupplier = ySupplier;
      }

      @Override
      public double getX()
      {
         return xSupplier.getAsDouble();
      }

      @Override
      public double getY()
      {
         return ySupplier.getAsDouble();
      }

      @Override
      public int hashCode()
      {
         return EuclidHashCodeTools.toIntHashCode(getX(), getY());
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof Vector2DReadOnly)
            return equals((Vector2DReadOnly) object);
         else
            return false;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }
   }

   /**
    * Creates a new point 3D that is a read-only view of the coordinate suppliers.
    *
    * @param xSupplier the x-coordinate supplier.
    * @param ySupplier the y-coordinate supplier.
    * @param zSupplier the z-coordinate supplier.
    * @return the new read-only vector 3D.
    */
   public static Point3DReadOnly newLinkedPoint3DReadOnly(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier)
   {
      return new LinkedPoint3DReadOnly(xSupplier, ySupplier, zSupplier);
   }

   private static final class LinkedPoint3DReadOnly implements Point3DReadOnly
   {
      private final DoubleSupplier xSupplier;
      private final DoubleSupplier ySupplier;
      private final DoubleSupplier zSupplier;

      private LinkedPoint3DReadOnly(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier)
      {
         this.xSupplier = xSupplier;
         this.ySupplier = ySupplier;
         this.zSupplier = zSupplier;
      }

      @Override
      public double getX()
      {
         return xSupplier.getAsDouble();
      }

      @Override
      public double getY()
      {
         return ySupplier.getAsDouble();
      }

      @Override
      public double getZ()
      {
         return zSupplier.getAsDouble();
      }

      @Override
      public int hashCode()
      {
         return EuclidHashCodeTools.toIntHashCode(getX(), getY(), getZ());
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof Point3DReadOnly)
            return equals((Point3DReadOnly) object);
         else
            return false;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }
   }

   /**
    * Creates a new vector 3D that is a read-only view of the component suppliers.
    *
    * @param xSupplier the x-component supplier.
    * @param ySupplier the y-component supplier.
    * @param zSupplier the z-component supplier.
    * @return the new read-only point 3D.
    */
   public static Vector3DReadOnly newLinkedVector3DReadOnly(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier)
   {
      return new LinkedVector3DReadOnly(ySupplier, xSupplier, zSupplier);
   }

   private static final class LinkedVector3DReadOnly implements Vector3DReadOnly
   {
      private final DoubleSupplier ySupplier;
      private final DoubleSupplier xSupplier;
      private final DoubleSupplier zSupplier;

      private LinkedVector3DReadOnly(DoubleSupplier ySupplier, DoubleSupplier xSupplier, DoubleSupplier zSupplier)
      {
         this.ySupplier = ySupplier;
         this.xSupplier = xSupplier;
         this.zSupplier = zSupplier;
      }

      @Override
      public double getX()
      {
         return xSupplier.getAsDouble();
      }

      @Override
      public double getY()
      {
         return ySupplier.getAsDouble();
      }

      @Override
      public double getZ()
      {
         return zSupplier.getAsDouble();
      }

      @Override
      public int hashCode()
      {
         return EuclidHashCodeTools.toIntHashCode(getX(), getY(), getZ());
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof Vector3DReadOnly)
            return equals((Vector3DReadOnly) object);
         else
            return false;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }
   }

   /**
    * Creates a new point 2D that is a read-only view of the given {@code originalPoint} negated.
    *
    * @param originalPoint the original point to create linked negative point for. Not modified.
    * @return the negative read-only view of {@code originalPoint}.
    */
   public static Point2DReadOnly newNegativeLinkedPoint2D(Point2DReadOnly originalPoint)
   {
      DoubleSupplier xSupplier = () -> -originalPoint.getX();
      DoubleSupplier ySupplier = () -> -originalPoint.getY();
      return newLinkedPoint2DReadOnly(xSupplier, ySupplier);
   }

   /**
    * Creates a new vector 2D that is a read-only view of the given {@code originalVector} negated.
    *
    * @param originalVector the original vector to create linked negative vector for. Not modified.
    * @return the negative read-only view of {@code originalVector}.
    */
   public static Vector2DReadOnly newNegativeLinkedVector2D(Vector2DReadOnly originalVector)
   {
      DoubleSupplier xSupplier = () -> -originalVector.getX();
      DoubleSupplier ySupplier = () -> -originalVector.getY();
      return newLinkedVector2DReadOnly(xSupplier, ySupplier);
   }

   /**
    * Creates a new point 3D that is a read-only view of the given {@code originalPoint} negated.
    *
    * @param originalPoint the original point to create linked negative point for. Not modified.
    * @return the negative read-only view of {@code originalPoint}.
    */
   public static Point3DReadOnly newNegativeLinkedPoint3D(Point3DReadOnly originalPoint)
   {
      DoubleSupplier xSupplier = () -> -originalPoint.getX();
      DoubleSupplier ySupplier = () -> -originalPoint.getY();
      DoubleSupplier zSupplier = () -> -originalPoint.getZ();
      return newLinkedPoint3DReadOnly(xSupplier, ySupplier, zSupplier);
   }

   /**
    * Creates a new vector 3D that is a read-only view of the given {@code originalVector} negated.
    *
    * @param originalVector the original vector to create linked negative vector for. Not modified.
    * @return the negative read-only view of {@code originalVector}.
    */
   public static Vector3DReadOnly newNegativeLinkedVector3D(Vector3DReadOnly originalVector)
   {
      DoubleSupplier xSupplier = () -> -originalVector.getX();
      DoubleSupplier ySupplier = () -> -originalVector.getY();
      DoubleSupplier zSupplier = () -> -originalVector.getZ();
      return newLinkedVector3DReadOnly(xSupplier, ySupplier, zSupplier);
   }

   /**
    * Creates a new unit vector 2D that is a read-only view of the given {@code originalUnitVector}
    * negated.
    *
    * @param originalUnitVector the original vector to create linked negative vector for. Not modified.
    * @return the negative read-only view of {@code originalUnitVector}.
    */
   public static UnitVector2DReadOnly newNegativeLinkedUnitVector2D(UnitVector2DReadOnly originalUnitVector)
   {
      return new NegativeLinkedUnitVector2DReadOnly(originalUnitVector);
   }

   private static final class NegativeLinkedUnitVector2DReadOnly implements UnitVector2DReadOnly
   {
      private final UnitVector2DReadOnly originalUnitVector;

      private NegativeLinkedUnitVector2DReadOnly(UnitVector2DReadOnly originalUnitVector)
      {
         this.originalUnitVector = originalUnitVector;
      }

      @Override
      public boolean isDirty()
      {
         return originalUnitVector.isDirty();
      }

      @Override
      public double getX()
      {
         return -originalUnitVector.getX();
      }

      @Override
      public double getY()
      {
         return -originalUnitVector.getY();
      }

      @Override
      public double getRawX()
      {
         return -originalUnitVector.getRawX();
      }

      @Override
      public double getRawY()
      {
         return -originalUnitVector.getRawY();
      }

      @Override
      public int hashCode()
      {
         return EuclidHashCodeTools.toIntHashCode(getX(), getY());
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof Vector2DReadOnly)
            return equals((Vector2DReadOnly) object);
         else
            return false;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }
   }

   /**
    * Creates a new unit vector 3D that is a read-only view of the given {@code originalUnitVector}
    * negated.
    *
    * @param originalUnitVector the original vector to create linked negative vector for. Not modified.
    * @return the negative read-only view of {@code originalUnitVector}.
    */
   public static UnitVector3DReadOnly newNegativeLinkedUnitVector3D(UnitVector3DReadOnly originalUnitVector)
   {
      return new NegativeLinkedUnitVector3DReadOnly(originalUnitVector);
   }

   private static final class NegativeLinkedUnitVector3DReadOnly implements UnitVector3DReadOnly
   {
      private final UnitVector3DReadOnly originalUnitVector;

      private NegativeLinkedUnitVector3DReadOnly(UnitVector3DReadOnly originalUnitVector)
      {
         this.originalUnitVector = originalUnitVector;
      }

      @Override
      public boolean isDirty()
      {
         return originalUnitVector.isDirty();
      }

      @Override
      public double getX()
      {
         return -originalUnitVector.getX();
      }

      @Override
      public double getY()
      {
         return -originalUnitVector.getY();
      }

      @Override
      public double getZ()
      {
         return -originalUnitVector.getZ();
      }

      @Override
      public double getRawX()
      {
         return -originalUnitVector.getRawX();
      }

      @Override
      public double getRawY()
      {
         return -originalUnitVector.getRawY();
      }

      @Override
      public double getRawZ()
      {
         return -originalUnitVector.getRawZ();
      }

      @Override
      public int hashCode()
      {
         return EuclidHashCodeTools.toIntHashCode(getX(), getY(), getZ());
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof Vector3DReadOnly)
            return equals((Vector3DReadOnly) object);
         else
            return false;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }
   }

   /**
    * Creates a new unit vector 4D that is a read-only view of the given {@code originalUnitVector}
    * negated.
    *
    * @param originalUnitVector the original vector to create linked negative vector for. Not modified.
    * @return the negative read-only view of {@code originalUnitVector}.
    */
   public static UnitVector4DReadOnly newNegativeLinkedUnitVector4D(UnitVector4DReadOnly originalUnitVector)
   {
      return new NegativeLinkedUnitVector4DReadOnly(originalUnitVector);
   }

   private static final class NegativeLinkedUnitVector4DReadOnly implements UnitVector4DReadOnly
   {
      private final UnitVector4DReadOnly originalUnitVector;

      private NegativeLinkedUnitVector4DReadOnly(UnitVector4DReadOnly originalUnitVector)
      {
         this.originalUnitVector = originalUnitVector;
      }

      @Override
      public boolean isDirty()
      {
         return originalUnitVector.isDirty();
      }

      @Override
      public double getX()
      {
         return -originalUnitVector.getX();
      }

      @Override
      public double getY()
      {
         return -originalUnitVector.getY();
      }

      @Override
      public double getZ()
      {
         return -originalUnitVector.getZ();
      }

      @Override
      public double getS()
      {
         return -originalUnitVector.getS();
      }

      @Override
      public double getRawX()
      {
         return -originalUnitVector.getRawX();
      }

      @Override
      public double getRawY()
      {
         return -originalUnitVector.getRawY();
      }

      @Override
      public double getRawZ()
      {
         return -originalUnitVector.getRawZ();
      }

      @Override
      public double getRawS()
      {
         return -originalUnitVector.getRawS();
      }

      @Override
      public int hashCode()
      {
         return EuclidHashCodeTools.toIntHashCode(getX(), getY(), getZ(), getS());
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof Vector4DReadOnly)
            return equals((Vector4DReadOnly) object);
         else
            return false;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }
   }

   /**
    * Creates a new quaternion that is a read-only view of the given {@code original} conjugated.
    *
    * @param originalQuaternion the original quaternion to create the linked conjugated quaternion for.
    *                           Not modified.
    * @return the conjugated read-only view of {@code original}.
    */
   public static QuaternionReadOnly newConjugateLinkedQuaternion(QuaternionReadOnly originalQuaternion)
   {
      return new ConjugateLinkedQuaternionReadOnly(originalQuaternion);
   }

   private static final class ConjugateLinkedQuaternionReadOnly implements QuaternionReadOnly
   {
      private final QuaternionReadOnly originalQuaternion;

      private ConjugateLinkedQuaternionReadOnly(QuaternionReadOnly originalQuaternion)
      {
         this.originalQuaternion = originalQuaternion;
      }

      @Override
      public double getX()
      {
         return -originalQuaternion.getX();
      }

      @Override
      public double getY()
      {
         return -originalQuaternion.getY();
      }

      @Override
      public double getZ()
      {
         return -originalQuaternion.getZ();
      }

      @Override
      public double getS()
      {
         return originalQuaternion.getS();
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
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }
   }

   /**
    * Creates a new matrix 3D that is a read-only view of the transpose of the given {@code original}.
    *
    * @param original the original matrix to create linked transpose matrix for. Not modified.
    * @return the transpose read-only view of {@code original}.
    */
   public static Matrix3DReadOnly newTransposeLinkedMatrix3DReadOnly(Matrix3DReadOnly original)
   {
      return new TransposeLinkedMatrix3DReadOnly(original);
   }

   private static final class TransposeLinkedMatrix3DReadOnly implements Matrix3DReadOnly
   {
      private final Matrix3DReadOnly original;

      private TransposeLinkedMatrix3DReadOnly(Matrix3DReadOnly original)
      {
         this.original = original;
      }

      @Override
      public double getM00()
      {
         return original.getM00();
      }

      @Override
      public double getM01()
      {
         return original.getM10();
      }

      @Override
      public double getM02()
      {
         return original.getM20();
      }

      @Override
      public double getM10()
      {
         return original.getM01();
      }

      @Override
      public double getM11()
      {
         return original.getM11();
      }

      @Override
      public double getM12()
      {
         return original.getM21();
      }

      @Override
      public double getM20()
      {
         return original.getM02();
      }

      @Override
      public double getM21()
      {
         return original.getM12();
      }

      @Override
      public double getM22()
      {
         return original.getM22();
      }

      @Override
      public int hashCode()
      {
         return EuclidHashCodeTools.toIntHashCode(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22());
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof Matrix3DReadOnly)
            return equals((Matrix3DReadOnly) object);
         else
            return false;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }
   }

   /**
    * Creates a new matrix 3D that is a read-only view of the tilde form of the given
    * {@code originalTuple}.
    * <p>
    * The tilde form is the matrix implementation of cross product:
    *
    * <pre>
    *               /  0 -z  y \
    * tildeMatrix = |  z  0 -x |
    *               \ -y  x  0 /
    * </pre>
    *
    * where <tt>x</tt>, <tt>y</tt>, and <tt>z</tt> are the components of {@code originalTuple}.
    * </p>
    *
    * @param originalTuple the original tuple to create linked tilde matrix for. Not modified.
    * @return the tilde read-only view of {@code originalTuple}.
    */
   public static Matrix3DReadOnly newTildeLinkedMatrix3DReadOnly(Tuple3DReadOnly originalTuple)
   {
      return new TildeLinkedMatrix3DReadOnly(originalTuple);
   }

   private static final class TildeLinkedMatrix3DReadOnly implements Matrix3DReadOnly
   {
      private final Tuple3DReadOnly originalTuple;

      private TildeLinkedMatrix3DReadOnly(Tuple3DReadOnly originalTuple)
      {
         this.originalTuple = originalTuple;
      }

      @Override
      public double getM00()
      {
         return 0.0;
      }

      @Override
      public double getM01()
      {
         return -originalTuple.getZ();
      }

      @Override
      public double getM02()
      {
         return originalTuple.getY();
      }

      @Override
      public double getM10()
      {
         return originalTuple.getZ();
      }

      @Override
      public double getM11()
      {
         return 0.0;
      }

      @Override
      public double getM12()
      {
         return -originalTuple.getX();
      }

      @Override
      public double getM20()
      {
         return -originalTuple.getY();
      }

      @Override
      public double getM21()
      {
         return originalTuple.getX();
      }

      @Override
      public double getM22()
      {
         return 0.0;
      }

      @Override
      public int hashCode()
      {
         return EuclidHashCodeTools.toIntHashCode(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22());
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof Matrix3DReadOnly)
            return equals((Matrix3DReadOnly) object);
         else
            return false;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }
   }

   /**
    * Creates a new matrix 3D that is a read-only view of the diagonal form of the given
    * {@code originalTuple}:
    *
    * <pre>
    *                  / x  0  0 \
    * diagonalMatrix = | 0  y  0 |
    *                  \ 0  0  z /
    * </pre>
    *
    * where <tt>x</tt>, <tt>y</tt>, and <tt>z</tt> are the components of {@code originalTuple}.
    *
    * @param originalTuple the original tuple to create linked diagonal matrix for. Not modified.
    * @return the diagonal read-only view of {@code originalTuple}.
    */
   public static Matrix3DReadOnly newDiagonalLinkedMatrix3DReadOnly(Tuple3DReadOnly originalTuple)
   {
      return new DiagonalLinkedMatrix3DReadOnly(originalTuple);
   }

   private static final class DiagonalLinkedMatrix3DReadOnly implements Matrix3DReadOnly
   {
      private final Tuple3DReadOnly originalTuple;

      private DiagonalLinkedMatrix3DReadOnly(Tuple3DReadOnly originalTuple)
      {
         this.originalTuple = originalTuple;
      }

      @Override
      public double getM00()
      {
         return originalTuple.getX();
      }

      @Override
      public double getM01()
      {
         return 0.0;
      }

      @Override
      public double getM02()
      {
         return 0.0;
      }

      @Override
      public double getM10()
      {
         return 0.0;
      }

      @Override
      public double getM11()
      {
         return originalTuple.getY();
      }

      @Override
      public double getM12()
      {
         return 0.0;
      }

      @Override
      public double getM20()
      {
         return 0.0;
      }

      @Override
      public double getM21()
      {
         return 0.0;
      }

      @Override
      public double getM22()
      {
         return originalTuple.getZ();
      }

      @Override
      public int hashCode()
      {
         return EuclidHashCodeTools.toIntHashCode(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22());
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof Matrix3DReadOnly)
            return equals((Matrix3DReadOnly) object);
         else
            return false;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }
   }

   /**
    * Creates a linked point that can be used to observe access to the source point's coordinates.
    *
    * @param accessListener the listener to be notified whenever a coordinate of the point is
    *                              being accessed. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate being accessed.
    * @param source                the original point to link and observe. Not modified.
    * @return the observable point.
    */
   public static Point2DReadOnly newObservablePoint2DReadOnly(EuclidAccessListener<Axis2D> accessListener, Point2DReadOnly source)
   {
      return new ObservablePoint2DReadOnly(accessListener, source);
   }

   private static final class ObservablePoint2DReadOnly extends ObservableEuclidGeometry<Axis2D, Point2DReadOnly> implements Point2DReadOnly
   {
      private ObservablePoint2DReadOnly(EuclidAccessListener<Axis2D> accessListener, Point2DReadOnly source)
      {
         super(accessListener, null, source);
      }

      @Override
      public double getX()
      {
         notifyAccess(Axis2D.X);
         return source.getX();
      }

      @Override
      public double getY()
      {
         notifyAccess(Axis2D.Y);
         return source.getY();
      }
   }

   /**
    * Creates a linked point that can be used to observe access to the source point's coordinates.
    *
    * @param accessListener the listener to be notified whenever a coordinate of the point is being
    *                       accessed. The corresponding constant {@link Axis3D} will be passed to
    *                       indicate the coordinate being accessed.
    * @param source         the original point to link and observe. Not modified.
    * @return the observable point.
    */
   public static Point3DReadOnly newObservablePoint3DReadOnly(EuclidAccessListener<Axis3D> accessListener, Point3DReadOnly source)
   {
      return new ObservablePoint3DReadOnly(accessListener, source);
   }

   private static final class ObservablePoint3DReadOnly extends ObservableEuclidGeometry<Axis3D, Point3DReadOnly> implements Point3DReadOnly
   {
      private ObservablePoint3DReadOnly(EuclidAccessListener<Axis3D> accessListener, Point3DReadOnly source)
      {
         super(accessListener, null, source);
      }

      @Override
      public double getX()
      {
         notifyAccess(Axis3D.X);
         return source.getX();
      }

      @Override
      public double getY()
      {
         notifyAccess(Axis3D.Y);
         return source.getY();
      }

      @Override
      public double getZ()
      {
         notifyAccess(Axis3D.Z);
         return source.getZ();
      }
   }

   /**
    * Creates a linked vector that can be used to observe access to the source vector's components.
    *
    * @param accessListener the listener to be notified whenever a component of the vector is being
    *                       accessed. The corresponding constant {@link Axis2D} will be passed to
    *                       indicate the component being accessed.
    * @param source         the original vector to link and observe. Not modified.
    * @return the observable vector.
    */
   public static Vector2DReadOnly newObservableVector2DReadOnly(EuclidAccessListener<Axis2D> accessListener, Vector2DReadOnly source)
   {
      return new ObservableVector2DReadOnly(accessListener, source);
   }

   private static final class ObservableVector2DReadOnly extends ObservableEuclidGeometry<Axis2D, Vector2DReadOnly> implements Vector2DReadOnly
   {
      private ObservableVector2DReadOnly(EuclidAccessListener<Axis2D> accessListener, Vector2DReadOnly source)
      {
         super(accessListener, null, source);
      }

      @Override
      public double getX()
      {
         notifyAccess(Axis2D.X);
         return source.getX();
      }

      @Override
      public double getY()
      {
         notifyAccess(Axis2D.Y);
         return source.getY();
      }
   }

   /**
    * Creates a linked vector that can be used to observe access to the source vector's components.
    *
    * @param accessListener the listener to be notified whenever a component of the vector is being
    *                       accessed. The corresponding constant {@link Axis3D} will be passed to
    *                       indicate the component being accessed.
    * @param source         the original vector to link and observe. Not modified.
    * @return the observable vector.
    */
   public static Vector3DReadOnly newObservableVector3DReadOnly(EuclidAccessListener<Axis3D> accessListener, Vector3DReadOnly source)
   {
      return new ObservableVector3DReadOnly(accessListener, source);
   }

   private static final class ObservableVector3DReadOnly extends ObservableEuclidGeometry<Axis3D, Vector3DReadOnly> implements Vector3DReadOnly
   {
      private ObservableVector3DReadOnly(EuclidAccessListener<Axis3D> accessListener, Vector3DReadOnly source)
      {
         super(accessListener, null, source);
      }

      @Override
      public double getX()
      {
         notifyAccess(Axis3D.X);
         return source.getX();
      }

      @Override
      public double getY()
      {
         notifyAccess(Axis3D.Y);
         return source.getY();
      }

      @Override
      public double getZ()
      {
         notifyAccess(Axis3D.Z);
         return source.getZ();
      }
   }

   /**
    * Creates a linked unit vector that can be used to observe access to the source unit vector's
    * components.
    *
    * @param accessListener the listener to be notified whenever a component of the vector is being
    *                       accessed. The corresponding constant {@link Axis2D} will be passed to
    *                       indicate the component being accessed.
    * @param source         the original unit vector to link and observe. Not modified.
    * @return the observable unit vector.
    */
   public static UnitVector2DReadOnly newObservableUnitVector2DReadOnly(EuclidAccessListener<Axis2D> accessListener, UnitVector2DReadOnly source)
   {
      return new ObservableUnitVector2DReadOnly(accessListener, source);
   }

   private static final class ObservableUnitVector2DReadOnly extends ObservableEuclidGeometry<Axis2D, UnitVector2DReadOnly> implements UnitVector2DReadOnly
   {
      private ObservableUnitVector2DReadOnly(EuclidAccessListener<Axis2D> accessListener, UnitVector2DReadOnly source)
      {
         super(accessListener, null, source);
      }

      @Override
      public boolean isDirty()
      {
         return source.isDirty();
      }

      @Override
      public double getX()
      {
         notifyAccess(Axis2D.X);
         return source.getX();
      }

      @Override
      public double getY()
      {
         notifyAccess(Axis2D.Y);
         return source.getY();
      }

      @Override
      public double getRawX()
      {
         return source.getRawX();
      }

      @Override
      public double getRawY()
      {
         return source.getRawY();
      }
   }

   /**
    * Creates a linked unit vector that can be used to observe access to the source unit vector's
    * components.
    *
    * @param accessListener the listener to be notified whenever a component of the vector is being
    *                       accessed. The corresponding constant {@link Axis3D} will be passed to
    *                       indicate the component being accessed.
    * @param source         the original unit vector to link and observe. Not modified.
    * @return the observable unit vector.
    */
   public static UnitVector3DReadOnly newObservableUnitVector3DReadOnly(EuclidAccessListener<Axis3D> accessListener, UnitVector3DReadOnly source)
   {
      return new ObservableUnitVector3DReadOnly(accessListener, source);
   }

   private static final class ObservableUnitVector3DReadOnly extends ObservableEuclidGeometry<Axis3D, UnitVector3DReadOnly> implements UnitVector3DReadOnly
   {
      private ObservableUnitVector3DReadOnly(EuclidAccessListener<Axis3D> accessListener, UnitVector3DReadOnly source)
      {
         super(accessListener, null, source);
      }

      @Override
      public boolean isDirty()
      {
         return source.isDirty();
      }

      @Override
      public double getX()
      {
         notifyAccess(Axis3D.X);
         return source.getX();
      }

      @Override
      public double getY()
      {
         notifyAccess(Axis3D.Y);
         return source.getY();
      }

      @Override
      public double getZ()
      {
         notifyAccess(Axis3D.Z);
         return source.getY();
      }

      @Override
      public double getRawX()
      {
         return source.getRawX();
      }

      @Override
      public double getRawY()
      {
         return source.getRawY();
      }

      @Override
      public double getRawZ()
      {
         return source.getRawZ();
      }
   }

   /**
    * Creates a linked rotation matrix that can be used to observe access to the source rotation
    * matrix's components. TODO fix the doc
    * 
    * @param accessListener the listener to be notified whenever a component of the rotation matrix is
    *                       being accessed. The corresponding constants {@link Axis3D} will be passed
    *                       to indicate the row and column respectively of the coefficient being
    *                       accessed.
    * @param source         the original rotation matrix to link and observe. Not modified.
    * @return the observable rotation matrix.
    */
   public static RotationMatrixReadOnly newObservableRotationMatrixReadOnly(EuclidAccessListener<Matrix3DElements> accessListener,
                                                                            RotationMatrixReadOnly source)
   {
      return new ObservableRotationMatrixReadOnly(accessListener, source);
   }

   private static final class ObservableRotationMatrixReadOnly extends ObservableEuclidGeometry<Matrix3DElements, RotationMatrixReadOnly>
         implements RotationMatrixReadOnly
   {
      private ObservableRotationMatrixReadOnly(EuclidAccessListener<Matrix3DElements> accessListener, RotationMatrixReadOnly source)
      {
         super(accessListener, null, source);
      }

      @Override
      public boolean isDirty()
      {
         return source.isDirty();
      }

      @Override
      public double getM00()
      {
         notifyAccess(Matrix3DElements.M00);
         return source.getM00();
      }

      @Override
      public double getM01()
      {
         notifyAccess(Matrix3DElements.M01);
         return source.getM01();
      }

      @Override
      public double getM02()
      {
         notifyAccess(Matrix3DElements.M02);
         return source.getM02();
      }

      @Override
      public double getM10()
      {
         notifyAccess(Matrix3DElements.M10);
         return source.getM10();
      }

      @Override
      public double getM11()
      {
         notifyAccess(Matrix3DElements.M11);
         return source.getM11();
      }

      @Override
      public double getM12()
      {
         notifyAccess(Matrix3DElements.M12);
         return source.getM12();
      }

      @Override
      public double getM20()
      {
         notifyAccess(Matrix3DElements.M20);
         return source.getM20();
      }

      @Override
      public double getM21()
      {
         notifyAccess(Matrix3DElements.M21);
         return source.getM21();
      }

      @Override
      public double getM22()
      {
         notifyAccess(Matrix3DElements.M22);
         return source.getM22();
      }
   }

   /**
    * Creates a linked quaternion that can be used to observe access to the source rotation matrix's
    * components. TODO Update the doc
    * 
    * @param accessListener the listener to be notified whenever a component of the quaternion is being
    *                       accessed. The index of the component being accessed will be passed.
    * @param source         the original quaternion to link and observe. Not modified.
    * @return the observable quaternion.
    */
   public static QuaternionReadOnly newObservableQuaternionReadOnly(EuclidAccessListener<Axis4D> accessListener, QuaternionReadOnly source)
   {
      return new ObservableQuaternionReadOnly(accessListener, source);
   }

   private static final class ObservableQuaternionReadOnly extends ObservableEuclidGeometry<Axis4D, QuaternionReadOnly> implements QuaternionReadOnly
   {
      private ObservableQuaternionReadOnly(EuclidAccessListener<Axis4D> accessListener, QuaternionReadOnly source)
      {
         super(accessListener, null, source);
      }

      @Override
      public double getX()
      {
         notifyAccess(Axis4D.X);
         return source.getX();
      }

      @Override
      public double getY()
      {
         notifyAccess(Axis4D.Y);
         return source.getY();
      }

      @Override
      public double getZ()
      {
         notifyAccess(Axis4D.Z);
         return source.getZ();
      }

      @Override
      public double getS()
      {
         notifyAccess(Axis4D.S);
         return source.getS();
      }
   }

   /**
    * Creates a new point that can be used to observe read and write operations.
    * 
    * @param accessListener the listener to be notified whenever a coordinate of the point is being
    *                       accessed. The corresponding constant {@link Axis2D} will be passed to
    *                       indicate the coordinate being accessed. Can be {@code null}.
    * @param changeListener the listener to be notified whenever a coordinate of the point has been
    *                       modified. The corresponding constant {@link Axis2D} will be passed to
    *                       indicate the coordinate that was changed alongside its new value. Can be
    *                       {@code null}.
    * @return the observable point.
    */
   public static Point2DBasics newObservablePoint2DBasics(EuclidAccessListener<Axis2D> accessListener, EuclidChangeListener<Axis2D> changeListener)
   {
      return newObservablePoint2DBasics(accessListener, changeListener, new Point2D());
   }

   /**
    * Creates a linked point that can be used to observe read and write operations on the source.
    * 
    * @param changeListener        the listener to be notified whenever a coordinate of the point has
    *                              been modified. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param source                the original point to link and observe. Modifiable via the linked
    *                              point interface.
    * @param accessListener the listener to be notified whenever a coordinate of the point is
    *                              being accessed. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate being accessed. Can be
    *                              {@code null}.
    * @return the observable point.
    */
   public static Point2DBasics newObservablePoint2DBasics(EuclidAccessListener<Axis2D> accessListener,
                                                          EuclidChangeListener<Axis2D> changeListener,
                                                          Point2DBasics source)
   {
      return new ObservablePoint2DBasics(accessListener, changeListener, source);
   }

   private static final class ObservablePoint2DBasics extends ObservableEuclidGeometry<Axis2D, Point2DBasics> implements Point2DBasics
   {
      private ObservablePoint2DBasics(EuclidAccessListener<Axis2D> accessListener, EuclidChangeListener<Axis2D> changeListener, Point2DBasics source)
      {
         super(accessListener, changeListener, source);
      }

      @Override
      public void setX(double x)
      {
         double oldX = source.getX();

         if (x != oldX)
         {
            source.setX(x);
            notifyChange(Axis2D.X);
         }
      }

      @Override
      public void setY(double y)
      {
         double oldY = source.getY();

         if (y != oldY)
         {
            source.setY(y);
            notifyChange(Axis2D.Y);
         }
      }

      @Override
      public double getX()
      {
         notifyAccess(Axis2D.X);
         return source.getX();
      }

      @Override
      public double getY()
      {
         notifyAccess(Axis2D.Y);
         return source.getY();
      }
   }

   /**
    * Creates a new point that can be used to observe read and write operations.
    * 
    * @param accessListener the listener to be notified whenever a coordinate of the point is being
    *                       accessed. The corresponding constant {@link Axis3D} will be passed to
    *                       indicate the coordinate being accessed. Can be {@code null}.
    * @param changeListener the listener to be notified whenever a coordinate of the point has been
    *                       modified. The corresponding constant {@link Axis3D} will be passed to
    *                       indicate the coordinate that was changed alongside its new value. Can be
    *                       {@code null}.
    * @return the observable point.
    */
   public static Point3DBasics newObservablePoint3DBasics(EuclidAccessListener<Axis3D> accessListener, EuclidChangeListener<Axis3D> changeListener)
   {
      return newObservablePoint3DBasics(accessListener, changeListener, new Point3D());
   }

   /**
    * Creates a linked point that can be used to observe read and write operations on the source.
    * 
    * @param accessListener       the listener to be notified whenever a coordinate of the point is
    *                             being accessed. The corresponding constant {@link Axis3D} will be
    *                             passed to indicate the coordinate being accessed. Can be
    *                             {@code null}.
    * @param source               the original point to link and observe. Modifiable via the linked
    *                             point interface.
    * @param valueChangedListener the listener to be notified whenever a coordinate of the point has
    *                             been modified. The corresponding constant {@link Axis3D} will be
    *                             passed to indicate the coordinate that was changed alongside its new
    *                             value. Can be {@code null}.
    * @return the observable point.
    */
   public static Point3DBasics newObservablePoint3DBasics(EuclidAccessListener<Axis3D> accessListener,
                                                          EuclidChangeListener<Axis3D> changeListener,
                                                          Point3DBasics source)
   {
      return new ObservablePoint3DBasics(accessListener, changeListener, source);
   }

   private static final class ObservablePoint3DBasics extends ObservableEuclidGeometry<Axis3D, Point3DBasics> implements Point3DBasics
   {
      private ObservablePoint3DBasics(EuclidAccessListener<Axis3D> accessListener, EuclidChangeListener<Axis3D> changeListener, Point3DBasics source)
      {
         super(accessListener, changeListener, source);
      }

      @Override
      public void setX(double x)
      {
         double oldX = source.getX();

         if (x != oldX)
         {
            source.setX(x);
            notifyChange(Axis3D.X);
         }
      }

      @Override
      public void setY(double y)
      {
         double oldY = source.getY();

         if (y != oldY)
         {
            source.setY(y);
            notifyChange(Axis3D.Y);
         }
      }

      @Override
      public void setZ(double z)
      {
         double oldZ = source.getZ();

         if (z != oldZ)
         {
            source.setZ(z);
            notifyChange(Axis3D.Z);
         }
      }

      @Override
      public double getX()
      {
         notifyAccess(Axis3D.X);
         return source.getX();
      }

      @Override
      public double getY()
      {
         notifyAccess(Axis3D.Y);
         return source.getY();
      }

      @Override
      public double getZ()
      {
         notifyAccess(Axis3D.Z);
         return source.getZ();
      }
   }

   /**
    * Creates a new vector that can be used to observe read and write operations.
    * 
    * @param accessListener the listener to be notified whenever a component of the vector is being
    *                       accessed. The corresponding constant {@link Axis2D} will be passed to
    *                       indicate the component being accessed. Can be {@code null}.
    * @param changeListener the listener to be notified whenever a component of the vector has been
    *                       modified. The corresponding constant {@link Axis2D} will be passed to
    *                       indicate the component that was changed alongside its new value. Can be
    *                       {@code null}.
    * @return the observable vector.
    */
   public static Vector2DBasics newObservableVector2DBasics(EuclidAccessListener<Axis2D> accessListener, EuclidChangeListener<Axis2D> changeListener)
   {
      return newObservableVector2DBasics(accessListener, changeListener, new Vector2D());
   }

   /**
    * Creates a linked vector that can be used to observe read and write operations on the source.
    * 
    * @param accessListener the listener to be notified whenever a coordinate of the vector is being
    *                       accessed. The corresponding constant {@link Axis2D} will be passed to
    *                       indicate the coordinate being accessed. Can be {@code null}.
    * @param changeListener the listener to be notified whenever a coordinate of the vector has been
    *                       modified. The corresponding constant {@link Axis2D} will be passed to
    *                       indicate the coordinate that was changed alongside its new value. Can be
    *                       {@code null}.
    * @param source         the original vector to link and observe. Modifiable via the linked vector
    *                       interface.
    * @return the observable vector.
    */
   public static Vector2DBasics newObservableVector2DBasics(EuclidAccessListener<Axis2D> accessListener,
                                                            EuclidChangeListener<Axis2D> changeListener,
                                                            Vector2DBasics source)
   {
      return new ObservableVector2DBasics(accessListener, changeListener, source);
   }

   private static final class ObservableVector2DBasics extends ObservableEuclidGeometry<Axis2D, Vector2DBasics> implements Vector2DBasics
   {
      private ObservableVector2DBasics(EuclidAccessListener<Axis2D> accessListener, EuclidChangeListener<Axis2D> changeListener, Vector2DBasics source)
      {
         super(accessListener, changeListener, source);
      }

      @Override
      public void setX(double x)
      {
         double oldX = source.getX();

         if (x != oldX)
         {
            source.setX(x);
            notifyChange(Axis2D.X);
         }
      }

      @Override
      public void setY(double y)
      {
         double oldY = source.getY();

         if (y != oldY)
         {
            source.setY(y);
            notifyChange(Axis2D.Y);
         }
      }

      @Override
      public double getX()
      {
         notifyAccess(Axis2D.X);
         return source.getX();
      }

      @Override
      public double getY()
      {
         notifyAccess(Axis2D.Y);
         return source.getY();
      }
   }

   /**
    * Creates a new vector that can be used to observe read and write operations.
    * 
    * @param accessListener the listener to be notified whenever a component of the vector is being
    *                       accessed. The corresponding constant {@link Axis3D} will be passed to
    *                       indicate the component being accessed. Can be {@code null}.
    * @param changeListener the listener to be notified whenever a component of the vector has been
    *                       modified. The corresponding constant {@link Axis3D} will be passed to
    *                       indicate the component that was changed alongside its new value. Can be
    *                       {@code null}.
    * @return the observable vector.
    */
   public static Vector3DBasics newObservableVector3DBasics(EuclidAccessListener<Axis3D> accessListener, EuclidChangeListener<Axis3D> changeListener)
   {
      return newObservableVector3DBasics(accessListener, changeListener, new Vector3D());
   }

   /**
    * Creates a linked vector that can be used to observe read and write operations on the source.
    * 
    * @param accessListener the listener to be notified whenever a coordinate of the vector is being
    *                       accessed. The corresponding constant {@link Axis3D} will be passed to
    *                       indicate the coordinate being accessed. Can be {@code null}.
    * @param changeListener the listener to be notified whenever a coordinate of the vector has been
    *                       modified. The corresponding constant {@link Axis3D} will be passed to
    *                       indicate the coordinate that was changed alongside its new value. Can be
    *                       {@code null}.
    * @param source         the original vector to link and observe. Modifiable via the linked vector
    *                       interface.
    * @return the observable vector.
    */
   public static Vector3DBasics newObservableVector3DBasics(EuclidAccessListener<Axis3D> accessListener,
                                                            EuclidChangeListener<Axis3D> changeListener,
                                                            Vector3DBasics source)
   {
      return new ObservableVector3DBasics(accessListener, changeListener, source);
   }

   private static final class ObservableVector3DBasics extends ObservableEuclidGeometry<Axis3D, Vector3DBasics> implements Vector3DBasics
   {
      private ObservableVector3DBasics(EuclidAccessListener<Axis3D> accessListener, EuclidChangeListener<Axis3D> changeListener, Vector3DBasics source)
      {
         super(accessListener, changeListener, source);
      }

      @Override
      public void setX(double x)
      {
         double oldX = source.getX();

         if (x != oldX)
         {
            source.setX(x);
            notifyChange(Axis3D.X);
         }
      }

      @Override
      public void setY(double y)
      {
         double oldY = source.getY();

         if (y != oldY)
         {
            source.setY(y);
            notifyChange(Axis3D.Y);
         }
      }

      @Override
      public void setZ(double z)
      {
         double oldZ = source.getZ();

         if (z != oldZ)
         {
            source.setZ(z);
            notifyChange(Axis3D.Z);
         }
      }

      @Override
      public double getX()
      {
         notifyAccess(Axis3D.X);
         return source.getX();
      }

      @Override
      public double getY()
      {
         notifyAccess(Axis3D.Y);
         return source.getY();
      }

      @Override
      public double getZ()
      {
         notifyAccess(Axis3D.Z);
         return source.getZ();
      }
   }

   /**
    * Creates a new vector that can be used to observe read and write operations.
    * 
    * @param accessListener the listener to be notified whenever a component of the vector is being
    *                       accessed. The corresponding constant {@link Axis2D} will be passed to
    *                       indicate the component being accessed. Can be {@code null}.
    * @param changeListener the listener to be notified whenever a component of the vector has been
    *                       modified. The corresponding constant {@link Axis2D} will be passed to
    *                       indicate the component that was changed alongside its new value. Can be
    *                       {@code null}.
    * @return the observable vector.
    */
   public static UnitVector2DBasics newObservableUnitVector2DBasics(EuclidAccessListener<Axis2D> accessListener, EuclidChangeListener<Axis2D> changeListener)
   {
      return newObservableUnitVector2DBasics(accessListener, changeListener, new UnitVector2D());
   }

   /**
    * Creates a linked vector that can be used to observe read and write operations on the source.
    * 
    * @param accessListener the listener to be notified whenever a coordinate of the vector is being
    *                       accessed. The corresponding constant {@link Axis2D} will be passed to
    *                       indicate the coordinate being accessed. Can be {@code null}.
    * @param changeListener the listener to be notified whenever a coordinate of the vector has been
    *                       modified. The corresponding constant {@link Axis2D} will be passed to
    *                       indicate the coordinate that was changed alongside its new value. Can be
    *                       {@code null}.
    * @param source         the original vector to link and observe. Modifiable via the linked vector
    *                       interface.
    * @return the observable vector.
    */
   public static UnitVector2DBasics newObservableUnitVector2DBasics(EuclidAccessListener<Axis2D> accessListener,
                                                                    EuclidChangeListener<Axis2D> changeListener,
                                                                    UnitVector2DBasics source)
   {
      return new ObservableUnitVector2DBasics(accessListener, changeListener, source);
   }

   private static final class ObservableUnitVector2DBasics extends ObservableEuclidGeometry<Axis2D, UnitVector2DBasics> implements UnitVector2DBasics
   {
      private ObservableUnitVector2DBasics(EuclidAccessListener<Axis2D> accessListener, EuclidChangeListener<Axis2D> changeListener, UnitVector2DBasics source)
      {
         super(accessListener, changeListener, source);
      }

      @Override
      public void absolute()
      {
         boolean xChange = getRawX() < 0.0;
         boolean yChange = getRawY() < 0.0;

         if (xChange || yChange)
         {
            source.absolute();

            if (xChange)
               notifyChange(Axis2D.X);
            if (yChange)
               notifyChange(Axis2D.Y);
         }
      }

      @Override
      public void negate()
      {
         source.negate();

         notifyChange(Axis2D.X);
         notifyChange(Axis2D.Y);
      }

      @Override
      public void markAsDirty()
      {
         source.markAsDirty();
      }

      @Override
      public boolean isDirty()
      {
         return source.isDirty();
      }

      @Override
      public void normalize()
      {
         if (isDirty())
         {
            source.normalize();

            notifyChange(Axis2D.X);
            notifyChange(Axis2D.Y);
         }
      }

      @Override
      public void set(UnitVector2DReadOnly other)
      {
         boolean xChange = getRawX() != other.getRawX();
         boolean yChange = getRawY() != other.getRawY();

         if (xChange || yChange)
         {
            source.set(other);

            if (xChange)
               notifyChange(Axis2D.X);
            if (yChange)
               notifyChange(Axis2D.Y);
         }
      }

      @Override
      public void setX(double x)
      {
         if (x != source.getRawX())
         {
            source.setX(x);
            notifyChange(Axis2D.X);
         }
      }

      @Override
      public void setY(double y)
      {
         if (y != source.getRawY())
         {
            source.setY(y);
            notifyChange(Axis2D.Y);
         }
      }

      @Override
      public double getX()
      {
         notifyAccess(Axis2D.X);
         return source.getX();
      }

      @Override
      public double getY()
      {
         notifyAccess(Axis2D.Y);
         return source.getY();
      }

      @Override
      public double getRawX()
      {
         return source.getRawX();
      }

      @Override
      public double getRawY()
      {
         return source.getRawY();
      }
   }

   /**
    * Creates a new vector that can be used to observe read and write operations.
    * 
    * @param accessListener the listener to be notified whenever a component of the vector is being
    *                       accessed. The corresponding constant {@link Axis3D} will be passed to
    *                       indicate the component being accessed. Can be {@code null}.
    * @param changeListener the listener to be notified whenever a component of the vector has been
    *                       modified. The corresponding constant {@link Axis3D} will be passed to
    *                       indicate the component that was changed alongside its new value. Can be
    *                       {@code null}.
    * @return the observable vector.
    */
   public static UnitVector3DBasics newObservableUnitVector3DBasics(EuclidAccessListener<Axis3D> accessListener, EuclidChangeListener<Axis3D> changeListener)
   {
      return newObservableUnitVector3DBasics(accessListener, changeListener, new UnitVector3D());
   }

   /**
    * Creates a linked vector that can be used to observe read and write operations on the source.
    * 
    * @param accessListener the listener to be notified whenever a coordinate of the vector is being
    *                       accessed. The corresponding constant {@link Axis3D} will be passed to
    *                       indicate the coordinate being accessed. Can be {@code null}.
    * @param changeListener the listener to be notified whenever a coordinate of the vector has been
    *                       modified. The corresponding constant {@link Axis3D} will be passed to
    *                       indicate the coordinate that was changed alongside its new value. Can be
    *                       {@code null}.
    * @param source         the original vector to link and observe. Modifiable via the linked vector
    *                       interface.
    * @return the observable vector.
    */
   public static UnitVector3DBasics newObservableUnitVector3DBasics(EuclidAccessListener<Axis3D> accessListener,
                                                                    EuclidChangeListener<Axis3D> changeListener,
                                                                    UnitVector3DBasics source)
   {
      return new ObservableUnitVector3DBasics(accessListener, changeListener, source);
   }

   private static final class ObservableUnitVector3DBasics extends ObservableEuclidGeometry<Axis3D, UnitVector3DBasics> implements UnitVector3DBasics
   {
      private ObservableUnitVector3DBasics(EuclidAccessListener<Axis3D> accessListener, EuclidChangeListener<Axis3D> changeListener, UnitVector3DBasics source)
      {
         super(accessListener, changeListener, source);
      }

      @Override
      public void absolute()
      {
         boolean xChange = getRawX() < 0.0;
         boolean yChange = getRawY() < 0.0;
         boolean zChange = getRawZ() < 0.0;

         if (xChange || yChange || zChange)
         {
            source.absolute();

            if (xChange)
               notifyChange(Axis3D.X);
            if (yChange)
               notifyChange(Axis3D.Y);
            if (zChange)
               notifyChange(Axis3D.Z);
         }
      }

      @Override
      public void negate()
      {
         source.negate();

         notifyChange(Axis3D.X);
         notifyChange(Axis3D.Y);
         notifyChange(Axis3D.Z);
      }

      @Override
      public void markAsDirty()
      {
         source.markAsDirty();
      }

      @Override
      public boolean isDirty()
      {
         return source.isDirty();
      }

      @Override
      public void normalize()
      {
         if (isDirty())
         {
            source.normalize();

            notifyChange(Axis3D.X);
            notifyChange(Axis3D.Y);
            notifyChange(Axis3D.Z);
         }
      }

      @Override
      public void set(UnitVector3DReadOnly other)
      {
         boolean xChange = getRawX() != other.getRawX();
         boolean yChange = getRawY() != other.getRawY();
         boolean zChange = getRawZ() != other.getRawZ();

         if (xChange || yChange || zChange)
         {
            source.set(other);

            if (xChange)
               notifyChange(Axis3D.X);
            if (yChange)
               notifyChange(Axis3D.Y);
            if (zChange)
               notifyChange(Axis3D.Z);
         }
      }

      @Override
      public void setX(double x)
      {
         if (x != source.getRawX())
         {
            source.setX(x);
            notifyChange(Axis3D.X);
         }
      }

      @Override
      public void setY(double y)
      {
         if (y != source.getRawY())
         {
            source.setY(y);
            notifyChange(Axis3D.Y);
         }
      }

      @Override
      public void setZ(double z)
      {
         if (z != source.getRawZ())
         {
            source.setZ(z);
            notifyChange(Axis3D.Z);
         }
      }

      @Override
      public double getX()
      {
         notifyAccess(Axis3D.X);
         return source.getX();
      }

      @Override
      public double getY()
      {
         notifyAccess(Axis3D.Y);
         return source.getY();
      }

      @Override
      public double getZ()
      {
         notifyAccess(Axis3D.Z);
         return source.getZ();
      }

      @Override
      public double getRawX()
      {
         return source.getRawX();
      }

      @Override
      public double getRawY()
      {
         return source.getRawY();
      }

      @Override
      public double getRawZ()
      {
         return source.getRawY();
      }
   }

   /**
    * Creates a new rotation matrix that can be used to observe read and write operations.
    * 
    * @param accessListener the listener to be notified whenever a component of the rotation matrix is
    *                       being accessed. The corresponding constants {@link Axis3D} will be passed
    *                       to indicate the row and column respectively of the coefficient being
    *                       accessed. Can be {@code null}.
    * @param changeListener the listener to be notified whenever the rotation matrix has been modified.
    *                       Can be {@code null}.
    * @return the observable rotation matrix.
    */
   public static RotationMatrixBasics newObservableRotationMatrixBasics(EuclidAccessListener<Matrix3DElements> accessListener,
                                                                        EuclidChangeListener<Matrix3DElements> changeListener)
   {
      return newObservableRotationMatrixBasics(accessListener, changeListener, new RotationMatrix());
   }

   /**
    * Creates a linked rotation matrix that can be used to observe read and write operations on the
    * source.
    * 
    * @param accessListener the listener to be notified whenever a component of the rotation matrix is
    *                       being accessed. The corresponding constants {@link Axis3D} will be passed
    *                       to indicate the row and column respectively of the coefficient being
    *                       accessed. Can be {@code null}.
    * @param changeListener the listener to be notified whenever the rotation matrix has been modified.
    *                       Can be {@code null}.
    * @param source         the original rotation matrix to link and observe. Modifiable via the linked
    *                       rotation matrix interface.
    * @return the observable rotation matrix.
    */
   public static RotationMatrixBasics newObservableRotationMatrixBasics(EuclidAccessListener<Matrix3DElements> accessListener,
                                                                        EuclidChangeListener<Matrix3DElements> changeListener,
                                                                        RotationMatrixBasics source)
   {
      return new ObservableRotationMatrixBasics(accessListener, changeListener, source);
   }

   private static final class ObservableRotationMatrixBasics extends ObservableEuclidGeometry<Matrix3DElements, RotationMatrixBasics>
         implements RotationMatrixBasics
   {
      private ObservableRotationMatrixBasics(EuclidAccessListener<Matrix3DElements> accessListener,
                                             EuclidChangeListener<Matrix3DElements> changeListener,
                                             RotationMatrixBasics source)
      {
         super(accessListener, changeListener, source);
      }

      @Override
      public void setIdentity()
      {
         source.setIdentity();
         notifyChange(null);
      }

      @Override
      public void setToNaN()
      {
         source.setToNaN();
         notifyChange(null);
      }

      @Override
      public void normalize()
      {
         source.normalize();
         notifyChange(null);
      }

      @Override
      public void transpose()
      {
         source.transpose();
         notifyChange(null);
      }

      @Override
      public void setUnsafe(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
      {
         source.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         notifyChange(null);
      }

      @Override
      public void set(RotationMatrixReadOnly other)
      {
         if (other != source)
         {
            source.set(other);
            notifyChange(null);
         }
      }

      @Override
      public boolean isDirty()
      {
         return source.isDirty();
      }

      @Override
      public double getM00()
      {
         notifyAccess(Matrix3DElements.M00);
         return source.getM00();
      }

      @Override
      public double getM01()
      {
         notifyAccess(Matrix3DElements.M01);
         return source.getM01();
      }

      @Override
      public double getM02()
      {
         notifyAccess(Matrix3DElements.M02);
         return source.getM02();
      }

      @Override
      public double getM10()
      {
         notifyAccess(Matrix3DElements.M10);
         return source.getM10();
      }

      @Override
      public double getM11()
      {
         notifyAccess(Matrix3DElements.M11);
         return source.getM11();
      }

      @Override
      public double getM12()
      {
         notifyAccess(Matrix3DElements.M12);
         return source.getM12();
      }

      @Override
      public double getM20()
      {
         notifyAccess(Matrix3DElements.M20);
         return source.getM20();
      }

      @Override
      public double getM21()
      {
         notifyAccess(Matrix3DElements.M21);
         return source.getM21();
      }

      @Override
      public double getM22()
      {
         notifyAccess(Matrix3DElements.M22);
         return source.getM22();
      }
   }

   /**
    * Creates a new quaternion that can be used to observe read and write operations.
    * 
    * @param accessListener the listener to be notified whenever a component of the quaternion is being
    *                       accessed. The index of the component being accessed will be passed. Can be
    *                       {@code null}.
    * @param changeListener the listener to be notified whenever the quaternion has been modified. Can
    *                       be {@code null}.
    * @return the observable quaternion.
    */
   public static QuaternionBasics newObservableQuaternionBasics(EuclidAccessListener<Axis4D> accessListener, EuclidChangeListener<Axis4D> changeListener)
   {
      return newObservableQuaternionBasics(accessListener, changeListener, new Quaternion());
   }

   /**
    * Creates a linked quaternion that can be used to observe read and write operations on the source.
    * 
    * @param accessListener the listener to be notified whenever a component of the rotation matrix is
    *                       being accessed. The index of the component being accessed will be passed.
    *                       Can be {@code null}.
    * @param changeListener the listener to be notified whenever the quaternion has been modified. Can
    *                       be {@code null}.
    * @param source         the original vector to link and observe. Modifiable via the linked vector
    *                       interface.
    * @return the observable quaternion.
    */
   public static QuaternionBasics newObservableQuaternionBasics(EuclidAccessListener<Axis4D> accessListener,
                                                                EuclidChangeListener<Axis4D> changeListener,
                                                                QuaternionBasics source)
   {
      return new ObservableQuaternionBasics(accessListener, changeListener, source);
   }

   private static final class ObservableQuaternionBasics extends ObservableEuclidGeometry<Axis4D, QuaternionBasics> implements QuaternionBasics
   {
      private ObservableQuaternionBasics(EuclidAccessListener<Axis4D> accessListener, EuclidChangeListener<Axis4D> changeListener, QuaternionBasics source)
      {
         super(accessListener, changeListener, source);
      }

      @Override
      public void setUnsafe(double qx, double qy, double qz, double qs)
      {
         source.setUnsafe(qx, qy, qz, qs);
         notifyChange(null);
      }

      @Override
      public double getX()
      {
         notifyAccess(Axis4D.X);
         return source.getX();
      }

      @Override
      public double getY()
      {
         notifyAccess(Axis4D.Y);
         return source.getY();
      }

      @Override
      public double getZ()
      {
         notifyAccess(Axis4D.Z);
         return source.getZ();
      }

      @Override
      public double getS()
      {
         notifyAccess(Axis4D.S);
         return source.getS();
      }
   }

   public static RigidBodyTransformBasics newObservableRigidBodyTransformBasics(EuclidAccessListener<TransformComponents> accessListener,
                                                                                EuclidChangeListener<TransformComponents> changeListener,
                                                                                RigidBodyTransform source)
   {
      return new ObservableRigidBodyTransform(accessListener, changeListener, source);
   }

   public enum TransformComponents
   {
      ROTATION, TRANSLATION
   };

   private static final class ObservableRigidBodyTransform extends ObservableEuclidGeometry<TransformComponents, EuclidGeometry>
         implements RigidBodyTransformBasics
   {
      private final ObservableRotationMatrixBasics observableRotation;
      private final ObservableVector3DBasics observableTranslation;

      public ObservableRigidBodyTransform(EuclidAccessListener<TransformComponents> accessListener,
                                          EuclidChangeListener<TransformComponents> changeListener,
                                          RigidBodyTransform source)
      {
         super(accessListener, changeListener, source);
         EuclidAccessListener<Matrix3DElements> rotationAccessListener = null;
         if (accessListener != null)
            rotationAccessListener = a -> accessListener.onAccess(TransformComponents.ROTATION);
         EuclidChangeListener<Matrix3DElements> rotationChangeListener = null;
         if (changeListener != null)
            rotationChangeListener = a -> changeListener.changed(TransformComponents.ROTATION);
         observableRotation = new ObservableRotationMatrixBasics(rotationAccessListener, rotationChangeListener, source.getRotation());

         EuclidAccessListener<Axis3D> translationAccessListener = null;
         if (accessListener != null)
            translationAccessListener = a -> accessListener.onAccess(TransformComponents.TRANSLATION);
         EuclidChangeListener<Axis3D> translationChangeListener = null;
         if (changeListener != null)
            translationChangeListener = a -> changeListener.changed(TransformComponents.TRANSLATION);
         observableTranslation = new ObservableVector3DBasics(translationAccessListener, translationChangeListener, source.getTranslation());
      }

      @Override
      public Vector3DBasics getTranslation()
      {
         return observableTranslation;
      }

      @Override
      public RotationMatrixBasics getRotation()
      {
         return observableRotation;
      }

      @Override
      public String toString(String format)
      {
         return source.toString(format);
      }
   }

   private static abstract class ObservableEuclidGeometry<I, T extends EuclidGeometry> implements EuclidGeometry
   {
      protected final T source;
      private final EuclidAccessListener<I> accessListener;
      private final EuclidChangeListener<I> changeListener;

      private boolean isNotifying = false;

      public ObservableEuclidGeometry(EuclidAccessListener<I> accessListener, EuclidChangeListener<I> changeListener, T source)
      {
         this.source = source;
         this.accessListener = accessListener;
         this.changeListener = changeListener;
      }

      protected void notifyAccess(I valueAccessed)
      {
         if (accessListener == null || isNotifying)
            return;

         isNotifying = true;
         try
         {
            accessListener.onAccess(valueAccessed);
         }
         finally
         {
            isNotifying = false;
         }
      }

      protected void notifyChange(I valueChanged)
      {
         if (changeListener == null || isNotifying)
            return;

         isNotifying = true;
         try
         {
            changeListener.changed(valueChanged);
         }
         finally
         {
            isNotifying = false;
         }
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof EuclidGeometry)
            return equals((EuclidGeometry) object);
         else
            return false;
      }

      @Override
      public int hashCode()
      {
         return source.hashCode();
      }

      @Override
      public boolean equals(EuclidGeometry geometry)
      {
         if (geometry == this)
            return true;
         return source.equals(geometry);
      }

      @Override
      public boolean epsilonEquals(EuclidGeometry geometry, double epsilon)
      {
         if (geometry == this)
            return true;
         return source.epsilonEquals(geometry, epsilon);
      }

      @Override
      public String toString()
      {
         return source.toString();
      }
   }

   public static interface EuclidAccessListener<I>
   {
      void onAccess(I valueAccessed);
   }

   public static interface EuclidChangeListener<I>
   {
      void changed(I valueChanged);
   }
}
