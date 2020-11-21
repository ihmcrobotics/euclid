package us.ihmc.euclid.tools;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.IntConsumer;
import java.util.function.ObjDoubleConsumer;
import java.util.function.ToDoubleFunction;

import us.ihmc.euclid.Axis2D;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
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
      return new Point2DReadOnly()
      {
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
            return EuclidCoreIOTools.getTuple2DString(this);
         }
      };
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
      return new Vector2DReadOnly()
      {
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
            return EuclidCoreIOTools.getTuple2DString(this);
         }
      };
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
      return new Point3DReadOnly()
      {
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
            return EuclidCoreIOTools.getTuple3DString(this);
         }
      };
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
      return new Vector3DReadOnly()
      {
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
            return EuclidCoreIOTools.getTuple3DString(this);
         }
      };
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
      return new UnitVector2DReadOnly()
      {
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
            return EuclidCoreIOTools.getTuple2DString(this);
         }
      };
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
      return new UnitVector3DReadOnly()
      {
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
            return EuclidCoreIOTools.getTuple3DString(this);
         }
      };
   }

   public static QuaternionReadOnly newConjugateLinkedQuaternion(QuaternionReadOnly original)
   {
      return new QuaternionReadOnly()
      {
         @Override
         public double getX()
         {
            return -original.getX();
         }

         @Override
         public double getY()
         {
            return -original.getY();
         }

         @Override
         public double getZ()
         {
            return -original.getZ();
         }

         @Override
         public double getS()
         {
            return original.getS();
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
   }

   /**
    * Creates a new matrix 3D that is a read-only view of the transpose of the given {@code original}.
    *
    * @param original the original matrix to create linked transpose matrix for. Not modified.
    * @return the transpose read-only view of {@code original}.
    */
   public static Matrix3DReadOnly newTransposeLinkedMatrix3DReadOnly(Matrix3DReadOnly original)
   {
      return new Matrix3DReadOnly()
      {
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
            return EuclidCoreIOTools.getMatrix3DString(this);
         }
      };
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
      return new Matrix3DReadOnly()
      {
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
            return EuclidCoreIOTools.getMatrix3DString(this);
         }
      };
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
      return new Matrix3DReadOnly()
      {
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
            return EuclidCoreIOTools.getMatrix3DString(this);
         }
      };
   }

   /**
    * Creates a linked point that can be used to observe access to the source point's coordinates.
    * 
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the point is
    *                              being accessed. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate being accessed.
    * @param source                the original point to link and observe. Not modified.
    * @return the observable point.
    */
   public static Point2DReadOnly newObservablePoint2DReadOnly(Consumer<Axis2D> valueAccessedListener, Point2DReadOnly source)
   {
      ToDoubleFunction<Axis2D> notifier = readNotification(valueAccessedListener, axis -> axis.extract(source));
      return newLinkedPoint2DReadOnly(() -> notifier.applyAsDouble(Axis2D.X), () -> notifier.applyAsDouble(Axis2D.Y));
   }

   /**
    * Creates a linked point that can be used to observe access to the source point's coordinates.
    * 
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the point is
    *                              being accessed. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the coordinate being accessed.
    * @param source                the original point to link and observe. Not modified.
    * @return the observable point.
    */
   public static Point3DReadOnly newObservablePoint3DReadOnly(Consumer<Axis3D> valueAccessedListener, Point3DReadOnly source)
   {
      ToDoubleFunction<Axis3D> notifier = readNotification(valueAccessedListener, axis -> axis.extract(source));
      return newLinkedPoint3DReadOnly(() -> notifier.applyAsDouble(Axis3D.X), () -> notifier.applyAsDouble(Axis3D.Y), () -> notifier.applyAsDouble(Axis3D.Z));
   }

   /**
    * Creates a linked vector that can be used to observe access to the source vector's components.
    * 
    * @param valueAccessedListener the listener to be notified whenever a component of the vector is
    *                              being accessed. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the component being accessed.
    * @param source                the original vector to link and observe. Not modified.
    * @return the observable vector.
    */
   public static Vector2DReadOnly newObservableVector2DReadOnly(Consumer<Axis2D> valueAccessedListener, Vector2DReadOnly source)
   {
      ToDoubleFunction<Axis2D> notifier = readNotification(valueAccessedListener, axis -> axis.extract(source));
      return newLinkedVector2DReadOnly(() -> notifier.applyAsDouble(Axis2D.X), () -> notifier.applyAsDouble(Axis2D.Y));
   }

   /**
    * Creates a linked vector that can be used to observe access to the source vector's components.
    * 
    * @param valueAccessedListener the listener to be notified whenever a component of the vector is
    *                              being accessed. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the component being accessed.
    * @param source                the original vector to link and observe. Not modified.
    * @return the observable vector.
    */
   public static Vector3DReadOnly newObservableVector3DReadOnly(Consumer<Axis3D> valueAccessedListener, Vector3DReadOnly source)
   {
      ToDoubleFunction<Axis3D> notifier = readNotification(valueAccessedListener, axis -> axis.extract(source));
      return newLinkedVector3DReadOnly(() -> notifier.applyAsDouble(Axis3D.X), () -> notifier.applyAsDouble(Axis3D.Y), () -> notifier.applyAsDouble(Axis3D.Z));
   }

   private static <T> ToDoubleFunction<T> readNotification(Consumer<T> listener, ToDoubleFunction<T> valueReader)
   {
      return new ToDoubleFunction<T>()
      {
         private boolean isNotifying = false;

         @Override
         public double applyAsDouble(T value)
         {
            if (!isNotifying)
            {
               isNotifying = true;
               listener.accept(value);
               isNotifying = false;
            }
            return valueReader.applyAsDouble(value);
         }
      };
   }

   /**
    * Creates a linked unit vector that can be used to observe access to the source unit vector's
    * components.
    * 
    * @param valueAccessedListener the listener to be notified whenever a component of the vector is
    *                              being accessed. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the component being accessed.
    * @param source                the original unit vector to link and observe. Not modified.
    * @return the observable unit vector.
    */
   public static UnitVector2DReadOnly newObservableUnitVector2DReadOnly(Consumer<Axis2D> valueAccessedListener, UnitVector2DReadOnly source)
   {
      ToDoubleFunction<Axis2D> notifier = readNotification(valueAccessedListener, axis -> axis.extract(source));
      return new UnitVector2DReadOnly()
      {
         @Override
         public boolean isDirty()
         {
            return source.isDirty();
         }

         @Override
         public double getX()
         {
            return notifier.applyAsDouble(Axis2D.X);
         }

         @Override
         public double getY()
         {
            return notifier.applyAsDouble(Axis2D.Y);
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
            return EuclidCoreIOTools.getTuple2DString(this);
         }
      };
   }

   /**
    * Creates a linked unit vector that can be used to observe access to the source unit vector's
    * components.
    * 
    * @param valueAccessedListener the listener to be notified whenever a component of the vector is
    *                              being accessed. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the component being accessed.
    * @param source                the original unit vector to link and observe. Not modified.
    * @return the observable unit vector.
    */
   public static UnitVector3DReadOnly newObservableUnitVector3DReadOnly(Consumer<Axis3D> valueAccessedListener, UnitVector3DReadOnly source)
   {
      ToDoubleFunction<Axis3D> notifier = readNotification(valueAccessedListener, axis -> axis.extract(source));
      return new UnitVector3DReadOnly()
      {
         @Override
         public boolean isDirty()
         {
            return source.isDirty();
         }

         @Override
         public double getX()
         {
            return notifier.applyAsDouble(Axis3D.X);
         }

         @Override
         public double getY()
         {
            return notifier.applyAsDouble(Axis3D.Y);
         }

         @Override
         public double getZ()
         {
            return notifier.applyAsDouble(Axis3D.Z);
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
            return EuclidCoreIOTools.getTuple3DString(this);
         }
      };
   }

   /**
    * Creates a linked rotation matrix that can be used to observe access to the source rotation
    * matrix's components.
    * 
    * @param valueAccessedListener the listener to be notified whenever a component of the rotation
    *                              matrix is being accessed. The corresponding constants {@link Axis3D}
    *                              will be passed to indicate the row and column respectively of the
    *                              coefficient being accessed.
    * @param source                the original rotation matrix to link and observe. Not modified.
    * @return the observable rotation matrix.
    */
   public static RotationMatrixReadOnly newObservableRotationMatrixReadOnly(BiConsumer<Axis3D, Axis3D> valueAccessedListener, RotationMatrixReadOnly source)
   {
      return new RotationMatrixReadOnly()
      {
         private boolean isNotifying = false;

         @Override
         public boolean isDirty()
         {
            return source.isDirty();
         }

         @Override
         public double getM00()
         {
            notifyAccessListener(Axis3D.X, Axis3D.X);
            return source.getM00();
         }

         @Override
         public double getM01()
         {
            notifyAccessListener(Axis3D.X, Axis3D.Y);
            return source.getM01();
         }

         @Override
         public double getM02()
         {
            notifyAccessListener(Axis3D.X, Axis3D.Z);
            return source.getM02();
         }

         @Override
         public double getM10()
         {
            notifyAccessListener(Axis3D.Y, Axis3D.X);
            return source.getM10();
         }

         @Override
         public double getM11()
         {
            notifyAccessListener(Axis3D.Y, Axis3D.Y);
            return source.getM11();
         }

         @Override
         public double getM12()
         {
            notifyAccessListener(Axis3D.Y, Axis3D.Z);
            return source.getM12();
         }

         @Override
         public double getM20()
         {
            notifyAccessListener(Axis3D.Z, Axis3D.X);
            return source.getM20();
         }

         @Override
         public double getM21()
         {
            notifyAccessListener(Axis3D.Z, Axis3D.Y);
            return source.getM21();
         }

         @Override
         public double getM22()
         {
            notifyAccessListener(Axis3D.Z, Axis3D.Z);
            return source.getM22();
         }

         private void notifyAccessListener(Axis3D row, Axis3D column)
         {
            if (valueAccessedListener == null)
               return;
            if (isNotifying)
               return;

            isNotifying = true;
            valueAccessedListener.accept(row, column);
            isNotifying = false;
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
            return EuclidCoreIOTools.getMatrix3DString(this);
         }
      };
   }

   /**
    * Creates a linked quaternion that can be used to observe access to the source rotation matrix's
    * components.
    * 
    * @param valueAccessedListener the listener to be notified whenever a component of the quaternion
    *                              is being accessed. The index of the component being accessed will be
    *                              passed.
    * @param source                the original quaternion to link and observe. Not modified.
    * @return the observable quaternion.
    */
   public static QuaternionReadOnly newObservableQuaternionReadOnly(IntConsumer valueAccessedListener, QuaternionReadOnly source)
   {
      return new QuaternionReadOnly()
      {
         private boolean isNotifying = false;

         @Override
         public double getX()
         {
            notifyAccessListener(0);
            return source.getX();
         }

         @Override
         public double getY()
         {
            notifyAccessListener(1);
            return source.getY();
         }

         @Override
         public double getZ()
         {
            notifyAccessListener(2);
            return source.getZ();
         }

         @Override
         public double getS()
         {
            notifyAccessListener(3);
            return source.getS();
         }

         private void notifyAccessListener(int index)
         {
            if (valueAccessedListener == null)
               return;
            if (isNotifying)
               return;

            isNotifying = true;
            valueAccessedListener.accept(index);
            isNotifying = false;
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
   }

   /**
    * Creates a new point that can be used to observe read and write operations.
    * 
    * @param valueChangedListener  the listener to be notified whenever a coordinate of the point has
    *                              been modified. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the point is
    *                              being accessed. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate being accessed. Can be
    *                              {@code null}.
    * @return the observable point.
    */
   public static Point2DBasics newObservablePoint2DBasics(ObjDoubleConsumer<Axis2D> valueChangedListener, Consumer<Axis2D> valueAccessedListener)
   {
      return newObservablePoint2DBasics(valueChangedListener, valueAccessedListener, new Point2D());
   }

   /**
    * Creates a linked point that can be used to observe read and write operations on the source.
    * 
    * @param valueChangedListener  the listener to be notified whenever a coordinate of the point has
    *                              been modified. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the point is
    *                              being accessed. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate being accessed. Can be
    *                              {@code null}.
    * @param source                the original point to link and observe. Modifiable via the linked
    *                              point interface.
    * @return the observable point.
    */
   public static Point2DBasics newObservablePoint2DBasics(ObjDoubleConsumer<Axis2D> valueChangedListener, Consumer<Axis2D> valueAccessedListener,
                                                          Point2DBasics source)
   {
      return new Point2DBasics()
      {
         private boolean isNotifying = false;

         @Override
         public void setX(double x)
         {
            if (x != source.getX())
            {
               source.setX(x);
               notifyChangeListener(Axis2D.X, x);
            }
         }

         @Override
         public void setY(double y)
         {
            if (y != source.getY())
            {
               source.setY(y);
               notifyChangeListener(Axis2D.Y, y);
            }
         }

         @Override
         public double getX()
         {
            notifyAccessListener(Axis2D.X);
            return source.getX();
         }

         @Override
         public double getY()
         {
            notifyAccessListener(Axis2D.Y);
            return source.getY();
         }

         private void notifyChangeListener(Axis2D axis, double newValue)
         {
            if (valueChangedListener == null)
               return;

            if (isNotifying)
               return;

            isNotifying = true;
            valueChangedListener.accept(axis, newValue);
            isNotifying = false;
         }

         private void notifyAccessListener(Axis2D axis)
         {
            if (valueAccessedListener == null)
               return;
            if (isNotifying)
               return;

            isNotifying = true;
            valueAccessedListener.accept(axis);
            isNotifying = false;
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
            return EuclidCoreIOTools.getTuple2DString(this);
         }
      };
   }

   /**
    * Creates a new point that can be used to observe read and write operations.
    * 
    * @param valueChangedListener  the listener to be notified whenever a coordinate of the point has
    *                              been modified. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the coordinate that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the point is
    *                              being accessed. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the coordinate being accessed. Can be
    *                              {@code null}.
    * @return the observable point.
    */
   public static Point3DBasics newObservablePoint3DBasics(ObjDoubleConsumer<Axis3D> valueChangedListener, Consumer<Axis3D> valueAccessedListener)
   {
      return newObservablePoint3DBasics(valueChangedListener, valueAccessedListener, new Point3D());
   }

   /**
    * Creates a linked point that can be used to observe read and write operations on the source.
    * 
    * @param valueChangedListener  the listener to be notified whenever a coordinate of the point has
    *                              been modified. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the coordinate that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the point is
    *                              being accessed. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the coordinate being accessed. Can be
    *                              {@code null}.
    * @param source                the original point to link and observe. Modifiable via the linked
    *                              point interface.
    * @return the observable point.
    */
   public static Point3DBasics newObservablePoint3DBasics(ObjDoubleConsumer<Axis3D> valueChangedListener, Consumer<Axis3D> valueAccessedListener,
                                                          Point3DBasics source)
   {

      return new Point3DBasics()
      {
         private boolean isNotifying = false;

         @Override
         public void setX(double x)
         {
            if (x != source.getX())
            {
               source.setX(x);
               notifyChangeListener(Axis3D.X, x);
            }
         }

         @Override
         public void setY(double y)
         {
            if (y != source.getY())
            {
               source.setY(y);
               notifyChangeListener(Axis3D.Y, y);
            }
         }

         @Override
         public void setZ(double z)
         {
            if (z != source.getZ())
            {
               source.setZ(z);
               notifyChangeListener(Axis3D.Z, z);
            }
         }

         @Override
         public double getX()
         {
            notifyAccessListener(Axis3D.X);
            return source.getX();
         }

         @Override
         public double getY()
         {
            notifyAccessListener(Axis3D.Y);
            return source.getY();
         }

         @Override
         public double getZ()
         {
            notifyAccessListener(Axis3D.Z);
            return source.getZ();
         }

         private void notifyChangeListener(Axis3D axis, double newValue)
         {
            if (valueChangedListener == null)
               return;

            if (isNotifying)
               return;

            isNotifying = true;
            valueChangedListener.accept(axis, newValue);
            isNotifying = false;
         }

         private void notifyAccessListener(Axis3D axis)
         {
            if (valueAccessedListener == null)
               return;
            if (isNotifying)
               return;

            isNotifying = true;
            valueAccessedListener.accept(axis);
            isNotifying = false;
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
            return EuclidCoreIOTools.getTuple3DString(this);
         }
      };
   }

   /**
    * Creates a new vector that can be used to observe read and write operations.
    * 
    * @param valueChangedListener  the listener to be notified whenever a component of the vector has
    *                              been modified. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the component that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a component of the vector is
    *                              being accessed. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the component being accessed. Can be
    *                              {@code null}.
    * @return the observable vector.
    */
   public static Vector2DBasics newObservableVector2DBasics(ObjDoubleConsumer<Axis2D> valueChangedListener, Consumer<Axis2D> valueAccessedListener)
   {
      return newObservableVector2DBasics(valueChangedListener, valueAccessedListener, new Vector2D());
   }

   /**
    * Creates a linked vector that can be used to observe read and write operations on the source.
    * 
    * @param valueChangedListener  the listener to be notified whenever a coordinate of the vector has
    *                              been modified. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the vector is
    *                              being accessed. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate being accessed. Can be
    *                              {@code null}.
    * @param source                the original vector to link and observe. Modifiable via the linked
    *                              vector interface.
    * @return the observable vector.
    */
   public static Vector2DBasics newObservableVector2DBasics(ObjDoubleConsumer<Axis2D> valueChangedListener, Consumer<Axis2D> valueAccessedListener,
                                                            Vector2DBasics source)
   {
      return new Vector2DBasics()
      {
         private boolean isNotifying = false;

         @Override
         public void setX(double x)
         {
            if (x != source.getX())
            {
               source.setX(x);
               notifyChangeListener(Axis2D.X, x);
            }
         }

         @Override
         public void setY(double y)
         {
            if (y != source.getY())
            {
               source.setY(y);
               notifyChangeListener(Axis2D.Y, y);
            }
         }

         @Override
         public double getX()
         {
            notifyAccessListener(Axis2D.X);
            return source.getX();
         }

         @Override
         public double getY()
         {
            notifyAccessListener(Axis2D.Y);
            return source.getY();
         }

         private void notifyChangeListener(Axis2D axis, double newValue)
         {
            if (valueChangedListener == null)
               return;

            if (isNotifying)
               return;

            isNotifying = true;
            valueChangedListener.accept(axis, newValue);
            isNotifying = false;
         }

         private void notifyAccessListener(Axis2D axis)
         {
            if (valueAccessedListener == null)
               return;
            if (isNotifying)
               return;

            isNotifying = true;
            valueAccessedListener.accept(axis);
            isNotifying = false;
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
            return EuclidCoreIOTools.getTuple2DString(this);
         }
      };
   }

   /**
    * Creates a new vector that can be used to observe read and write operations.
    * 
    * @param valueChangedListener  the listener to be notified whenever a component of the vector has
    *                              been modified. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the component that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a component of the vector is
    *                              being accessed. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the component being accessed. Can be
    *                              {@code null}.
    * @return the observable vector.
    */
   public static Vector3DBasics newObservableVector3DBasics(ObjDoubleConsumer<Axis3D> valueChangedListener, Consumer<Axis3D> valueAccessedListener)
   {
      return newObservableVector3DBasics(valueChangedListener, valueAccessedListener, new Vector3D());
   }

   /**
    * Creates a linked vector that can be used to observe read and write operations on the source.
    * 
    * @param valueChangedListener  the listener to be notified whenever a coordinate of the vector has
    *                              been modified. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the coordinate that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the vector is
    *                              being accessed. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the coordinate being accessed. Can be
    *                              {@code null}.
    * @param source                the original vector to link and observe. Modifiable via the linked
    *                              vector interface.
    * @return the observable vector.
    */
   public static Vector3DBasics newObservableVector3DBasics(ObjDoubleConsumer<Axis3D> valueChangedListener, Consumer<Axis3D> valueAccessedListener,
                                                            Vector3DBasics source)
   {
      return new Vector3DBasics()
      {
         private boolean isNotifying = false;

         @Override
         public void setX(double x)
         {
            if (x != source.getX())
            {
               source.setX(x);
               notifyChangeListener(Axis3D.X, x);
            }
         }

         @Override
         public void setY(double y)
         {
            if (y != source.getY())
            {
               source.setY(y);
               notifyChangeListener(Axis3D.Y, y);
            }
         }

         @Override
         public void setZ(double z)
         {
            if (z != source.getZ())
            {
               source.setZ(z);
               notifyChangeListener(Axis3D.Z, z);
            }
         }

         @Override
         public double getX()
         {
            notifyAccessListener(Axis3D.X);
            return source.getX();
         }

         @Override
         public double getY()
         {
            notifyAccessListener(Axis3D.Y);
            return source.getY();
         }

         @Override
         public double getZ()
         {
            notifyAccessListener(Axis3D.Z);
            return source.getZ();
         }

         private void notifyChangeListener(Axis3D axis, double newValue)
         {
            if (valueChangedListener == null)
               return;

            if (isNotifying)
               return;

            isNotifying = true;
            valueChangedListener.accept(axis, newValue);
            isNotifying = false;
         }

         private void notifyAccessListener(Axis3D axis)
         {
            if (valueAccessedListener == null)
               return;
            if (isNotifying)
               return;

            isNotifying = true;
            valueAccessedListener.accept(axis);
            isNotifying = false;
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
            return EuclidCoreIOTools.getTuple3DString(this);
         }
      };
   }

   /**
    * Creates a new vector that can be used to observe read and write operations.
    * 
    * @param valueChangedListener  the listener to be notified whenever a component of the vector has
    *                              been modified. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the component that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a component of the vector is
    *                              being accessed. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the component being accessed. Can be
    *                              {@code null}.
    * @return the observable vector.
    */
   public static UnitVector2DBasics newObservableUnitVector2DBasics(ObjDoubleConsumer<Axis2D> valueChangedListener, Consumer<Axis2D> valueAccessedListener)
   {
      return newObservableUnitVector2DBasics(valueChangedListener, valueAccessedListener, new UnitVector2D());
   }

   /**
    * Creates a linked vector that can be used to observe read and write operations on the source.
    * 
    * @param valueChangedListener  the listener to be notified whenever a coordinate of the vector has
    *                              been modified. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the vector is
    *                              being accessed. The corresponding constant {@link Axis2D} will be
    *                              passed to indicate the coordinate being accessed. Can be
    *                              {@code null}.
    * @param source                the original vector to link and observe. Modifiable via the linked
    *                              vector interface.
    * @return the observable vector.
    */
   public static UnitVector2DBasics newObservableUnitVector2DBasics(ObjDoubleConsumer<Axis2D> valueChangedListener, Consumer<Axis2D> valueAccessedListener,
                                                                    UnitVector2DBasics source)
   {
      return new UnitVector2DBasics()
      {
         private boolean isNotifying = false;

         @Override
         public void absolute()
         {
            boolean notifyX = getRawX() < 0.0;
            boolean notifyY = getRawY() < 0.0;

            source.absolute();

            if (notifyX)
               notifyChangeListener(Axis2D.X, getRawX());
            if (notifyY)
               notifyChangeListener(Axis2D.Y, getRawY());
         }

         @Override
         public void negate()
         {
            source.negate();

            notifyChangeListener(Axis2D.X, getRawX());
            notifyChangeListener(Axis2D.Y, getRawY());
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
            boolean notify = isDirty();

            source.normalize();

            if (notify)
            {
               notifyChangeListener(Axis2D.X, getRawX());
               notifyChangeListener(Axis2D.Y, getRawY());
            }
         }

         @Override
         public void set(UnitVector2DReadOnly other)
         {
            boolean notifyX = getRawX() != other.getRawX();
            boolean notifyY = getRawY() != other.getRawY();

            source.set(other);

            if (notifyX)
               notifyChangeListener(Axis2D.X, getRawX());
            if (notifyY)
               notifyChangeListener(Axis2D.Y, getRawY());
         }

         @Override
         public void setX(double x)
         {
            if (x != source.getRawX())
            {
               source.setX(x);
               notifyChangeListener(Axis2D.X, x);
            }
         }

         @Override
         public void setY(double y)
         {
            if (y != source.getRawY())
            {
               source.setY(y);
               notifyChangeListener(Axis2D.Y, y);
            }
         }

         @Override
         public double getX()
         {
            notifyAccessListener(Axis2D.X);
            return source.getX();
         }

         @Override
         public double getY()
         {
            notifyAccessListener(Axis2D.Y);
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

         private void notifyChangeListener(Axis2D axis, double newValue)
         {
            if (valueChangedListener == null)
               return;

            if (isNotifying)
               return;

            isNotifying = true;
            valueChangedListener.accept(axis, newValue);
            isNotifying = false;
         }

         private void notifyAccessListener(Axis2D axis)
         {
            if (valueAccessedListener == null)
               return;
            if (isNotifying)
               return;

            isNotifying = true;
            valueAccessedListener.accept(axis);
            isNotifying = false;
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
            return EuclidCoreIOTools.getTuple2DString(this);
         }
      };
   }

   /**
    * Creates a new vector that can be used to observe read and write operations.
    * 
    * @param valueChangedListener  the listener to be notified whenever a component of the vector has
    *                              been modified. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the component that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a component of the vector is
    *                              being accessed. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the component being accessed. Can be
    *                              {@code null}.
    * @return the observable vector.
    */
   public static UnitVector3DBasics newObservableUnitVector3DBasics(ObjDoubleConsumer<Axis3D> valueChangedListener, Consumer<Axis3D> valueAccessedListener)
   {
      return newObservableUnitVector3DBasics(valueChangedListener, valueAccessedListener, new UnitVector3D());
   }

   /**
    * Creates a linked vector that can be used to observe read and write operations on the source.
    * 
    * @param valueChangedListener  the listener to be notified whenever a coordinate of the vector has
    *                              been modified. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the coordinate that was changed alongside its new
    *                              value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a coordinate of the vector is
    *                              being accessed. The corresponding constant {@link Axis3D} will be
    *                              passed to indicate the coordinate being accessed. Can be
    *                              {@code null}.
    * @param source                the original vector to link and observe. Modifiable via the linked
    *                              vector interface.
    * @return the observable vector.
    */
   public static UnitVector3DBasics newObservableUnitVector3DBasics(ObjDoubleConsumer<Axis3D> valueChangedListener, Consumer<Axis3D> valueAccessedListener,
                                                                    UnitVector3DBasics source)
   {
      return new UnitVector3DBasics()
      {
         private boolean isNotifying = false;

         @Override
         public void absolute()
         {
            boolean notifyX = getRawX() < 0.0;
            boolean notifyY = getRawY() < 0.0;
            boolean notifyZ = getRawY() < 0.0;

            source.absolute();

            if (notifyX)
               notifyChangeListener(Axis3D.X, getRawX());
            if (notifyY)
               notifyChangeListener(Axis3D.Y, getRawY());
            if (notifyZ)
               notifyChangeListener(Axis3D.Z, getRawZ());
         }

         @Override
         public void negate()
         {
            source.negate();

            notifyChangeListener(Axis3D.X, getRawX());
            notifyChangeListener(Axis3D.Y, getRawY());
            notifyChangeListener(Axis3D.Z, getRawZ());
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
            boolean notify = isDirty();

            source.normalize();

            if (notify)
            {
               notifyChangeListener(Axis3D.X, getRawX());
               notifyChangeListener(Axis3D.Y, getRawY());
               notifyChangeListener(Axis3D.Z, getRawZ());
            }
         }

         @Override
         public void set(UnitVector3DReadOnly other)
         {
            boolean notifyX = getRawX() != other.getRawX();
            boolean notifyY = getRawY() != other.getRawY();
            boolean notifyZ = getRawZ() != other.getRawZ();

            source.set(other);

            if (notifyX)
               notifyChangeListener(Axis3D.X, getRawX());
            if (notifyY)
               notifyChangeListener(Axis3D.Y, getRawY());
            if (notifyZ)
               notifyChangeListener(Axis3D.Z, getRawZ());
         }

         @Override
         public void setX(double x)
         {
            if (x != source.getRawX())
            {
               source.setX(x);
               notifyChangeListener(Axis3D.X, x);
            }
         }

         @Override
         public void setY(double y)
         {
            if (y != source.getRawY())
            {
               source.setY(y);
               notifyChangeListener(Axis3D.Y, y);
            }
         }

         @Override
         public void setZ(double z)
         {
            if (z != getRawZ())
            {
               source.setZ(z);
               notifyChangeListener(Axis3D.Z, z);
            }
         }

         @Override
         public double getX()
         {
            notifyAccessListener(Axis3D.X);
            return source.getX();
         }

         @Override
         public double getY()
         {
            notifyAccessListener(Axis3D.Y);
            return source.getY();
         }

         @Override
         public double getZ()
         {
            notifyAccessListener(Axis3D.Z);
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
            return source.getRawZ();
         }

         private void notifyChangeListener(Axis3D axis, double newValue)
         {
            if (valueChangedListener == null)
               return;

            if (isNotifying)
               return;

            isNotifying = true;
            valueChangedListener.accept(axis, newValue);
            isNotifying = false;
         }

         private void notifyAccessListener(Axis3D axis)
         {
            if (valueAccessedListener == null)
               return;
            if (isNotifying)
               return;

            isNotifying = true;
            valueAccessedListener.accept(axis);
            isNotifying = false;
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
            return EuclidCoreIOTools.getTuple3DString(this);
         }
      };
   }

   /**
    * Creates a new rotation matrix that can be used to observe read and write operations.
    * 
    * @param valueChangedListener  the listener to be notified whenever the rotation matrix has been
    *                              modified. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a component of the rotation
    *                              matrix is being accessed. The corresponding constants {@link Axis3D}
    *                              will be passed to indicate the row and column respectively of the
    *                              coefficient being accessed. Can be {@code null}.
    * @return the observable rotation matrix.
    */
   public static RotationMatrixBasics newObservableRotationMatrixBasics(Runnable valueChangedListener, BiConsumer<Axis3D, Axis3D> valueAccessedListener)
   {
      return newObservableRotationMatrixBasics(valueChangedListener, valueAccessedListener, new RotationMatrix());
   }

   /**
    * Creates a linked rotation matrix that can be used to observe read and write operations on the
    * source.
    * 
    * @param valueChangedListener  the listener to be notified whenever the rotation matrix has been
    *                              modified. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a component of the rotation
    *                              matrix is being accessed. The corresponding constants {@link Axis3D}
    *                              will be passed to indicate the row and column respectively of the
    *                              coefficient being accessed. Can be {@code null}.
    * @param source                the original rotation matrix to link and observe. Modifiable via the
    *                              linked rotation matrix interface.
    * @return the observable rotation matrix.
    */
   public static RotationMatrixBasics newObservableRotationMatrixBasics(Runnable valueChangedListener, BiConsumer<Axis3D, Axis3D> valueAccessedListener,
                                                                        RotationMatrixBasics source)
   {
      return new RotationMatrixBasics()
      {
         private boolean isNotifying = false;

         @Override
         public void setIdentity()
         {
            source.setIdentity();
            notifyChangeListener();
         }

         @Override
         public void setToNaN()
         {
            source.setToNaN();
            notifyChangeListener();
         }

         @Override
         public void normalize()
         {
            source.normalize();
            notifyChangeListener();
         }

         @Override
         public void transpose()
         {
            source.transpose();
            notifyChangeListener();
         }

         @Override
         public void setUnsafe(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
         {
            source.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
            notifyChangeListener();
         }

         @Override
         public void set(RotationMatrixReadOnly other)
         {
            if (other != source)
            {
               source.set(other);
               notifyChangeListener();
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
            notifyAccessListener(Axis3D.X, Axis3D.X);
            return source.getM00();
         }

         @Override
         public double getM01()
         {
            notifyAccessListener(Axis3D.X, Axis3D.Y);
            return source.getM01();
         }

         @Override
         public double getM02()
         {
            notifyAccessListener(Axis3D.X, Axis3D.Z);
            return source.getM02();
         }

         @Override
         public double getM10()
         {
            notifyAccessListener(Axis3D.Y, Axis3D.X);
            return source.getM10();
         }

         @Override
         public double getM11()
         {
            notifyAccessListener(Axis3D.Y, Axis3D.Y);
            return source.getM11();
         }

         @Override
         public double getM12()
         {
            notifyAccessListener(Axis3D.Y, Axis3D.Z);
            return source.getM12();
         }

         @Override
         public double getM20()
         {
            notifyAccessListener(Axis3D.Z, Axis3D.X);
            return source.getM20();
         }

         @Override
         public double getM21()
         {
            notifyAccessListener(Axis3D.Z, Axis3D.Y);
            return source.getM21();
         }

         @Override
         public double getM22()
         {
            notifyAccessListener(Axis3D.Z, Axis3D.Z);
            return source.getM22();
         }

         private void notifyChangeListener()
         {
            if (valueChangedListener == null)
               return;

            if (isNotifying)
               return;

            isNotifying = true;
            valueChangedListener.run();
            isNotifying = false;
         }

         private void notifyAccessListener(Axis3D row, Axis3D column)
         {
            if (valueAccessedListener == null)
               return;
            if (isNotifying)
               return;

            isNotifying = true;
            valueAccessedListener.accept(row, column);
            isNotifying = false;
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
            return EuclidCoreIOTools.getMatrix3DString(this);
         }
      };
   }

   /**
    * Creates a new quaternion that can be used to observe read and write operations.
    * 
    * @param valueChangedListener  the listener to be notified whenever the quaternion has been
    *                              modified. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a component of the quaternion
    *                              is being accessed. The index of the component being accessed will be
    *                              passed. Can be {@code null}.
    * @return the observable quaternion.
    */
   public static QuaternionBasics newObservableQuaternionBasics(Runnable valueChangedListener, IntConsumer valueAccessedListener)
   {
      return newObservableQuaternionBasics(valueChangedListener, valueAccessedListener, new Quaternion());
   }

   /**
    * Creates a linked quaternion that can be used to observe read and write operations on the source.
    * 
    * @param valueChangedListener  the listener to be notified whenever the quaternion has been
    *                              modified. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a component of the rotation
    *                              matrix is being accessed. The index of the component being accessed
    *                              will be passed. Can be {@code null}.
    * @param source                the original vector to link and observe. Modifiable via the linked
    *                              vector interface.
    * @return the observable quaternion.
    */
   public static QuaternionBasics newObservableQuaternionBasics(Runnable valueChangedListener, IntConsumer valueAccessedListener, QuaternionBasics source)
   {
      return new QuaternionBasics()
      {
         private boolean isNotifying = false;

         @Override
         public void setUnsafe(double qx, double qy, double qz, double qs)
         {
            source.setUnsafe(qx, qy, qz, qs);
            notifyChangeListener();
         }

         @Override
         public double getX()
         {
            notifyAccessListener(0);
            return source.getX();
         }

         @Override
         public double getY()
         {
            notifyAccessListener(1);
            return source.getY();
         }

         @Override
         public double getZ()
         {
            notifyAccessListener(2);
            return source.getZ();
         }

         @Override
         public double getS()
         {
            notifyAccessListener(3);
            return source.getS();
         }

         private void notifyChangeListener()
         {
            if (valueChangedListener == null)
               return;

            if (isNotifying)
               return;

            isNotifying = true;
            valueChangedListener.run();
            isNotifying = false;
         }

         private void notifyAccessListener(int index)
         {
            if (valueAccessedListener == null)
               return;
            if (isNotifying)
               return;

            isNotifying = true;
            valueAccessedListener.accept(index);
            isNotifying = false;
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
   }

   public static Matrix3DBasics newObservableMatrix3DBasics(ObjObjDoubleConsumer<Axis3D, Axis3D> valueChangedListener,
                                                            BiConsumer<Axis3D, Axis3D> valueAccessedListener)
   {
      return newObservableMatrix3DBasics(valueChangedListener, valueAccessedListener, new Matrix3D());
   }

   public static Matrix3DBasics newObservableMatrix3DBasics(ObjObjDoubleConsumer<Axis3D, Axis3D> valueChangedListener,
                                                            BiConsumer<Axis3D, Axis3D> valueAccessedListener, Matrix3DBasics source)
   {
      return new Matrix3DBasics()
      {
         private boolean isNotifying = false;

         @Override
         public void setM00(double m00)
         {
            if (m00 != source.getM00())
            {
               source.setM00(m00);
               notifyChangeListener(Axis3D.X, Axis3D.X, m00);
            }
         }

         @Override
         public void setM01(double m01)
         {
            if (m01 != source.getM01())
            {
               source.setM01(m01);
               notifyChangeListener(Axis3D.X, Axis3D.Y, m01);
            }
         }

         @Override
         public void setM02(double m02)
         {
            if (m02 != source.getM02())
            {
               source.setM02(m02);
               notifyChangeListener(Axis3D.X, Axis3D.Z, m02);
            }
         }

         @Override
         public void setM10(double m10)
         {
            if (m10 != source.getM10())
            {
               source.setM10(m10);
               notifyChangeListener(Axis3D.Y, Axis3D.X, m10);
            }
         }

         @Override
         public void setM11(double m11)
         {
            if (m11 != source.getM11())
            {
               source.setM11(m11);
               notifyChangeListener(Axis3D.Y, Axis3D.Y, m11);
            }
         }

         @Override
         public void setM12(double m12)
         {
            if (m12 != source.getM12())
            {
               source.setM12(m12);
               notifyChangeListener(Axis3D.Y, Axis3D.Z, m12);
            }
         }

         @Override
         public void setM20(double m20)
         {
            if (m20 != source.getM20())
            {
               source.setM20(m20);
               notifyChangeListener(Axis3D.Z, Axis3D.X, m20);
            }
         }

         @Override
         public void setM21(double m21)
         {
            if (m21 != source.getM21())
            {
               source.setM21(m21);
               notifyChangeListener(Axis3D.Z, Axis3D.Y, m21);
            }
         }

         @Override
         public void setM22(double m22)
         {
            if (m22 != source.getM22())
            {
               source.setM22(m22);
               notifyChangeListener(Axis3D.Z, Axis3D.Z, m22);
            }
         }

         @Override
         public double getM00()
         {
            notifyAccessListener(Axis3D.X, Axis3D.X);
            return source.getM00();
         }

         @Override
         public double getM01()
         {
            notifyAccessListener(Axis3D.X, Axis3D.Y);
            return source.getM01();
         }

         @Override
         public double getM02()
         {
            notifyAccessListener(Axis3D.X, Axis3D.Z);
            return source.getM02();
         }

         @Override
         public double getM10()
         {
            notifyAccessListener(Axis3D.Y, Axis3D.X);
            return source.getM10();
         }

         @Override
         public double getM11()
         {
            notifyAccessListener(Axis3D.Y, Axis3D.Y);
            return source.getM11();
         }

         @Override
         public double getM12()
         {
            notifyAccessListener(Axis3D.Y, Axis3D.Z);
            return source.getM12();
         }

         @Override
         public double getM20()
         {
            notifyAccessListener(Axis3D.Z, Axis3D.X);
            return source.getM20();
         }

         @Override
         public double getM21()
         {
            notifyAccessListener(Axis3D.Z, Axis3D.Y);
            return source.getM21();
         }

         @Override
         public double getM22()
         {
            notifyAccessListener(Axis3D.Z, Axis3D.Z);
            return source.getM22();
         }

         private void notifyChangeListener(Axis3D row, Axis3D col, double newValue)
         {
            if (valueChangedListener == null)
               return;

            if (isNotifying)
               return;

            isNotifying = true;
            valueChangedListener.accept(row, col, newValue);
            isNotifying = false;
         }

         private void notifyAccessListener(Axis3D row, Axis3D col)
         {
            if (valueAccessedListener == null)
               return;
            if (isNotifying)
               return;

            isNotifying = true;
            valueAccessedListener.accept(row, col);
            isNotifying = false;
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
            return EuclidCoreIOTools.getMatrix3DString(this);
         }
      };
   }

   public static interface ObjObjDoubleConsumer<T, U>
   {
      void accept(T t, U u, double value);
   }
}
