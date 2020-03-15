package us.ihmc.euclid.tools;

import java.util.function.DoubleSupplier;

import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
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
    * Creates a new point 2D that is a read-only view of the three coordinate suppliers.
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
               return Point2DReadOnly.super.equals((Point2DReadOnly) object);
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
    * Creates a new vector 2D that is a read-only view of the three component suppliers.
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
               return Vector2DReadOnly.super.equals((Vector2DReadOnly) object);
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
    * Creates a new point 3D that is a read-only view of the three coordinate suppliers.
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
               return Point3DReadOnly.super.equals((Point3DReadOnly) object);
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
    * Creates a new vector 3D that is a read-only view of the three component suppliers.
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
               return Vector3DReadOnly.super.equals((Vector3DReadOnly) object);
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
    * Creates a new matrix 3D that is a read-only view of the transpose of the given {@code original}.
    *
    * @param original the original matrix to create linked transpose matrix for. Not modified.
    * @return the transpose read-only view of {@code original}.
    */
   public static Matrix3DReadOnly newTransposeLinkedMatrix3DReadOnly(Matrix3DBasics original)
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
         public void transform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
         {
            Matrix3DTools.transform(this, matrixOriginal, matrixTransformed);
         }

         @Override
         public void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
         {
            Matrix3DTools.inverseTransform(this, matrixOriginal, matrixTransformed);
         }

         @Override
         public void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
         {
            Matrix3DTools.inverseTransform(this, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
         }

         @Override
         public void inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
         {
            Matrix3DTools.inverseTransform(this, tupleOriginal, tupleTransformed);
         }

         @Override
         public void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
         {
            Matrix3DTools.inverseTransform(this, vectorOriginal, vectorTransformed);
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
               return Matrix3DReadOnly.super.equals((Matrix3DReadOnly) object);
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
}
