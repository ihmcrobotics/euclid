package us.ihmc.euclid.tools;

import org.ejml.MatrixDimensionException;
import org.ejml.data.Matrix;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * This class provides a variety of generic tools such as fast square-root algorithm
 * {@link #fastSquareRoot(double)}, or also an linear interpolation algorithm
 * {@link #interpolate(double, double, double)}.
 *
 * @author Sylvain Bertrand
 */
public class EuclidCoreTools
{
   /**
    * This field is used to save unnecessary computation and represents the number
    * {@code 2.0 * Math.PI}.
    */
   public static final double TwoPI = 2.0 * Math.PI;

   /**
    * Magic number used as a tolerance for switching to the fast square-root formula.
    */
   public static final double EPS_NORM_FAST_SQRT = 2.107342e-08;
   /**
    * An epsilon that is just slightly bigger than numerical inaccuracies used in
    * {@link #shiftAngleInRange(double, double)}.
    */
   public static final double EPS_ANGLE_SHIFT = 1.0e-12;
   /**
    * Tolerance used in {@link #clamp(double, double, double)} to verify the bounds are sane.
    */
   public static final double CLAMP_EPS = 1.0e-10;

   /**
    * Constant representing the coordinates (0, 0) of the origin in the 2D plane.
    */
   public static final Point2DReadOnly origin2D = new Point2DReadOnly()
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
         long hashCode = 1L;
         hashCode = EuclidHashCodeTools.addToHashCode(hashCode, 0.0);
         hashCode = EuclidHashCodeTools.addToHashCode(hashCode, 0.0);
         return EuclidHashCodeTools.toIntHashCode(hashCode);
      }

      @Override
      public String toString()
      {
         return EuclidCoreIOTools.getTuple2DString(this);
      }
   };

   /**
    * Constant representing the coordinates (0, 0, 0) of the origin in 3D.
    */
   public static final Point3DReadOnly origin3D = new Point3DReadOnly()
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
         long hashCode = 1L;
         hashCode = EuclidHashCodeTools.addToHashCode(hashCode, 0.0);
         hashCode = EuclidHashCodeTools.addToHashCode(hashCode, 0.0);
         hashCode = EuclidHashCodeTools.addToHashCode(hashCode, 0.0);
         return EuclidHashCodeTools.toIntHashCode(hashCode);
      }

      @Override
      public String toString()
      {
         return EuclidCoreIOTools.getTuple3DString(this);
      }
   };

   /**
    * Constant representing the zero vector 2D: (0, 0).
    */
   public static final Vector2DReadOnly zeroVector2D = new Vector2DReadOnly()
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
         long hashCode = 1L;
         hashCode = EuclidHashCodeTools.addToHashCode(hashCode, 0.0);
         hashCode = EuclidHashCodeTools.addToHashCode(hashCode, 0.0);
         return EuclidHashCodeTools.toIntHashCode(hashCode);
      }

      @Override
      public String toString()
      {
         return EuclidCoreIOTools.getTuple2DString(this);
      }
   };

   /**
    * Constant representing the zero vector 3D: (0, 0, 0).
    */
   public static final Vector3DReadOnly zeroVector3D = new Vector3DReadOnly()
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
         long hashCode = 1L;
         hashCode = EuclidHashCodeTools.addToHashCode(hashCode, 0.0);
         hashCode = EuclidHashCodeTools.addToHashCode(hashCode, 0.0);
         hashCode = EuclidHashCodeTools.addToHashCode(hashCode, 0.0);
         return EuclidHashCodeTools.toIntHashCode(hashCode);
      }

      @Override
      public String toString()
      {
         return EuclidCoreIOTools.getTuple3DString(this);
      }
   };

   /** Constant representing the zero-matrix 3D, i.e. the 9 elements are zero. */
   public static final Matrix3DReadOnly zeroMatrix3D = new Matrix3DReadOnly()
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

      @Override
      public int hashCode()
      {
         return EuclidHashCodeTools.toIntHashCode(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22());
      }
   };

   /**
    * Constant representing the identity matrix 3D, i.e. the 3 diagonal elements are equal to one and
    * the other equal to zero.
    */
   public static final Matrix3DReadOnly identityMatrix3D = new Matrix3DReadOnly()
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

      @Override
      public int hashCode()
      {
         return EuclidHashCodeTools.toIntHashCode(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22());
      }
   };

   private EuclidCoreTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Calculates and returns the square root of the given value.
    *
    * @param value the value to calculate the square root of.
    * @return the value of the square root.
    * @see Math#sqrt(double)
    */
   public static double squareRoot(double value)
   {
      return Math.sqrt(value);
   }

   /**
    * Calculates and returns the square root of the given value.
    * <p>
    * This method is optimized when {@code squaredValueClosedToOne} is equal to
    * 1+/-{@value #EPS_NORM_FAST_SQRT} by using an approximation of the square root.
    *
    * @param squaredValueClosedToOne the value to calculate the square root of.
    * @return the value of the square root.
    */
   public static double fastSquareRoot(double squaredValueClosedToOne)
   {
      if (Math.abs(1.0 - squaredValueClosedToOne) < EPS_NORM_FAST_SQRT)
         squaredValueClosedToOne = 0.5 * (1.0 + squaredValueClosedToOne);
      else
         squaredValueClosedToOne = EuclidCoreTools.squareRoot(squaredValueClosedToOne);

      return squaredValueClosedToOne;
   }

   /**
    * Tests if at least one of the two given elements is equal to {@linkplain Double#NaN}.
    *
    * @param a the first element.
    * @param b the second element.
    * @return {@code true} if at least one element is equal to {@link Double#NaN}, {@code false}
    *         otherwise.
    */
   public static boolean containsNaN(double a, double b)
   {
      return Double.isNaN(a) || Double.isNaN(b);
   }

   /**
    * Tests if at least one of the three given elements is equal to {@linkplain Double#NaN}.
    *
    * @param a the first element.
    * @param b the second element.
    * @param c the third element.
    * @return {@code true} if at least one element is equal to {@link Double#NaN}, {@code false}
    *         otherwise.
    */
   public static boolean containsNaN(double a, double b, double c)
   {
      return Double.isNaN(a) || Double.isNaN(b) || Double.isNaN(c);
   }

   /**
    * Tests if at least one of the four given elements is equal to {@linkplain Double#NaN}.
    *
    * @param a the first element.
    * @param b the second element.
    * @param c the third element.
    * @param d the fourth element.
    * @return {@code true} if at least one element is equal to {@link Double#NaN}, {@code false}
    *         otherwise.
    */
   public static boolean containsNaN(double a, double b, double c, double d)
   {
      return Double.isNaN(a) || Double.isNaN(b) || Double.isNaN(c) || Double.isNaN(d);
   }

   /**
    * Tests if at least one of the nine given elements is equal to {@linkplain Double#NaN}.
    *
    * @param a0 the first element.
    * @param a1 the second element.
    * @param a2 the third element.
    * @param a3 the fourth element.
    * @param a4 the fifth element.
    * @param a5 the sixth element.
    * @param a6 the seventh element.
    * @param a7 the eighth element.
    * @param a8 the ninth element.
    * @return {@code true} if at least one element is equal to {@link Double#NaN}, {@code false}
    *         otherwise.
    */
   public static boolean containsNaN(double a0, double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8)
   {
      if (Double.isNaN(a0) || Double.isNaN(a1) || Double.isNaN(a2))
         return true;
      if (Double.isNaN(a3) || Double.isNaN(a4) || Double.isNaN(a5))
         return true;
      if (Double.isNaN(a6) || Double.isNaN(a7) || Double.isNaN(a8))
         return true;
      return false;
   }

   /**
    * Tests if at least one element in the given array is equal to {@link Double#NaN}.
    *
    * @param array the array containing the elements to test for {@link Double#NaN}. Not modified.
    * @return {@code true} if at least one element is equal to {@link Double#NaN}, {@code false}
    *         otherwise.
    */
   public static boolean containsNaN(double[] array)
   {
      for (int i = 0; i < array.length; i++)
      {
         if (Double.isNaN(array[i]))
            return true;
      }
      return false;
   }

   /**
    * Calculates and returns {@code value}<sup>2</sup>
    *
    * @param value the value to compute the square of.
    * @return {@code value * value}.
    */
   public static double square(double value)
   {
      return value * value;
   }

   /**
    * Calculates and returns the norm squared of the given two elements.
    * <p>
    * norm<sup>2</sup> = x<sup>2</sup> + y<sup>2</sup>
    * </p>
    *
    * @param x the first element.
    * @param y the second element.
    * @return the value of the square of the norm.
    */
   public static double normSquared(double x, double y)
   {
      return x * x + y * y;
   }

   /**
    * Calculates and returns the norm squared of the given three elements.
    * <p>
    * norm<sup>2</sup> = x<sup>2</sup> + y<sup>2</sup> + z<sup>2</sup>
    * </p>
    *
    * @param x the first element.
    * @param y the second element.
    * @param z the third element.
    * @return the value of the square of the norm.
    */
   public static double normSquared(double x, double y, double z)
   {
      return x * x + y * y + z * z;
   }

   /**
    * Calculates and returns the norm squared of the given four elements.
    * <p>
    * norm<sup>2</sup> = x<sup>2</sup> + y<sup>2</sup> + z<sup>2</sup> + s<sup>2</sup>
    * </p>
    *
    * @param x the first element.
    * @param y the second element.
    * @param z the third element.
    * @param s the fourth element.
    * @return the value of the square of the norm.
    */
   public static double normSquared(double x, double y, double z, double s)
   {
      return x * x + y * y + z * z + s * s;
   }

   /**
    * Calculates and returns the norm of the given two elements.
    * <p>
    * norm = &radic;(x<sup>2</sup> + y<sup>2</sup>)
    * </p>
    *
    * @param x the first element.
    * @param y the second element.
    * @return the value of the norm.
    */
   public static double norm(double x, double y)
   {
      return squareRoot(normSquared(x, y));
   }

   /**
    * Calculates and returns the norm of the given three elements.
    * <p>
    * norm = &radic;(x<sup>2</sup> + y<sup>2</sup> + z<sup>2</sup>)
    * </p>
    *
    * @param x the first element.
    * @param y the second element.
    * @param z the third element.
    * @return the value of the norm.
    */
   public static double norm(double x, double y, double z)
   {
      return squareRoot(normSquared(x, y, z));
   }

   /**
    * Calculates and returns the norm of the given four elements.
    * <p>
    * norm = &radic;(x<sup>2</sup> + y<sup>2</sup> + z<sup>2</sup> + s<sup>2</sup>)
    * </p>
    *
    * @param x the first element.
    * @param y the second element.
    * @param z the third element.
    * @param s the fourth element.
    * @return the value of the norm.
    */
   public static double norm(double x, double y, double z, double s)
   {
      return squareRoot(normSquared(x, y, z, s));
   }

   /**
    * Calculates and returns the norm of the given two elements.
    * <p>
    * norm = &radic;(x<sup>2</sup> + y<sup>2</sup>)
    * </p>
    * <p>
    * This method is optimized for calculating norms closed to 1 by using
    * {@link #fastSquareRoot(double)}. For computing norms that are not closed to 1, prefer using
    * {@link #norm(double, double)}.
    * </p>
    *
    * @param x the first element.
    * @param y the second element.
    * @return the value of the norm.
    */
   public static double fastNorm(double x, double y)
   {
      return fastSquareRoot(normSquared(x, y));
   }

   /**
    * Calculates and returns the norm of the given three elements.
    * <p>
    * norm = &radic;(x<sup>2</sup> + y<sup>2</sup> + z<sup>2</sup>)
    * </p>
    * <p>
    * This method is optimized for calculating norms closed to 1 by using
    * {@link #fastSquareRoot(double)}. For computing norms that are not closed to 1, prefer using
    * {@link #norm(double, double, double)}.
    * </p>
    *
    * @param x the first element.
    * @param y the second element.
    * @param z the third element.
    * @return the value of the norm.
    */
   public static double fastNorm(double x, double y, double z)
   {
      return fastSquareRoot(normSquared(x, y, z));
   }

   /**
    * Calculates and returns the norm of the given four elements.
    * <p>
    * norm = &radic;(x<sup>2</sup> + y<sup>2</sup> + z<sup>2</sup> + s<sup>2</sup>)
    * </p>
    * <p>
    * This method is optimized for calculating norms closed to 1 by using
    * {@link #fastSquareRoot(double)}. For computing norms that are not closed to 1, prefer using
    * {@link #norm(double, double, double, double)}.
    * </p>
    *
    * @param x the first element.
    * @param y the second element.
    * @param z the third element.
    * @param s the fourth element.
    * @return the value of the norm.
    */
   public static double fastNorm(double x, double y, double z, double s)
   {
      return fastSquareRoot(normSquared(x, y, z, s));
   }

   /**
    * Recomputes the angle value {@code angleToShift} such that the result is in [ -<i>pi</i>,
    * <i>pi</i> [ and still represent the same physical angle as {@code angleToShift}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code Math.abs(angleToShift + Math.PI) <} {@link #EPS_ANGLE_SHIFT}, the returned angle is
    * {@code -Math.PI}.
    * <li>if {@code Math.abs(angleToShift - Math.PI) <} {@link #EPS_ANGLE_SHIFT}, the returned angle is
    * {@code -Math.PI}.
    * </ul>
    * </p>
    *
    * @param angleToShift the angle to shift.
    * @return the result that is in [ -<i>pi</i>, <i>pi</i> [
    */
   public static double trimAngleMinusPiToPi(double angleToShift)
   {
      return shiftAngleInRange(angleToShift, -Math.PI);
   }

   /**
    * Computes the angle difference:<br>
    * {@code difference = angleA - angleB}<br>
    * and shift the result to be contained in [ -<i>pi</i>, <i>pi</i> [.
    *
    * @param angleA the first angle in the difference.
    * @param angleB the second angle in the difference.
    * @return the result of the subtraction contained in [ -<i>pi</i>, <i>pi</i> [.
    */
   public static double angleDifferenceMinusPiToPi(double angleA, double angleB)
   {
      return trimAngleMinusPiToPi(angleA - angleB);
   }

   /**
    * Recomputes the angle value {@code angleToShift} such that the result is in [{@code angleStart},
    * {@code angleStart} + 2<i>pi</i>[ and still represent the same physical angle as
    * {@code angleToShift}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code Math.abs(angleToShift - angleStart) <} {@link #EPS_ANGLE_SHIFT}, the returned angle
    * is {@code angleStart}.
    * <li>if {@code Math.abs(angleToShift - angleStart + 2.0 * Math.PI) <} {@link #EPS_ANGLE_SHIFT},
    * the returned angle is {@code angleStart}.
    * </ul>
    * </p>
    *
    * @param angleToShift the angle to shift.
    * @param angleStart   the lowest admissible angle value.
    * @return the result that is in [{@code angleStart}, {@code angleStart} + 2<i>pi</i>[
    */
   public static double shiftAngleInRange(double angleToShift, double angleStart)
   {
      angleStart = angleStart - EPS_ANGLE_SHIFT;

      double deltaFromStart = (angleToShift - angleStart) % TwoPI;

      if (deltaFromStart < 0)
         deltaFromStart += TwoPI;

      return angleStart + deltaFromStart;
   }

   /**
    * Find and return the argument with the maximum value.
    *
    * @param a the first argument to compare.
    * @param b the second argument to compare.
    * @param c the third argument to compare.
    * @return the maximum value of the three arguments.
    */
   public static final double max(double a, double b, double c)
   {
      if (a > b)
         return a > c ? a : c;
      else
         return b > c ? b : c;
   }

   /**
    * Find and return the argument with the maximum value.
    *
    * @param a the first argument to compare.
    * @param b the second argument to compare.
    * @param c the third argument to compare.
    * @param d the fourth argument to compare.
    * @return the maximum value of the four arguments.
    */
   public static final double max(double a, double b, double c, double d)
   {
      if (a > b)
      {
         if (a > c)
            return a > d ? a : d;
         else
            return c > d ? c : d;
      }
      else
      {
         if (b > c)
            return b > d ? b : d;
         else
            return c > d ? c : d;
      }
   }

   /**
    * Find and return the argument with the minimum value.
    *
    * @param a the first argument to compare.
    * @param b the second argument to compare.
    * @param c the third argument to compare.
    * @return the minimum value of the three arguments.
    */
   public static final double min(double a, double b, double c)
   {
      if (a < b)
         return a < c ? a : c;
      else
         return b < c ? b : c;
   }

   /**
    * Find and return the argument with the minimum value.
    *
    * @param a the first argument to compare.
    * @param b the second argument to compare.
    * @param c the third argument to compare.
    * @param d the fourth argument to compare.
    * @return the minimum value of the four arguments.
    */
   public static final double min(double a, double b, double c, double d)
   {
      if (a < b)
      {
         if (a < c)
            return a < d ? a : d;
         else
            return c < d ? c : d;
      }
      else
      {
         if (b < c)
            return b < d ? b : d;
         else
            return c < d ? c : d;
      }
   }

   /**
    * Find and return the argument with the value in between the two others.
    *
    * @param a the first argument to compare.
    * @param b the second argument to compare.
    * @param c the third argument to compare.
    * @return the value in between the two other arguments.
    */
   public static final double med(double a, double b, double c)
   {
      if (a > b)
      {
         if (a > c)
            return b > c ? b : c;
         else
            return a;
      }
      else
      {
         if (b > c)
            return a > c ? a : c;
         else
            return b;
      }
   }

   /**
    * Tests if the two values are equal to an {@code epsilon}:<br>
    * |{@code expectedValue} - {@code actualValue}| &leq; {@code epsilon}
    * <p>
    * If any of the two values is equal to {@link Double#NaN}, this method fails and returns
    * {@code false}.
    * </p>
    *
    * @param expectedValue the first value to compare.
    * @param actualValue   the second value to compare.
    * @param epsilon       the tolerance to use for the test.
    * @return {@code true} if the two values are considered to be equal, {@code false otherwise}.
    */
   public static boolean epsilonEquals(double expectedValue, double actualValue, double epsilon)
   {
      return Math.abs(expectedValue - actualValue) <= epsilon;
   }

   /**
    * Tests whether the value is equal to zero to an epsilon.
    * <p>
    * This is equivalent to {@code epsilonEquals(value, 0.0, epsilon)}.
    * </p>
    *
    * @param value   the query.
    * @param epsilon the tolerance to use for the test.
    * @return {@code true} if the value can be considered zero, {@code false} otherwise.
    */
   public static boolean isZero(double value, double epsilon)
   {
      return Math.abs(value) <= epsilon;
   }

   /**
    * Tests whether both values are equal to zero to an epsilon.
    *
    * @param x       the first value.
    * @param y       the second value.
    * @param epsilon the tolerance to use for the test.
    * @return {@code true} if both values can be considered zero, {@code false} otherwise.
    */
   public static boolean areAllZero(double x, double y, double epsilon)
   {
      return isZero(x, epsilon) && isZero(y, epsilon);
   }

   /**
    * Tests whether all values are equal to zero to an epsilon.
    *
    * @param x       the first value.
    * @param y       the second value.
    * @param z       the third value.
    * @param epsilon the tolerance to use for the test.
    * @return {@code true} if all values can be considered zero, {@code false} otherwise.
    */
   public static boolean areAllZero(double x, double y, double z, double epsilon)
   {
      return isZero(x, epsilon) && isZero(y, epsilon) && isZero(z, epsilon);
   }

   /**
    * Tests whether all values are equal to zero to an epsilon.
    *
    * @param x       the first value.
    * @param y       the second value.
    * @param z       the third value.
    * @param s       the fourth value.
    * @param epsilon the tolerance to use for the test.
    * @return {@code true} if all values can be considered zero, {@code false} otherwise.
    */
   public static boolean areAllZero(double x, double y, double z, double s, double epsilon)
   {
      return isZero(x, epsilon) && isZero(y, epsilon) && isZero(z, epsilon) && isZero(s, epsilon);
   }

   /**
    * Tests whether the to angles represent the same physical angle to an {@code epsilon}.
    * <p>
    * If any of the two angles is equal to {@link Double#NaN}, this method fails and returns
    * {@code false}.
    * </p>
    *
    * @param expectedAngle the first angle in the comparison.
    * @param actualAngle   the second angle in the comparison.
    * @param epsilon       the tolerance to use for the test.
    * @return {@code true} if the two angles are considered to be geometrically equal, {@code false}
    *         otherwise.
    */
   public static boolean angleGeometricallyEquals(double expectedAngle, double actualAngle, double epsilon)
   {
      return Math.abs(angleDifferenceMinusPiToPi(expectedAngle, actualAngle)) <= epsilon;
   }

   /**
    * Tests whether the angle is geometrically equal to the zero angle to an epsilon.
    * <p>
    * This is equivalent to {@code angleGeometricallyEquals(angle, 0.0, epsilon)}.
    * </p>
    *
    * @param angle   the query.
    * @param epsilon the tolerance to use for the test.
    * @return {@code true} if the angle can be considered zero, {@code false} otherwise.
    */
   public static boolean isAngleZero(double angle, double epsilon)
   {
      angle = Math.abs(angle) % TwoPI;
      if (angle > Math.PI)
         angle -= TwoPI;
      return Math.abs(angle) <= epsilon;
   }

   /**
    * Clamps value to the given range, defined by {@code -minMax} and {@code minMax}, inclusive.
    *
    * @param value  value
    * @param minMax inclusive absolute boundary
    * @return
    *         <li>{@code -minMax} if {@code value} is less than {@code -minMax}</li>
    *         <li>{@code minMax} if {@code value} is greater than {@code minMax}</li>
    *         <li>{@code value} if {@code value} is between or equal to {@code -minMax} and
    *         {@code minMax}</li>
    */
   public static double clamp(double value, double minMax)
   {
      return clamp(value, -minMax, minMax);
   }

   /**
    * Clamps value to the given range, inclusive.
    *
    * @param value value
    * @param min   inclusive boundary start
    * @param max   inclusive boundary end
    * @return
    *         <li>{@code min} if {@code value} is less than {@code min}</li>
    *         <li>{@code max} if {@code value} is greater than {@code max}</li>
    *         <li>{@code value} if {@code value} is between or equal to {@code min} and
    *         {@code max}</li>
    */
   public static double clamp(double value, double min, double max)
   {
      if (min > max + CLAMP_EPS)
      {
         throw new RuntimeException(EuclidCoreTools.class.getSimpleName() + ".clamp(double, double, double): min > max (" + min + " > " + max + ")");
      }

      return Math.min(max, Math.max(value, min));
   }

   /**
    * Performs a linear interpolation from {@code a} to {@code b} given the percentage {@code alpha}.
    * <p>
    * result = (1.0 - alpha) * a + alpha * b
    * </p>
    *
    * @param a     the first value used in the interpolation.
    * @param b     the second value used in the interpolation.
    * @param alpha the percentage to use for the interpolation. A value of 0 will return {@code a},
    *              while a value of 1 will return {@code b}.
    * @return the interpolated value.
    */
   public static double interpolate(double a, double b, double alpha)
   {
      return (1.0 - alpha) * a + alpha * b;
   }

   /**
    * Returns the trigonometric tangent of an angle.
    *
    * @param a an angle, in radians.
    * @return the tangent of the argument.
    * @see Math#tan(double)
    */
   public static double tan(double a)
   {
      return StrictMath.tan(a);
   }

   /**
    * Returns the arc tangent of a value; the returned angle is in the range -<i>pi</i>/2 through
    * <i>pi</i>/2.
    *
    * @param a the value whose arc tangent is to be returned.
    * @return the arc tangent of the argument.
    * @see Math#atan(double)
    */
   public static double atan(double a)
   {
      return StrictMath.atan(a);
   }

   /**
    * Returns the angle <i>theta</i> from the conversion of rectangular coordinates
    * ({@code x},&nbsp;{@code y}) to polar coordinates (r,&nbsp;<i>theta</i>).
    *
    * @param y the ordinate coordinate
    * @param x the abscissa coordinate
    * @return the <i>theta</i> component of the point (<i>r</i>,&nbsp;<i>theta</i>) in polar
    *         coordinates that corresponds to the point (<i>x</i>,&nbsp;<i>y</i>) in Cartesian
    *         coordinates.
    * @see Math#atan2(double, double)
    */
   public static double atan2(double y, double x)
   {
      return StrictMath.atan2(y, x);
   }

   /**
    * Returns the trigonometric cosine of an angle.
    *
    * @param a an angle, in radians.
    * @return the cosine of the argument.
    * @see Math#cos(double)
    */
   public static double cos(double a)
   {
      return StrictMath.cos(a);
   }

   /**
    * Returns the trigonometric sine of an angle.
    *
    * @param a an angle, in radians.
    * @return the sine of the argument.
    * @see Math#sin(double)
    */
   public static double sin(double a)
   {
      return StrictMath.sin(a);
   }

   /**
    * Returns the arc cosine of a value; the returned angle is in the range 0.0 through <i>pi</i>.
    *
    * @param a the value whose arc cosine is to be returned.
    * @return the arc cosine of the argument.
    * @see Math#acos(double)
    */
   public static double acos(double a)
   {
      return StrictMath.acos(a);
   }

   /**
    * Returns the arc sine of a value; the returned angle is in the range -<i>pi</i>/2 through
    * <i>pi</i>/2.
    *
    * @param a the value whose arc sine is to be returned.
    * @return the arc sine of the argument.
    * @see Math#asin(double)
    */
   public static double asin(double a)
   {
      return StrictMath.asin(a);
   }

   /**
    * Checks that the given {@code matrixToTest} has a minimum size of [{@code minRows},
    * {@code minColumns}].
    *
    * @param minRows      the minimum number of rows that the matrix should have.
    * @param minColumns   the minimum number of columns that the matrix should have.
    * @param matrixToTest the matrix which size is to be tested.
    * @throws MatrixDimensionException if the matrix does not meet the minimum size.
    */
   public static void checkMatrixMinimumSize(int minRows, int minColumns, Matrix matrixToTest)
   {
      if (matrixToTest.getNumCols() < minColumns || matrixToTest.getNumRows() < minRows)
      {
         throw new MatrixDimensionException("The matrix is too small, expected: [nRows >= " + minRows + ", nColumns >= " + minColumns + "], was: [nRows = "
               + matrixToTest.getNumRows() + ", nCols = " + matrixToTest.getNumCols() + "].");
      }
   }
}
