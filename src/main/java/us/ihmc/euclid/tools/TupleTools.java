package us.ihmc.euclid.tools;

import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

/**
 * Tools for generic operations on tuples.
 *
 * @author Sylvain Bertrand
 */
public class TupleTools
{
   /**
    * Tolerance used in {@link Tuple2DBasics#clipToMaxNorm(double)} and
    * {@link Tuple3DBasics#clipToMaxNorm(double)}.
    */
   public static final double EPS_MAX_NORM = 1.0e-7;

   private TupleTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Convenience method to calculate the dot product of two tuples.
    *
    * @param x     the x-component of the first tuple.
    * @param y     the y-component of the first tuple.
    * @param tuple the second tuple. Not modified.
    * @return the value of the dot product.
    */
   public static double dot(double x, double y, Tuple2DReadOnly tuple)
   {
      return x * tuple.getX() + y * tuple.getY();
   }

   /**
    * Convenience method to calculate the dot product of two tuples.
    *
    * @param x     the x-component of the first tuple.
    * @param y     the y-component of the first tuple.
    * @param z     the z-component of the first tuple.
    * @param tuple the second tuple. Not modified.
    * @return the value of the dot product.
    */
   public static double dot(double x, double y, double z, Tuple3DReadOnly tuple)
   {
      return x * tuple.getX() + y * tuple.getY() + z * tuple.getZ();
   }

   /**
    * Convenience method to calculate the dot product of two tuples.
    *
    * @param x     the x-component of the first tuple.
    * @param y     the y-component of the first tuple.
    * @param z     the z-component of the first tuple.
    * @param s     the s-component of the first tuple.
    * @param tuple the second tuple. Not modified.
    * @return the value of the dot product.
    */
   public static double dot(double x, double y, double z, double s, Tuple4DReadOnly tuple)
   {
      return x * tuple.getX() + y * tuple.getY() + z * tuple.getZ() + s * tuple.getS();
   }

   /**
    * Convenience method to calculate the dot product of two tuples.
    *
    * @param tuple1 the first tuple in the dot product. Not modified.
    * @param tuple2 the second tuple in the dot product. Not modified.
    * @return the value of the dot product of the two tuples.
    */
   public static double dot(Tuple2DReadOnly tuple1, Tuple2DReadOnly tuple2)
   {
      return tuple1.getX() * tuple2.getX() + tuple1.getY() * tuple2.getY();
   }

   /**
    * Convenience method to calculate the dot product of two tuples.
    *
    * @param tuple1 the first tuple in the dot product. Not modified.
    * @param tuple2 the second tuple in the dot product. Not modified.
    * @return the value of the dot product of the two tuples.
    */
   public static double dot(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
   {
      return tuple1.getX() * tuple2.getX() + tuple1.getY() * tuple2.getY() + tuple1.getZ() * tuple2.getZ();
   }

   /**
    * Convenience method to calculate the angle in radians between two vectors given as tuples.
    *
    * @param tuple1 the first tuple. Not modified.
    * @param tuple2 the second tuple. Not modified.
    * @return the value of the angle from {@code tuple1} to {@code tuple2} in the range [0; <i>pi</i>].
    */
   public static double angle(Tuple2DReadOnly tuple1, Tuple2DReadOnly tuple2)
   {
      double x1 = tuple1.getX();
      double y1 = tuple1.getY();
      double x2 = tuple2.getX();
      double y2 = tuple2.getY();
      return angle(x1, y1, x2, y2);
   }

   /**
    * Convenience method to calculate the angle in radians between to vectors.
    *
    * @param x1 the x-component of first the vector.
    * @param y1 the y-component of first the vector.
    * @param x2 the x-component of second the vector.
    * @param y2 the y-component of second the vector.
    * @return the value of the angle from the first vector to the second vector in the range
    *         [-<i>pi</i>; <i>pi</i>].
    */
   public static double angle(double x1, double y1, double x2, double y2)
   {
      double cosTheta = x1 * x2 + y1 * y2;
      double sinTheta = x1 * y2 - y1 * x2;
      return EuclidCoreTools.atan2(sinTheta, cosTheta);
   }

   /**
    * Convenience method to calculate the angle in radians between two vectors given as tuples.
    *
    * @param tuple1 the first tuple. Not modified.
    * @param tuple2 the second tuple. Not modified.
    * @return the value of the angle from {@code tuple1} to {@code tuple2} in the range [0; <i>pi</i>].
    */
   public static double angle(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
   {
      double x1 = tuple1.getX();
      double y1 = tuple1.getY();
      double z1 = tuple1.getZ();
      double x2 = tuple2.getX();
      double y2 = tuple2.getY();
      double z2 = tuple2.getZ();
      return angle(x1, y1, z1, x2, y2, z2);
   }

   /**
    * Convenience method to calculate the angle in radians between to vectors.
    *
    * @param x1 the x-component of first the vector.
    * @param y1 the y-component of first the vector.
    * @param z1 the z-component of first the vector.
    * @param x2 the x-component of second the vector.
    * @param y2 the y-component of second the vector.
    * @param z2 the z-component of second the vector.
    * @return the value of the angle from the first vector to the second vector in the range [0;
    *         <i>pi</i>].
    */
   public static double angle(double x1, double y1, double z1, double x2, double y2, double z2)
   {
      double crossX = y1 * z2 - z1 * y2;
      double crossY = z1 * x2 - x1 * z2;
      double crossZ = x1 * y2 - y1 * x2;

      double cosTheta = x1 * x2 + y1 * y2 + z1 * z2;
      double sinTheta = EuclidCoreTools.norm(crossX, crossY, crossZ);
      return EuclidCoreTools.atan2(sinTheta, cosTheta);
   }

   /**
    * Tests on a per component basis if the two given tuples are equal to an {@code epsilon}.
    *
    * @param tuple1  the first tuple. Not modified.
    * @param tuple2  the second tuple. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two tuples are equal, {@code false} otherwise.
    */
   public static boolean epsilonEquals(Tuple2DReadOnly tuple1, Tuple2DReadOnly tuple2, double epsilon)
   {
      if (!EuclidCoreTools.epsilonEquals(tuple1.getX(), tuple2.getX(), epsilon))
         return false;

      if (!EuclidCoreTools.epsilonEquals(tuple1.getY(), tuple2.getY(), epsilon))
         return false;

      return true;
   }

   /**
    * Tests on a per component basis if the two given tuples are equal to an {@code epsilon}.
    *
    * @param tuple1  the first tuple. Not modified.
    * @param tuple2  the second tuple. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two tuples are equal, {@code false} otherwise.
    */
   public static boolean epsilonEquals(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2, double epsilon)
   {
      if (!EuclidCoreTools.epsilonEquals(tuple1.getX(), tuple2.getX(), epsilon))
         return false;

      if (!EuclidCoreTools.epsilonEquals(tuple1.getY(), tuple2.getY(), epsilon))
         return false;

      if (!EuclidCoreTools.epsilonEquals(tuple1.getZ(), tuple2.getZ(), epsilon))
         return false;

      return true;
   }

   /**
    * Tests on a per component basis if the two given tuples are equal to an {@code epsilon}.
    *
    * @param tuple1  the first tuple. Not modified.
    * @param tuple2  the second tuple. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two tuples are equal, {@code false} otherwise.
    */
   public static boolean epsilonEquals(Tuple4DReadOnly tuple1, Tuple4DReadOnly tuple2, double epsilon)
   {
      if (!EuclidCoreTools.epsilonEquals(tuple1.getX(), tuple2.getX(), epsilon))
         return false;

      if (!EuclidCoreTools.epsilonEquals(tuple1.getY(), tuple2.getY(), epsilon))
         return false;

      if (!EuclidCoreTools.epsilonEquals(tuple1.getZ(), tuple2.getZ(), epsilon))
         return false;

      if (!EuclidCoreTools.epsilonEquals(tuple1.getS(), tuple2.getS(), epsilon))
         return false;

      return true;
   }

   /**
    * Tests whether all the tuple's components are equal to zero to an {@code epsilon}.
    *
    * @param tuple   the query. Not modified.
    * @param epsilon the tolerance to use for the test.
    * @return {@code true} if all the tuple's components are considered to be equal to zero,
    *         {@code false} otherwise.
    */
   public static boolean isTupleZero(Tuple2DReadOnly tuple, double epsilon)
   {
      return EuclidCoreTools.areAllZero(tuple.getX(), tuple.getY(), epsilon);
   }

   /**
    * Tests whether all the tuple's components are equal to zero to an {@code epsilon}.
    *
    * @param tuple   the query. Not modified.
    * @param epsilon the tolerance to use for the test.
    * @return {@code true} if all the tuple's components are considered to be equal to zero,
    *         {@code false} otherwise.
    */
   public static boolean isTupleZero(Tuple3DReadOnly tuple, double epsilon)
   {
      return EuclidCoreTools.areAllZero(tuple.getX(), tuple.getY(), tuple.getZ(), epsilon);
   }

   /**
    * Tests whether all the tuple's components are equal to zero to an {@code epsilon}.
    *
    * @param tuple   the query. Not modified.
    * @param epsilon the tolerance to use for the test.
    * @return {@code true} if all the tuple's components are considered to be equal to zero,
    *         {@code false} otherwise.
    */
   public static boolean isTupleZero(Tuple4DReadOnly tuple, double epsilon)
   {
      return EuclidCoreTools.areAllZero(tuple.getX(), tuple.getY(), tuple.getZ(), tuple.getS(), epsilon);
   }
}
