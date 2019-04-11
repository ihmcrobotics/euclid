package us.ihmc.euclid.tools;

import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

/**
 * Tools for generic operations on tuples.
 * 
 * @author Sylvain Bertrand
 */
public class TupleTools
{
   private TupleTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Convenience method to calculate the dot product of two tuples.
    * 
    * @param x the x-component of the first tuple.
    * @param y the y-component of the first tuple.
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
    * @param x the x-component of the first tuple.
    * @param y the y-component of the first tuple.
    * @param z the z-component of the first tuple.
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
    * @param x the x-component of the first tuple.
    * @param y the y-component of the first tuple.
    * @param z the z-component of the first tuple.
    * @param s the s-component of the first tuple.
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
    * Tests on a per component basis if the two given tuples are equal to an {@code epsilon}.
    *
    * @param tuple1 the first tuple. Not modified.
    * @param tuple2 the second tuple. Not modified.
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
    * @param tuple1 the first tuple. Not modified.
    * @param tuple2 the second tuple. Not modified.
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
    * @param tuple1 the first tuple. Not modified.
    * @param tuple2 the second tuple. Not modified.
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
    * @param tuple the query. Not modified.
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
    * @param tuple the query. Not modified.
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
    * @param tuple the query. Not modified.
    * @param epsilon the tolerance to use for the test.
    * @return {@code true} if all the tuple's components are considered to be equal to zero,
    *         {@code false} otherwise.
    */
   public static boolean isTupleZero(Tuple4DReadOnly tuple, double epsilon)
   {
      return EuclidCoreTools.areAllZero(tuple.getX(), tuple.getY(), tuple.getZ(), tuple.getS(), epsilon);
   }
}
