package us.ihmc.euclid.tools;

import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

/**
 * Tools for generic operations on tuples.
 * 
 * @author Sylvain Bertrand
 */
public abstract class TupleTools
{
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
