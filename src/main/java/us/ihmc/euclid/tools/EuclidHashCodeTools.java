package us.ihmc.euclid.tools;

import java.util.Objects;

import us.ihmc.euclid.tuple3D.Point3D;

/**
 * Generic operations on hash-code used throughout this library for computing hash-codes of object
 * as {@link Point3D}.
 *
 * @author Sylvain Bertrand
 */
public class EuclidHashCodeTools
{
   /**
    * Long used for the multiplication factor in each step of the hash.
    */
   public final static long MULTIPLIER = 31L;

   private EuclidHashCodeTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Returns a hash bit stream as an integer hash value.
    *
    * @param bits the bits to turn into integer hash code.
    * @return final integer hash value.
    */
   public static int toIntHashCode(long bits)
   {
      return (int) (bits ^ bits >> 32);
   }

   public static int toIntHashCode(double a)
   {
      return toIntHashCode(toLongHashCode(a));
   }

   public static int toIntHashCode(double a0, double a1)
   {
      return toIntHashCode(toLongHashCode(a0, a1));
   }

   public static int toIntHashCode(double a0, double a1, double a2)
   {
      return toIntHashCode(toLongHashCode(a0, a1, a2));
   }

   public static int toIntHashCode(double a0, double a1, double a2, double a3)
   {
      return toIntHashCode(toLongHashCode(a0, a1, a2, a3));
   }

   public static int toIntHashCode(double a0, double a1, double a2, double a3, double a4)
   {
      return toIntHashCode(toLongHashCode(a0, a1, a2, a3, a4));
   }

   public static int toIntHashCode(double a0, double a1, double a2, double a3, double a4, double a5)
   {
      return toIntHashCode(toLongHashCode(a0, a1, a2, a3, a4, a5));
   }

   public static int toIntHashCode(double a0, double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8)
   {
      return toIntHashCode(toLongHashCode(a0, a1, a2, a3, a4, a5, a6, a7, a8));
   }

   public static int toIntHashCode(float a)
   {
      return toIntHashCode(toLongHashCode(a));
   }

   public static int toIntHashCode(float a0, float a1)
   {
      return toIntHashCode(toLongHashCode(a0, a1));
   }

   public static int toIntHashCode(float a0, float a1, float a2)
   {
      return toIntHashCode(toLongHashCode(a0, a1, a2));
   }

   public static int toIntHashCode(float a0, float a1, float a2, float a3)
   {
      return toIntHashCode(toLongHashCode(a0, a1, a2, a3));
   }

   public static long toLongHashCode(double a)
   {
      long bits = 1L;
      bits = addToHashCode(bits, a);
      return bits;
   }

   public static long toLongHashCode(double a0, double a1)
   {
      long bits = 1L;
      bits = addToHashCode(bits, a0);
      bits = addToHashCode(bits, a1);
      return bits;
   }

   public static long toLongHashCode(double a0, double a1, double a2)
   {
      long bits = 1L;
      bits = addToHashCode(bits, a0);
      bits = addToHashCode(bits, a1);
      bits = addToHashCode(bits, a2);
      return bits;
   }

   public static long toLongHashCode(double a0, double a1, double a2, double a3)
   {
      long bits = 1L;
      bits = addToHashCode(bits, a0);
      bits = addToHashCode(bits, a1);
      bits = addToHashCode(bits, a2);
      bits = addToHashCode(bits, a3);
      return bits;
   }

   public static long toLongHashCode(double a0, double a1, double a2, double a3, double a4)
   {
      long bits = 1L;
      bits = addToHashCode(bits, a0);
      bits = addToHashCode(bits, a1);
      bits = addToHashCode(bits, a2);
      bits = addToHashCode(bits, a3);
      bits = addToHashCode(bits, a4);
      return bits;
   }

   public static long toLongHashCode(double a0, double a1, double a2, double a3, double a4, double a5)
   {
      long bits = 1L;
      bits = addToHashCode(bits, a0);
      bits = addToHashCode(bits, a1);
      bits = addToHashCode(bits, a2);
      bits = addToHashCode(bits, a3);
      bits = addToHashCode(bits, a4);
      bits = addToHashCode(bits, a5);
      return bits;
   }

   public static long toLongHashCode(double a0, double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8)
   {
      long bits = 1L;
      bits = addToHashCode(bits, a0);
      bits = addToHashCode(bits, a1);
      bits = addToHashCode(bits, a2);
      bits = addToHashCode(bits, a3);
      bits = addToHashCode(bits, a4);
      bits = addToHashCode(bits, a5);
      bits = addToHashCode(bits, a6);
      bits = addToHashCode(bits, a7);
      bits = addToHashCode(bits, a8);
      return bits;
   }

   public static long toLongHashCode(float a)
   {
      long bits = 1L;
      bits = addToHashCode(bits, a);
      return bits;
   }

   public static long toLongHashCode(float a, float b)
   {
      long bits = 1L;
      bits = addToHashCode(bits, a);
      bits = addToHashCode(bits, b);
      return bits;
   }

   public static long toLongHashCode(float a, float b, float c)
   {
      long bits = 1L;
      bits = addToHashCode(bits, a);
      bits = addToHashCode(bits, b);
      bits = addToHashCode(bits, c);
      return bits;
   }

   public static long toLongHashCode(float a, float b, float c, float d)
   {
      long bits = 1L;
      bits = addToHashCode(bits, a);
      bits = addToHashCode(bits, b);
      bits = addToHashCode(bits, c);
      bits = addToHashCode(bits, d);
      return bits;
   }

   public static long toLongHashCode(float a0, float a1, float a2, float a3, float a4, float a5)
   {
      long bits = 1L;
      bits = addToHashCode(bits, a0);
      bits = addToHashCode(bits, a1);
      bits = addToHashCode(bits, a2);
      bits = addToHashCode(bits, a3);
      bits = addToHashCode(bits, a4);
      bits = addToHashCode(bits, a5);
      return bits;
   }

   /**
    * Calls {@link #combineHashCode(long, long)} after converting {@code value} to long bits.
    *
    * @param hashCode long hash code bit stream to add to.
    * @param value    double value to add to the hash code.
    * @return new hash code bit stream
    */
   public static long addToHashCode(long hashCode, double value)
   {
      return combineHashCode(hashCode, Double.doubleToLongBits(value));
   }

   /**
    * Calls {@link #combineHashCode(long, long)} after converting {@code value} to int bits.
    *
    * @param hashCode long hash code bit stream to add to.
    * @param value    float value to add to the hash code.
    * @return new hash code bit stream
    */
   public static long addToHashCode(long hashCode, float value)
   {
      return combineHashCode(hashCode, Float.floatToIntBits(value));
   }

   /**
    * Combine the two hash code bit streams.
    *
    * @param hashCode1 first hash code to combine.
    * @param hashCode2 second hash code to combine.
    * @return combined hash code, {@code hashCode1} + {@link #MULTIPLIER} * {@code hashCode2}
    */
   public static long combineHashCode(long hashCode1, long hashCode2)
   {
      return hashCode1 + MULTIPLIER * hashCode2;
   }

   public static long combineHashCode(long hashCode1, Object object)
   {
      return hashCode1 + MULTIPLIER * Objects.hashCode(object);
   }
}
