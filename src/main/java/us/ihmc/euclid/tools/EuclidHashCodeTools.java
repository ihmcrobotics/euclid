package us.ihmc.euclid.tools;

import java.util.Arrays;
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
   /** Smallest prime number used as the base for computing name based hash codes. */
   public static final long DEFAULT_HASHCODE = 1L;
   /** Default hash code for any object that is equal to {@code null}. */
   public static final long NULL_HASHCODE = 0L;

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

   /**
    * Convenience method for computing the hash-code of the given argument.
    *
    * @param a the argument to get the hash-code of.
    * @return the 32-bit hash-code.
    */
   public static int toIntHashCode(double a)
   {
      return toIntHashCode(toLongHashCode(a));
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @return the 32-bit hash-code.
    */
   public static int toIntHashCode(double a0, double a1)
   {
      return toIntHashCode(toLongHashCode(a0, a1));
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @return the 32-bit hash-code.
    */
   public static int toIntHashCode(double a0, double a1, double a2)
   {
      return toIntHashCode(toLongHashCode(a0, a1, a2));
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @param a3 the argument to use for computing a hash-code.
    * @return the 32-bit hash-code.
    */
   public static int toIntHashCode(double a0, double a1, double a2, double a3)
   {
      return toIntHashCode(toLongHashCode(a0, a1, a2, a3));
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @param a3 the argument to use for computing a hash-code.
    * @param a4 the argument to use for computing a hash-code.
    * @return the 32-bit hash-code.
    */
   public static int toIntHashCode(double a0, double a1, double a2, double a3, double a4)
   {
      return toIntHashCode(toLongHashCode(a0, a1, a2, a3, a4));
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @param a3 the argument to use for computing a hash-code.
    * @param a4 the argument to use for computing a hash-code.
    * @param a5 the argument to use for computing a hash-code.
    * @return the 32-bit hash-code.
    */
   public static int toIntHashCode(double a0, double a1, double a2, double a3, double a4, double a5)
   {
      return toIntHashCode(toLongHashCode(a0, a1, a2, a3, a4, a5));
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @param a3 the argument to use for computing a hash-code.
    * @param a4 the argument to use for computing a hash-code.
    * @param a5 the argument to use for computing a hash-code.
    * @param a6 the argument to use for computing a hash-code.
    * @param a7 the argument to use for computing a hash-code.
    * @param a8 the argument to use for computing a hash-code.
    * @return the 32-bit hash-code.
    */
   public static int toIntHashCode(double a0, double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8)
   {
      return toIntHashCode(toLongHashCode(a0, a1, a2, a3, a4, a5, a6, a7, a8));
   }

   /**
    * Convenience method for computing the hash-code of the given argument.
    *
    * @param a the argument to get the hash-code of.
    * @return the 32-bit hash-code.
    */
   public static int toIntHashCode(float a)
   {
      return toIntHashCode(toLongHashCode(a));
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @return the 32-bit hash-code.
    */
   public static int toIntHashCode(float a0, float a1)
   {
      return toIntHashCode(toLongHashCode(a0, a1));
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @return the 32-bit hash-code.
    */
   public static int toIntHashCode(float a0, float a1, float a2)
   {
      return toIntHashCode(toLongHashCode(a0, a1, a2));
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @param a3 the argument to use for computing a hash-code.
    * @return the 32-bit hash-code.
    */
   public static int toIntHashCode(float a0, float a1, float a2, float a3)
   {
      return toIntHashCode(toLongHashCode(a0, a1, a2, a3));
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @return the 32-bit hash-code.
    */
   public static int toIntHashCode(Object a0, Object a1)
   {
      return toIntHashCode(toLongHashCode(a0, a1));
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @return the 32-bit hash-code.
    */
   public static int toIntHashCode(Object a0, Object a1, Object a2)
   {
      return toIntHashCode(toLongHashCode(a0, a1, a2));
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @param a3 the argument to use for computing a hash-code.
    * @return the 32-bit hash-code.
    */
   public static int toIntHashCode(Object a0, Object a1, Object a2, Object a3)
   {
      return toIntHashCode(toLongHashCode(a0, a1, a2, a3));
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @param a3 the argument to use for computing a hash-code.
    * @param a4 the argument to use for computing a hash-code.
    * @return the 32-bit hash-code.
    */
   public static int toIntHashCode(Object a0, Object a1, Object a2, Object a3, Object a4)
   {
      return toIntHashCode(toLongHashCode(a0, a1, a2, a3, a4));
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @param a3 the argument to use for computing a hash-code.
    * @param a4 the argument to use for computing a hash-code.
    * @param a5 the argument to use for computing a hash-code.
    * @return the 32-bit hash-code.
    */
   public static int toIntHashCode(Object a0, Object a1, Object a2, Object a3, Object a4, Object a5)
   {
      return toIntHashCode(toLongHashCode(a0, a1, a2, a3, a4, a5));
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @param a3 the argument to use for computing a hash-code.
    * @param a4 the argument to use for computing a hash-code.
    * @param a5 the argument to use for computing a hash-code.
    * @param a6 the argument to use for computing a hash-code.
    * @return the 32-bit hash-code.
    */
   public static int toIntHashCode(Object a0, Object a1, Object a2, Object a3, Object a4, Object a5, Object a6)
   {
      return toIntHashCode(toLongHashCode(a0, a1, a2, a3, a4, a5, a6));
   }

   /**
    * Convenience method for computing the hash-code of the given argument.
    *
    * @param a the argument to get the hash-code of.
    * @return the 64-bit hash-code.
    */
   public static long toLongHashCode(double a)
   {
      long bits = DEFAULT_HASHCODE;
      bits = addToHashCode(bits, a);
      return bits;
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @return the 64-bit hash-code.
    */
   public static long toLongHashCode(double a0, double a1)
   {
      long bits = DEFAULT_HASHCODE;
      bits = addToHashCode(bits, a0);
      bits = addToHashCode(bits, a1);
      return bits;
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @return the 64-bit hash-code.
    */
   public static long toLongHashCode(double a0, double a1, double a2)
   {
      long bits = DEFAULT_HASHCODE;
      bits = addToHashCode(bits, a0);
      bits = addToHashCode(bits, a1);
      bits = addToHashCode(bits, a2);
      return bits;
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @param a3 the argument to use for computing a hash-code.
    * @return the 64-bit hash-code.
    */
   public static long toLongHashCode(double a0, double a1, double a2, double a3)
   {
      long bits = DEFAULT_HASHCODE;
      bits = addToHashCode(bits, a0);
      bits = addToHashCode(bits, a1);
      bits = addToHashCode(bits, a2);
      bits = addToHashCode(bits, a3);
      return bits;
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @param a3 the argument to use for computing a hash-code.
    * @param a4 the argument to use for computing a hash-code.
    * @return the 64-bit hash-code.
    */
   public static long toLongHashCode(double a0, double a1, double a2, double a3, double a4)
   {
      long bits = DEFAULT_HASHCODE;
      bits = addToHashCode(bits, a0);
      bits = addToHashCode(bits, a1);
      bits = addToHashCode(bits, a2);
      bits = addToHashCode(bits, a3);
      bits = addToHashCode(bits, a4);
      return bits;
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @param a3 the argument to use for computing a hash-code.
    * @param a4 the argument to use for computing a hash-code.
    * @param a5 the argument to use for computing a hash-code.
    * @return the 64-bit hash-code.
    */
   public static long toLongHashCode(double a0, double a1, double a2, double a3, double a4, double a5)
   {
      long bits = DEFAULT_HASHCODE;
      bits = addToHashCode(bits, a0);
      bits = addToHashCode(bits, a1);
      bits = addToHashCode(bits, a2);
      bits = addToHashCode(bits, a3);
      bits = addToHashCode(bits, a4);
      bits = addToHashCode(bits, a5);
      return bits;
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @param a3 the argument to use for computing a hash-code.
    * @param a4 the argument to use for computing a hash-code.
    * @param a5 the argument to use for computing a hash-code.
    * @param a6 the argument to use for computing a hash-code.
    * @param a7 the argument to use for computing a hash-code.
    * @param a8 the argument to use for computing a hash-code.
    * @return the 64-bit hash-code.
    */
   public static long toLongHashCode(double a0, double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8)
   {
      long bits = DEFAULT_HASHCODE;
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

   /**
    * Convenience method for computing the hash-code of the given argument.
    *
    * @param a the argument to get the hash-code of.
    * @return the 64-bit hash-code.
    */
   public static long toLongHashCode(float a)
   {
      long bits = DEFAULT_HASHCODE;
      bits = addToHashCode(bits, a);
      return bits;
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @return the 64-bit hash-code.
    */
   public static long toLongHashCode(float a0, float a1)
   {
      long bits = DEFAULT_HASHCODE;
      bits = addToHashCode(bits, a0);
      bits = addToHashCode(bits, a1);
      return bits;
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @return the 64-bit hash-code.
    */
   public static long toLongHashCode(float a0, float a1, float a2)
   {
      long bits = DEFAULT_HASHCODE;
      bits = addToHashCode(bits, a0);
      bits = addToHashCode(bits, a1);
      bits = addToHashCode(bits, a2);
      return bits;
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @param a3 the argument to use for computing a hash-code.
    * @return the 64-bit hash-code.
    */
   public static long toLongHashCode(float a0, float a1, float a2, float a3)
   {
      long bits = DEFAULT_HASHCODE;
      bits = addToHashCode(bits, a0);
      bits = addToHashCode(bits, a1);
      bits = addToHashCode(bits, a2);
      bits = addToHashCode(bits, a3);
      return bits;
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @param a3 the argument to use for computing a hash-code.
    * @param a4 the argument to use for computing a hash-code.
    * @param a5 the argument to use for computing a hash-code.
    * @return the 64-bit hash-code.
    */
   public static long toLongHashCode(float a0, float a1, float a2, float a3, float a4, float a5)
   {
      long bits = DEFAULT_HASHCODE;
      bits = addToHashCode(bits, a0);
      bits = addToHashCode(bits, a1);
      bits = addToHashCode(bits, a2);
      bits = addToHashCode(bits, a3);
      bits = addToHashCode(bits, a4);
      bits = addToHashCode(bits, a5);
      return bits;
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @return the 64-bit hash-code.
    */
   public static long toLongHashCode(Object a0, Object a1)
   {
      long bits = DEFAULT_HASHCODE;
      bits = addToHashCode(bits, a0);
      bits = addToHashCode(bits, a1);
      return bits;
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @return the 64-bit hash-code.
    */
   public static long toLongHashCode(Object a0, Object a1, Object a2)
   {
      long bits = DEFAULT_HASHCODE;
      bits = addToHashCode(bits, a0);
      bits = addToHashCode(bits, a1);
      bits = addToHashCode(bits, a2);
      return bits;
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @param a3 the argument to use for computing a hash-code.
    * @return the 64-bit hash-code.
    */
   public static long toLongHashCode(Object a0, Object a1, Object a2, Object a3)
   {
      long bits = DEFAULT_HASHCODE;
      bits = addToHashCode(bits, a0);
      bits = addToHashCode(bits, a1);
      bits = addToHashCode(bits, a2);
      bits = addToHashCode(bits, a3);
      return bits;
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @param a3 the argument to use for computing a hash-code.
    * @param a4 the argument to use for computing a hash-code.
    * @return the 64-bit hash-code.
    */
   public static long toLongHashCode(Object a0, Object a1, Object a2, Object a3, Object a4)
   {
      long bits = DEFAULT_HASHCODE;
      bits = addToHashCode(bits, a0);
      bits = addToHashCode(bits, a1);
      bits = addToHashCode(bits, a2);
      bits = addToHashCode(bits, a3);
      bits = addToHashCode(bits, a4);
      return bits;
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @param a3 the argument to use for computing a hash-code.
    * @param a4 the argument to use for computing a hash-code.
    * @param a5 the argument to use for computing a hash-code.
    * @return the 64-bit hash-code.
    */
   public static long toLongHashCode(Object a0, Object a1, Object a2, Object a3, Object a4, Object a5)
   {
      long bits = DEFAULT_HASHCODE;
      bits = addToHashCode(bits, a0);
      bits = addToHashCode(bits, a1);
      bits = addToHashCode(bits, a2);
      bits = addToHashCode(bits, a3);
      bits = addToHashCode(bits, a4);
      bits = addToHashCode(bits, a5);
      return bits;
   }

   /**
    * Convenience method for computing the hash-code of the given arguments.
    *
    * @param a0 the argument to use for computing a hash-code.
    * @param a1 the argument to use for computing a hash-code.
    * @param a2 the argument to use for computing a hash-code.
    * @param a3 the argument to use for computing a hash-code.
    * @param a4 the argument to use for computing a hash-code.
    * @param a5 the argument to use for computing a hash-code.
    * @param a6 the argument to use for computing a hash-code.
    * @return the 64-bit hash-code.
    */
   public static long toLongHashCode(Object a0, Object a1, Object a2, Object a3, Object a4, Object a5, Object a6)
   {
      long bits = DEFAULT_HASHCODE;
      bits = addToHashCode(bits, a0);
      bits = addToHashCode(bits, a1);
      bits = addToHashCode(bits, a2);
      bits = addToHashCode(bits, a3);
      bits = addToHashCode(bits, a4);
      bits = addToHashCode(bits, a5);
      bits = addToHashCode(bits, a6);
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
    * Calls {@link #combineHashCode(long, long)} after casting {@code value} to a long.
    *
    * @param hashCode long hash code bit stream to add to.
    * @param value    byte value to add to the hash code.
    * @return new hash code bit stream
    */
   public static long addToHashCode(long hashCode, byte value)
   {
      return combineHashCode(hashCode, (long) value);
   }

   /**
    * Calls {@link #combineHashCode(long, long)} after casting {@code value} to a long.
    *
    * @param hashCode long hash code bit stream to add to.
    * @param value    short value to add to the hash code.
    * @return new hash code bit stream
    */
   public static long addToHashCode(long hashCode, short value)
   {
      return combineHashCode(hashCode, (long) value);
   }

   /**
    * Calls {@link #combineHashCode(long, long)} after casting {@code value} to a long.
    *
    * @param hashCode long hash code bit stream to add to.
    * @param value    char value to add to the hash code.
    * @return new hash code bit stream
    */
   public static long addToHashCode(long hashCode, char value)
   {
      return combineHashCode(hashCode, (long) value);
   }

   /**
    * Calls {@link #combineHashCode(long, long)} after casting {@code value} to a long.
    *
    * @param hashCode long hash code bit stream to add to.
    * @param value    int value to add to the hash code.
    * @return new hash code bit stream
    */
   public static long addToHashCode(long hashCode, int value)
   {
      return combineHashCode(hashCode, (long) value);
   }

   /**
    * Simple redirection to {@link #combineHashCode(long, long)}.
    *
    * @param hashCode long hash code bit stream to add to.
    * @param value    long value to add to the hash code.
    * @return new hash code bit stream
    */
   public static long addToHashCode(long hashCode, long value)
   {
      return combineHashCode(hashCode, value);
   }

   /**
    * Calls {@link #combineHashCode(long, long)} after casting {@code value} to a long.
    *
    * @param hashCode long hash code bit stream to add to.
    * @param value    int value to add to the hash code.
    * @return new hash code bit stream
    */
   public static long addToHashCode(long hashCode, boolean value)
   {
      return combineHashCode(hashCode, (long) Boolean.hashCode(value));
   }

   /**
    * Calls {@link #combineHashCode(long, long)} using the given {@code object}'s hash-code.
    *
    * @param hashCode long hash code bit stream to add to.
    * @param object   object value to add to the hash code.
    * @return new hash code bit stream
    */
   public static long addToHashCode(long hashCode, Object object)
   {
      return combineHashCode(hashCode, Objects.hashCode(object));
   }

   /**
    * Calls {@link #combineHashCode(long, long)} using the given {@code array}'s hash-code.
    *
    * @param hashCode long hash code bit stream to add to.
    * @param array    the array of doubles to add to the hash code.
    * @return new hash code bit stream
    */
   public static long addToHashCode(long hashCode, double[] array)
   {
      return combineHashCode(hashCode, Arrays.hashCode(array));
   }

   /**
    * Calls {@link #combineHashCode(long, long)} using the given {@code array}'s hash-code.
    *
    * @param hashCode long hash code bit stream to add to.
    * @param array    the array of floats to add to the hash code.
    * @return new hash code bit stream
    */
   public static long addToHashCode(long hashCode, float[] array)
   {
      return combineHashCode(hashCode, Arrays.hashCode(array));
   }

   /**
    * Calls {@link #combineHashCode(long, long)} using the given {@code array}'s hash-code.
    *
    * @param hashCode long hash code bit stream to add to.
    * @param array    the array of bytes to add to the hash code.
    * @return new hash code bit stream
    */
   public static long addToHashCode(long hashCode, byte[] array)
   {
      return combineHashCode(hashCode, Arrays.hashCode(array));
   }

   /**
    * Calls {@link #combineHashCode(long, long)} using the given {@code array}'s hash-code.
    *
    * @param hashCode long hash code bit stream to add to.
    * @param array    the array of shorts to add to the hash code.
    * @return new hash code bit stream
    */
   public static long addToHashCode(long hashCode, short[] array)
   {
      return combineHashCode(hashCode, Arrays.hashCode(array));
   }

   /**
    * Calls {@link #combineHashCode(long, long)} using the given {@code array}'s hash-code.
    *
    * @param hashCode long hash code bit stream to add to.
    * @param array    the array of chars to add to the hash code.
    * @return new hash code bit stream
    */
   public static long addToHashCode(long hashCode, char[] array)
   {
      return combineHashCode(hashCode, Arrays.hashCode(array));
   }

   /**
    * Calls {@link #combineHashCode(long, long)} using the given {@code array}'s hash-code.
    *
    * @param hashCode long hash code bit stream to add to.
    * @param array    the array of ints to add to the hash code.
    * @return new hash code bit stream
    */
   public static long addToHashCode(long hashCode, int[] array)
   {
      return combineHashCode(hashCode, Arrays.hashCode(array));
   }

   /**
    * Calls {@link #combineHashCode(long, long)} using the given {@code array}'s hash-code.
    *
    * @param hashCode long hash code bit stream to add to.
    * @param array    the array of longs to add to the hash code.
    * @return new hash code bit stream
    */
   public static long addToHashCode(long hashCode, long[] array)
   {
      return combineHashCode(hashCode, Arrays.hashCode(array));
   }

   /**
    * Calls {@link #combineHashCode(long, long)} using the given {@code array}'s hash-code.
    *
    * @param hashCode long hash code bit stream to add to.
    * @param array    the array of booleans to add to the hash code.
    * @return new hash code bit stream
    */
   public static long addToHashCode(long hashCode, boolean[] array)
   {
      return combineHashCode(hashCode, Arrays.hashCode(array));
   }

   /**
    * Calls {@link #combineHashCode(long, long)} using the given {@code array}'s hash-code.
    *
    * @param hashCode long hash code bit stream to add to.
    * @param array    the array of objects to add to the hash code.
    * @return new hash code bit stream
    */
   public static long addToHashCode(long hashCode, Object[] array)
   {
      return combineHashCode(hashCode, Arrays.hashCode(array));
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
}
