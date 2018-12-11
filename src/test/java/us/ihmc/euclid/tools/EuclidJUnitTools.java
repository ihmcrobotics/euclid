package us.ihmc.euclid.tools;

import static org.junit.jupiter.api.Assertions.*;

public class EuclidJUnitTools
{
   public static void assertArrayEqualsDelta(double[] expecteds, double[] actuals, double delta)
   {
      if (delta == 0.0)
         assertArrayEquals(expecteds, actuals);
      else
         assertArrayEquals(expecteds, actuals, delta);
   }

   public static void assertArrayEqualsDelta(float[] expecteds, float[] actuals, float delta)
   {
      if (delta == 0.0)
         assertArrayEquals(expecteds, actuals);
      else
         assertArrayEquals(expecteds, actuals, delta);
   }

   public static void assertArrayEqualsDelta(String string, double[] data, double[] ds, double delta)
   {
      if (delta == 0.0)
         assertArrayEquals(data, ds, string);
      else
         assertArrayEquals(data, ds, delta, string);
   }

   static public void assertEqualsDelta(String message, double expected, double actual, double delta)
   {
      if (delta == 0.0)
         assertEquals(expected, actual, message);
      else
         assertEquals(expected, actual, delta, message);
   }

   static public void assertEqualsDelta(String message, float expected, float actual, float delta)
   {
      if (delta == 0.0)
         assertEquals(expected, actual, message);
      else
         assertEquals(expected, actual, delta, message);
   }

   static public void assertEqualsDelta(double expected, double actual, double delta)
   {
      if (delta == 0.0)
         assertEquals(expected, actual);
      else
         assertEquals(expected, actual, delta);
   }

   static public void assertEqualsDelta(float expected, float actual, float delta)
   {
      if (delta == 0.0)
         assertEquals(expected, actual);
      else
         assertEquals(expected, actual, delta);
   }
}
