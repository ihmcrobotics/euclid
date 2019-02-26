package us.ihmc.euclid.tools;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.EuclidTestConstants.*;
import static us.ihmc.euclid.tools.EuclidCoreTools.*;

import java.util.Arrays;
import java.util.Random;

import org.junit.jupiter.api.Test;

public class EuclidCoreToolsTest
{
   @Test
   public void testFastSquareRoot() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < 1000; i++)
      {
         double squaredValue = 1.5 * random.nextDouble();
         squaredValue = Math.min(squaredValue, 1.0 - EPS_NORM_FAST_SQRT);

         double actualValue = EuclidCoreTools.fastSquareRoot(squaredValue);
         double expectedValue = Math.sqrt(squaredValue);
         assertEquals(expectedValue, actualValue, Double.MIN_VALUE);
      }

      for (int i = 0; i < 1000; i++)
      {
         double squaredValue = 0.8 + 1.5 * random.nextDouble();
         squaredValue = Math.max(squaredValue, 1.0 + EPS_NORM_FAST_SQRT);

         double actualValue = EuclidCoreTools.fastSquareRoot(squaredValue);
         double expectedValue = Math.sqrt(squaredValue);
         assertEquals(expectedValue, actualValue, Double.MIN_VALUE);
      }

      for (int i = 0; i < 1000; i++)
      {
         double squaredValue = EuclidCoreRandomTools.nextDouble(random, 1.0 - EPS_NORM_FAST_SQRT, 1.0 + EPS_NORM_FAST_SQRT);
         squaredValue = Math.max(squaredValue, 1.0 + EPS_NORM_FAST_SQRT);

         double actualValue = EuclidCoreTools.fastSquareRoot(squaredValue);
         double expectedValue = Math.sqrt(squaredValue);
         assertEquals(expectedValue, actualValue, Double.MIN_VALUE);
      }
   }

   @Test
   public void testContainsNaNWith2Elements() throws Exception
   {
      assertFalse(EuclidCoreTools.containsNaN(0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(Double.NaN, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, Double.NaN));
   }

   @Test
   public void testContainsNaNWith3Elements() throws Exception
   {
      assertFalse(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(Double.NaN, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, Double.NaN, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, Double.NaN));
   }

   @Test
   public void testContainsNaNWith4Elements() throws Exception
   {
      assertFalse(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(Double.NaN, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, Double.NaN, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, Double.NaN, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, Double.NaN));
   }

   @Test
   public void testContainsNaNWith9Elements() throws Exception
   {
      assertFalse(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN));
   }

   @Test
   public void testContainsNaNWithArray() throws Exception
   {
      assertFalse(EuclidCoreTools.containsNaN(new double[0]));
      assertFalse(EuclidCoreTools.containsNaN(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN}));
   }

   @Test
   public void testNormSquaredWith2Elements() throws Exception
   {
      assertTrue(EuclidCoreTools.normSquared(1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 1.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0) == 0.0);

      assertTrue(EuclidCoreTools.normSquared(-1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, -1.0) == 1.0);

      assertTrue(EuclidCoreTools.normSquared(2.0, 0.0) == 4.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 2.0) == 4.0);

      assertTrue(EuclidCoreTools.normSquared(-2.0, 0.0) == 4.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, -2.0) == 4.0);
   }

   @Test
   public void testNormSquaredWith3Elements() throws Exception
   {
      assertTrue(EuclidCoreTools.normSquared(1.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, 1.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, 0.0) == 0.0);

      assertTrue(EuclidCoreTools.normSquared(-1.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, -1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, -1.0) == 1.0);

      assertTrue(EuclidCoreTools.normSquared(2.0, 0.0, 0.0) == 4.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 2.0, 0.0) == 4.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, 2.0) == 4.0);

      assertTrue(EuclidCoreTools.normSquared(-2.0, 0.0, 0.0) == 4.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, -2.0, 0.0) == 4.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, -2.0) == 4.0);
   }

   @Test
   public void testNormSquaredWith4Elements() throws Exception
   {
      assertTrue(EuclidCoreTools.normSquared(1.0, 0.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 1.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, 1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, 0.0, 1.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, 0.0, 0.0) == 0.0);

      assertTrue(EuclidCoreTools.normSquared(-1.0, 0.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, -1.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, -1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, 0.0, -1.0) == 1.0);

      assertTrue(EuclidCoreTools.normSquared(2.0, 0.0, 0.0, 0.0) == 4.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 2.0, 0.0, 0.0) == 4.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, 2.0, 0.0) == 4.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, 0.0, 2.0) == 4.0);

      assertTrue(EuclidCoreTools.normSquared(-2.0, 0.0, 0.0, 0.0) == 4.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, -2.0, 0.0, 0.0) == 4.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, -2.0, 0.0) == 4.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, 0.0, -2.0) == 4.0);
   }

   @Test
   public void testNormWith2Elements() throws Exception
   {
      assertTrue(EuclidCoreTools.norm(1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, 1.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0) == 0.0);

      assertTrue(EuclidCoreTools.norm(-1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, -1.0) == 1.0);

      assertTrue(EuclidCoreTools.norm(2.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.norm(0.0, 2.0) == 2.0);

      assertTrue(EuclidCoreTools.norm(-2.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.norm(0.0, -2.0) == 2.0);
   }

   @Test
   public void testNormWith3Elements() throws Exception
   {
      assertTrue(EuclidCoreTools.norm(1.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, 1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, 1.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, 0.0) == 0.0);

      assertTrue(EuclidCoreTools.norm(-1.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, -1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, -1.0) == 1.0);

      assertTrue(EuclidCoreTools.norm(2.0, 0.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.norm(0.0, 2.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, 2.0) == 2.0);

      assertTrue(EuclidCoreTools.norm(-2.0, 0.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.norm(0.0, -2.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, -2.0) == 2.0);
   }

   @Test
   public void testNormWith4Elements() throws Exception
   {
      assertTrue(EuclidCoreTools.norm(1.0, 0.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, 1.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, 1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, 0.0, 1.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, 0.0, 0.0) == 0.0);

      assertTrue(EuclidCoreTools.norm(-1.0, 0.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, -1.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, -1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, 0.0, -1.0) == 1.0);

      assertTrue(EuclidCoreTools.norm(2.0, 0.0, 0.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.norm(0.0, 2.0, 0.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, 2.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, 0.0, 2.0) == 2.0);

      assertTrue(EuclidCoreTools.norm(-2.0, 0.0, 0.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.norm(0.0, -2.0, 0.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, -2.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, 0.0, -2.0) == 2.0);
   }

   @Test
   public void testTrimAngleMinusPiToPi() throws Exception
   {
      Random random = new Random(2323L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double startOfRange = -Math.PI;
         double endOfRange = Math.PI;
         double expectedAngle = EuclidCoreRandomTools.nextDouble(random, startOfRange, endOfRange);
         double angleToShift = expectedAngle + (random.nextInt(21) - 10) * 2.0 * Math.PI;
         double actualAngle = EuclidCoreTools.trimAngleMinusPiToPi(angleToShift);
         assertEquals(expectedAngle, actualAngle, 1.0e-12, "iteration: " + i);

         expectedAngle = startOfRange;
         angleToShift = expectedAngle + (random.nextInt(21) - 10) * 2.0 * Math.PI;
         actualAngle = EuclidCoreTools.trimAngleMinusPiToPi(angleToShift);
         assertEquals(expectedAngle, actualAngle, 1.0e-12);

         expectedAngle = endOfRange - 1.0e-9;
         angleToShift = expectedAngle + (random.nextInt(21) - 10) * 2.0 * Math.PI;
         actualAngle = EuclidCoreTools.trimAngleMinusPiToPi(angleToShift);
         assertEquals(expectedAngle, actualAngle, 1.0e-12);
      }
   }

   @Test
   public void testAngleDifferenceMinusPiToPi() throws Exception
   {
      Random random = new Random(2323L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double startOfRange = -Math.PI;
         double endOfRange = Math.PI;
         double expectedDifference = EuclidCoreRandomTools.nextDouble(random, startOfRange, endOfRange);
         double untrimmedDifference = expectedDifference + (random.nextInt(21) - 10) * 2.0 * Math.PI;
         double angleA = EuclidCoreRandomTools.nextDouble(random, 4.0 * Math.PI);
         double angleB = angleA - untrimmedDifference;
         double actualDifference = EuclidCoreTools.angleDifferenceMinusPiToPi(angleA, angleB);
         assertEquals(expectedDifference, actualDifference, 1.0e-12, "iteration: " + i);
      }
   }

   @Test
   public void testShiftAngleInRange() throws Exception
   {
      Random random = new Random(23423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double startOfRange = EuclidCoreRandomTools.nextDouble(random, 2.0 * Math.PI);
         double endOfRange = startOfRange + 2.0 * Math.PI;
         double expectedAngle = EuclidCoreRandomTools.nextDouble(random, startOfRange, endOfRange);
         double angleToShift = expectedAngle + (random.nextInt(21) - 10) * 2.0 * Math.PI;
         double actualAngle = EuclidCoreTools.shiftAngleInRange(angleToShift, startOfRange);
         assertEquals(expectedAngle, actualAngle, 1.0e-12, "iteration: " + i);

         expectedAngle = startOfRange;
         angleToShift = expectedAngle + (random.nextInt(21) - 10) * 2.0 * Math.PI;
         actualAngle = EuclidCoreTools.shiftAngleInRange(angleToShift, startOfRange);
         assertEquals(expectedAngle, actualAngle, 1.0e-12);

         expectedAngle = endOfRange - 1.0e-9;
         angleToShift = expectedAngle + (random.nextInt(21) - 10) * 2.0 * Math.PI;
         actualAngle = EuclidCoreTools.shiftAngleInRange(angleToShift, startOfRange);
         assertEquals(expectedAngle, actualAngle, 1.0e-12);
      }
   }

   @Test
   public void testMax() throws Exception
   {
      Random random = new Random(45645L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double a = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double b = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double c = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double expected = Math.max(a, Math.max(b, c));
         double actual = EuclidCoreTools.max(a, b, c);
         assertTrue(expected == actual);
      }
   }

   @Test
   public void testMin() throws Exception
   {
      Random random = new Random(45645L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double a = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double b = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double c = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double expected = Math.min(a, Math.min(b, c));
         double actual = EuclidCoreTools.min(a, b, c);
         assertTrue(expected == actual);
      }
   }

   @Test
   public void testMed() throws Exception
   {
      Random random = new Random(45645L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double a = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double b = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double c = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double[] sorted = {a, b, c};
         Arrays.sort(sorted);
         double expected = sorted[1];
         double actual = EuclidCoreTools.med(a, b, c);
         if (expected != actual)
            EuclidCoreTools.med(a, b, c);
         assertTrue(expected == actual, "a = " + a + ", b = " + b + ", c = " + c);
      }
   }

   @Test
   public void testInterpolate() throws Exception
   {
      Random random = new Random(3665L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Tuple.interpolate(double a, double b, double alpha)
         double a = random.nextDouble();
         double b = random.nextDouble();
         double alpha = random.nextDouble();

         double result = EuclidCoreTools.interpolate(a, b, alpha);
         double expected = a + alpha * (b - a);
         assertEquals(result, expected, 1.0e-10);

         alpha = 0.5;
         result = EuclidCoreTools.interpolate(a, b, alpha);
         assertTrue(result == 0.5 * a + 0.5 * b);
         alpha = 0.0;
         result = EuclidCoreTools.interpolate(a, b, alpha);
         assertTrue(result == a);
         alpha = 1.0;
         result = EuclidCoreTools.interpolate(a, b, alpha);
         assertTrue(result == b);

         for (alpha = -2.0; alpha <= 2.0; alpha += 0.1)
         {
            result = EuclidCoreTools.interpolate(a, b, alpha);
            assertEquals(result, a + alpha * (b - a), 1.0e-10);
         }
      }
   }
}
