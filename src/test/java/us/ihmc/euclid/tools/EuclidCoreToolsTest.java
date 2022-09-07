package us.ihmc.euclid.tools;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;
import static us.ihmc.euclid.tools.EuclidCoreTools.EPS_NORM_FAST_SQRT;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Vector4D;

public class EuclidCoreToolsTest
{
   @Test
   public void testConstants()
   {

      Point2D expectedOrigin2D = new Point2D();
      assertEquals(expectedOrigin2D, EuclidCoreTools.origin2D);
      assertEquals(expectedOrigin2D.hashCode(), EuclidCoreTools.origin2D.hashCode());
      assertEquals(expectedOrigin2D.toString(), EuclidCoreTools.origin2D.toString());

      Point3D expectedOrigin3D = new Point3D();
      assertEquals(expectedOrigin3D, EuclidCoreTools.origin3D);
      assertEquals(expectedOrigin3D.hashCode(), EuclidCoreTools.origin3D.hashCode());
      assertEquals(expectedOrigin3D.toString(), EuclidCoreTools.origin3D.toString());

      Vector2D expectedZeroVector2D = new Vector2D();
      assertEquals(expectedZeroVector2D, EuclidCoreTools.zeroVector2D);
      assertEquals(expectedZeroVector2D.hashCode(), EuclidCoreTools.zeroVector2D.hashCode());
      assertEquals(expectedZeroVector2D.toString(), EuclidCoreTools.zeroVector2D.toString());

      Vector3D expectedZeroVector3D = new Vector3D();
      assertEquals(expectedZeroVector3D, EuclidCoreTools.zeroVector3D);
      assertEquals(expectedZeroVector3D.hashCode(), EuclidCoreTools.zeroVector3D.hashCode());
      assertEquals(expectedZeroVector3D.toString(), EuclidCoreTools.zeroVector3D.toString());

      Vector4D expectedQuaternion = new Vector4D();
      expectedQuaternion.setElement(3, 1);
      assertEquals(expectedQuaternion, EuclidCoreTools.neutralQuaternion);
      assertEquals(expectedQuaternion.hashCode(), EuclidCoreTools.neutralQuaternion.hashCode());
      assertEquals(expectedQuaternion.toString(), EuclidCoreTools.neutralQuaternion.toString());

      Matrix3D expectedZeroMatrix3D = new Matrix3D();
      expectedZeroMatrix3D.setToZero();
      assertEquals(expectedZeroMatrix3D, EuclidCoreTools.zeroMatrix3D);
      assertEquals(expectedZeroMatrix3D.toString(), EuclidCoreTools.zeroMatrix3D.toString());
      assertEquals(expectedZeroMatrix3D.hashCode(), EuclidCoreTools.zeroMatrix3D.hashCode());

      Matrix3D expectedIdentity3D = new Matrix3D();
      expectedIdentity3D.setIdentity();
      assertEquals(expectedIdentity3D, EuclidCoreTools.identityMatrix3D);
      assertEquals(expectedIdentity3D.toString(), EuclidCoreTools.identityMatrix3D.toString());
      assertEquals(expectedIdentity3D.hashCode(), EuclidCoreTools.identityMatrix3D.hashCode());

   }

   @Test
   public void testFastSquareRoot() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < 1000; i++)
      {
         double squaredValue = 1.5 * random.nextDouble();
         squaredValue = Math.min(squaredValue, 1.0 - EPS_NORM_FAST_SQRT);

         double actualValue = EuclidCoreTools.fastSquareRoot(squaredValue);
         double expectedValue = EuclidCoreTools.squareRoot(squaredValue);
         assertEquals(expectedValue, actualValue, Double.MIN_VALUE);
      }

      for (int i = 0; i < 1000; i++)
      {
         double squaredValue = 0.8 + 1.5 * random.nextDouble();
         squaredValue = Math.max(squaredValue, 1.0 + EPS_NORM_FAST_SQRT);

         double actualValue = EuclidCoreTools.fastSquareRoot(squaredValue);
         double expectedValue = EuclidCoreTools.squareRoot(squaredValue);
         assertEquals(expectedValue, actualValue, Double.MIN_VALUE);
      }

      for (int i = 0; i < 1000; i++)
      {
         double squaredValue = EuclidCoreRandomTools.nextDouble(random, 1.0 - EPS_NORM_FAST_SQRT, 1.0 + EPS_NORM_FAST_SQRT);
         squaredValue = Math.max(squaredValue, 1.0 + EPS_NORM_FAST_SQRT);

         double actualValue = EuclidCoreTools.fastSquareRoot(squaredValue);
         double expectedValue = EuclidCoreTools.squareRoot(squaredValue);
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

      assertTrue(EuclidCoreTools.fastNorm(1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, 1.0) == 1.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, 0.0) == 0.0);
      assertTrue(EuclidCoreTools.fastNorm(-1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, -1.0) == 1.0);
      assertTrue(EuclidCoreTools.fastNorm(2.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, 2.0) == 2.0);
      assertTrue(EuclidCoreTools.fastNorm(-2.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, -2.0) == 2.0);
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

      assertTrue(EuclidCoreTools.fastNorm(1.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, 1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, 0.0, 1.0) == 1.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, 0.0, 0.0) == 0.0);
      assertTrue(EuclidCoreTools.fastNorm(-1.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, -1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, 0.0, -1.0) == 1.0);
      assertTrue(EuclidCoreTools.fastNorm(2.0, 0.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, 2.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, 0.0, 2.0) == 2.0);
      assertTrue(EuclidCoreTools.fastNorm(-2.0, 0.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, -2.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, 0.0, -2.0) == 2.0);
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

      assertTrue(EuclidCoreTools.fastNorm(1.0, 0.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, 1.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, 0.0, 1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, 0.0, 0.0, 1.0) == 1.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, 0.0, 0.0, 0.0) == 0.0);
      assertTrue(EuclidCoreTools.fastNorm(-1.0, 0.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, -1.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, 0.0, -1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, 0.0, 0.0, -1.0) == 1.0);
      assertTrue(EuclidCoreTools.fastNorm(2.0, 0.0, 0.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, 2.0, 0.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, 0.0, 2.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, 0.0, 0.0, 2.0) == 2.0);
      assertTrue(EuclidCoreTools.fastNorm(-2.0, 0.0, 0.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, -2.0, 0.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, 0.0, -2.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.fastNorm(0.0, 0.0, 0.0, -2.0) == 2.0);
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
   public void testAngleGeometricallyEquals() throws Exception
   {
      Random random = new Random(35635);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with angle that are close-ish to each other
         double epsilon = EuclidCoreRandomTools.nextDouble(random, 1.0e-12, 0.5);
         double angleA = EuclidCoreRandomTools.nextDouble(random, 10.0 * Math.PI);
         double angleNotEqual = angleA + (random.nextBoolean() ? -1.01 : 1.01) * epsilon;
         double angleEqual = angleA + (random.nextBoolean() ? -0.99 : 0.99) * epsilon;

         assertFalse(EuclidCoreTools.angleGeometricallyEquals(angleA, angleNotEqual, epsilon));
         assertTrue(EuclidCoreTools.angleGeometricallyEquals(angleA, angleEqual, epsilon));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with angle that are far-ish to each other
         double epsilon = EuclidCoreRandomTools.nextDouble(random, 1.0e-12, 0.5);
         double angleA = EuclidCoreRandomTools.nextDouble(random, 10.0 * Math.PI);
         double twoPIMutiple = random.nextInt(15) * 2.0 * Math.PI;
         if (random.nextBoolean())
            twoPIMutiple = -twoPIMutiple;
         double angleNotEqual = angleA + (random.nextBoolean() ? -1.01 : 1.01) * epsilon + twoPIMutiple;
         double angleEqual = angleA + (random.nextBoolean() ? -0.99 : 0.99) * epsilon + twoPIMutiple;

         assertFalse(EuclidCoreTools.angleGeometricallyEquals(angleA, angleNotEqual, epsilon));
         assertTrue(EuclidCoreTools.angleGeometricallyEquals(angleA, angleEqual, epsilon));
      }
   }

   @Test
   public void testIsAngleZero() throws Exception
   {
      Random random = new Random(35635);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double epsilon = EuclidCoreRandomTools.nextDouble(random, 1.0e-12, 0.5);
         double twoPIMutiple = random.nextInt(15) * 2.0 * Math.PI;
         double zeroAngle = 0.99 * EuclidCoreRandomTools.nextDouble(random, epsilon);
         double nonZeroAngle = EuclidCoreRandomTools.nextDouble(random, 1.01 * epsilon, Math.PI);
         if (random.nextBoolean())
            nonZeroAngle = -nonZeroAngle;

         assertTrue(EuclidCoreTools.isAngleZero(zeroAngle, epsilon));
         assertTrue(EuclidCoreTools.isAngleZero(zeroAngle + twoPIMutiple, epsilon));

         assertFalse(EuclidCoreTools.isAngleZero(nonZeroAngle, epsilon));
         assertFalse(EuclidCoreTools.isAngleZero(nonZeroAngle + twoPIMutiple, epsilon));
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

      for (int i = 0; i < ITERATIONS; i++)
      {
         double a = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double b = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double c = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double d = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double expected = Math.max(a, Math.max(b, Math.max(c, d)));
         double actual = EuclidCoreTools.max(a, b, c, d);
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

      for (int i = 0; i < ITERATIONS; i++)
      {
         double a = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double b = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double c = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double d = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double expected = Math.min(a, Math.min(b, Math.min(c, d)));
         double actual = EuclidCoreTools.min(a, b, c, d);
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

   @Test
   public void testClamp() throws Exception
   {
      Random random = new Random(3453);

      { // Test clamp(double value, double minMax)

         for (int i = 0; i < ITERATIONS; i++)
         {
            double minMax = EuclidCoreRandomTools.nextDouble(random, 0.0, 100.0);
            double valueInside = EuclidCoreRandomTools.nextDouble(random, minMax);
            double valueUnder = EuclidCoreRandomTools.nextDouble(random, -100.0, 0.0) - minMax;
            double valueOver = EuclidCoreRandomTools.nextDouble(random, 0.0, 100.0) + minMax;

            assertTrue(valueInside == EuclidCoreTools.clamp(valueInside, minMax));
            assertTrue(-minMax == EuclidCoreTools.clamp(valueUnder, minMax));
            assertTrue(minMax == EuclidCoreTools.clamp(valueOver, minMax));
         }

         EuclidCoreTestTools.assertExceptionIsThrown(() -> EuclidCoreTools.clamp(0.0, -EuclidCoreTools.CLAMP_EPS - 1.0e-12), RuntimeException.class);
      }

      { // Test clamp(double value, double minMax)

         for (int i = 0; i < ITERATIONS; i++)
         {
            double min = EuclidCoreRandomTools.nextDouble(random, -100.0, 100.0);
            double max = min + EuclidCoreRandomTools.nextDouble(random, 0.0, 100.0);
            double valueInside = EuclidCoreRandomTools.nextDouble(random, min, max);
            double valueUnder = min - EuclidCoreRandomTools.nextDouble(random, 0.0, 100.0);
            double valueOver = max + EuclidCoreRandomTools.nextDouble(random, 0.0, 100.0);

            assertTrue(valueInside == EuclidCoreTools.clamp(valueInside, min, max));
            assertTrue(min == EuclidCoreTools.clamp(valueUnder, min, max));
            assertTrue(max == EuclidCoreTools.clamp(valueOver, min, max));
         }

         double min = EuclidCoreRandomTools.nextDouble(random, -100.0, 100.0);
         EuclidCoreTestTools.assertExceptionIsThrown(() -> EuclidCoreTools.clamp(0.0, min, min - EuclidCoreTools.CLAMP_EPS - 1.0e-12), RuntimeException.class);
      }
   }

   @Test
   public void testFastAcos()
   {
      double epsilon = 1.0e-14;
      double maxError = 0.0;
      double maxErrorX = 0.0;
      double error;
      double expected;
      double actual;

      for (double x = -1.0; x < 1.0; x += 1.0e-6)
      {
         expected = Math.acos(x);
         actual = EuclidCoreTools.fastAcos(x);
         assertEquals(expected, actual, epsilon, "x=" + x);

         error = Math.abs(actual - expected);
         if (error > maxError)
         {
            maxError = error;
            maxErrorX = x;
         }
      }

      double x = -1.0;
      expected = Math.acos(x);
      actual = EuclidCoreTools.fastAcos(x);
      assertEquals(expected, actual, epsilon, "x=" + x);
      error = Math.abs(actual - expected);
      if (error > maxError)
      {
         maxError = error;
         maxErrorX = x;
      }

      x = 1.0;
      expected = Math.acos(x);
      actual = EuclidCoreTools.fastAcos(x);
      assertEquals(expected, actual, epsilon, "x=" + x);
      error = Math.abs(actual - expected);
      if (error > maxError)
      {
         maxError = error;
         maxErrorX = x;
      }

      System.out.println("EuclidCoreToolsTest.testFastAcos: max error=" + maxError + " at x=" + maxErrorX);
   }

   @Test
   public void testReverse()
   {
      Random random = new Random(234234);

      for (int i = 0; i < ITERATIONS; i++)
      {
         List<Integer> originalList = new ArrayList<>();

         int size = random.nextInt(20);
         while (originalList.size() < size)
            originalList.add(Integer.valueOf(originalList.size()));

         int fromIndex = size == 0 ? 0 : random.nextInt(size);
         int toIndex = size == 0 ? 0 : fromIndex + random.nextInt(originalList.size() - fromIndex);

         List<Integer> expected = new ArrayList<>(originalList);
         Collections.reverse(expected.subList(fromIndex, toIndex));
         List<Integer> actual = new ArrayList<>(originalList);
         EuclidCoreTools.reverse(actual, fromIndex, toIndex);

         assertEquals(expected, actual);
      }
   }
}
