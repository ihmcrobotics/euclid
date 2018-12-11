package us.ihmc.euclid.tools;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.tools.EuclidJUnitTools.*;

import static us.ihmc.euclid.testSuite.EuclidTestSuite.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;

public class EuclidCoreRandomToolsTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testNextVector3D()
   {
      for (int i = 0; i < ITERATIONS; i++)
      {
         Random random = new Random(16451L);

         Vector3D vector = new Vector3D();
         Vector3D zeroVector = new Vector3D();
         Vector3D min = new Vector3D(-1, -1, -1);
         Vector3D max = new Vector3D(1, 1, 1);

         EuclidCoreRandomTools.nextVector3D(random);

         assertTrue(min.getX() <= vector.getX());
         assertTrue(vector.getX() <= max.getX());
         assertTrue(min.getY() <= vector.getY());
         assertTrue(vector.getY() <= max.getY());
         assertTrue(min.getZ() <= vector.getZ());
         assertTrue(vector.getZ() <= max.getZ());

         assertTrue(zeroVector.getX() == 0);
         assertTrue(zeroVector.getY() == 0);
         assertTrue(zeroVector.getZ() == 0);

         assertNotSame(vector, null);
      }
   }

   @Test
   public void testRandomizeTuple3D()
   {
      Random random = new Random(6841032L);
      Tuple3DBasics tupleToRandomize = new Point3D();

      Tuple3DBasics tupleToRandomizeCopy = new Point3D();
      tupleToRandomizeCopy.setX(tupleToRandomize.getX());
      tupleToRandomizeCopy.setY(tupleToRandomize.getY());
      tupleToRandomizeCopy.setZ(tupleToRandomize.getZ());

      { // Test randomize(Random random, TupleBasics tupleToRandomize)
         Tuple3DBasics previousValue = new Point3D();
         previousValue.setToNaN();

         for (int i = 0; i < ITERATIONS; i++)
         {
            EuclidCoreRandomTools.randomizeTuple3D(random, tupleToRandomize);

            assertTrue(-1.0 <= tupleToRandomize.getX());
            assertTrue(tupleToRandomize.getX() <= 1.0);
            assertTrue(-1.0 <= tupleToRandomize.getY());
            assertTrue(tupleToRandomize.getY() <= 1.0);
            assertTrue(-1.0 <= tupleToRandomize.getZ());
            assertTrue(tupleToRandomize.getZ() <= 1.0);

            assertFalse(TupleTools.epsilonEquals(tupleToRandomize, previousValue, 1.0e-10));
            previousValue.set(tupleToRandomize);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test randomize(Random random, TupleBasics minMax, TupleBasics tupleToRandomize)
         Tuple3DBasics minMax = new Point3D();

         minMax.setX(random.nextDouble());
         minMax.setY(random.nextDouble());
         minMax.setZ(random.nextDouble());

         EuclidCoreRandomTools.randomizeTuple3D(random, minMax, tupleToRandomize);

         assertTrue(tupleToRandomizeCopy.getX() - minMax.getX() <= tupleToRandomize.getX());
         assertTrue(tupleToRandomize.getX() <= tupleToRandomizeCopy.getX() + minMax.getX());
         assertTrue(tupleToRandomizeCopy.getY() - minMax.getY() <= tupleToRandomize.getY());
         assertTrue(tupleToRandomize.getY() <= tupleToRandomizeCopy.getY() + minMax.getY());
         assertTrue(tupleToRandomizeCopy.getZ() - minMax.getZ() <= tupleToRandomize.getZ());
         assertTrue(tupleToRandomize.getZ() <= tupleToRandomizeCopy.getZ() + minMax.getZ());
      }

      { // Test randomize(Random random, TupleBasics min, TupleBasics max, TupleBasics tupleToRandomize)
         Tuple3DBasics min = new Point3D();
         Tuple3DBasics max = new Point3D();

         for (int i = 0; i < ITERATIONS; i++)
         {
            min.setX(random.nextDouble());
            min.setY(random.nextDouble());
            min.setZ(random.nextDouble());

            max.setX(min.getX() + random.nextDouble());
            max.setY(min.getY() + random.nextDouble());
            max.setZ(min.getZ() + random.nextDouble());

            EuclidCoreRandomTools.randomizeTuple3D(random, min, max, tupleToRandomize);

            assertTrue(tupleToRandomizeCopy.getX() - min.getX() <= tupleToRandomize.getX());
            assertTrue(tupleToRandomize.getX() <= tupleToRandomizeCopy.getX() + max.getX());
            assertTrue(tupleToRandomizeCopy.getY() - min.getY() <= tupleToRandomize.getY());
            assertTrue(tupleToRandomize.getY() <= tupleToRandomizeCopy.getY() + max.getY());
            assertTrue(tupleToRandomizeCopy.getZ() - min.getZ() <= tupleToRandomize.getZ());
            assertTrue(tupleToRandomize.getZ() <= tupleToRandomizeCopy.getZ() + max.getZ());
         }
      }
   }

   @Test
   public void testNextAxisAngle() throws Exception
   {
      Random random = new Random(54654L);
      AxisAngle axisAngle = new AxisAngle();
      AxisAngle axisAnglePrevious = new AxisAngle();
      double minMax = 1.0;
      double actualAngleMin = Double.POSITIVE_INFINITY;
      double actualAngleMax = Double.NEGATIVE_INFINITY;

      axisAnglePrevious.setToNaN();

      for (int i = 0; i < ITERATIONS; i++)
      {
         EuclidCoreRandomTools.randomizeAxisAngle(random, minMax, axisAngle);
         assertTrue(Math.abs(axisAngle.getAngle()) < minMax);
         double uNorm = Math.sqrt(axisAngle.getX() * axisAngle.getX() + axisAngle.getY() * axisAngle.getY() + axisAngle.getZ() * axisAngle.getZ());
         assertTrue(Math.abs(uNorm - 1.0) < EPSILON);
         boolean hasChanged = axisAngle.getX() != axisAnglePrevious.getX();
         hasChanged &= axisAngle.getY() != axisAnglePrevious.getY();
         hasChanged &= axisAngle.getZ() != axisAnglePrevious.getZ();
         hasChanged &= axisAngle.getAngle() != axisAnglePrevious.getAngle();
         assertTrue(hasChanged);

         actualAngleMin = Math.min(actualAngleMin, axisAngle.getAngle());
         actualAngleMax = Math.max(actualAngleMax, axisAngle.getAngle());

         axisAnglePrevious.setX(axisAngle.getX());
         axisAnglePrevious.setY(axisAngle.getY());
         axisAnglePrevious.setZ(axisAngle.getZ());
         axisAnglePrevious.setAngle(axisAngle.getAngle());
      }

      assertTrue(actualAngleMax > 0.75 * minMax);
      assertTrue(actualAngleMin < -0.75 * minMax);

      axisAnglePrevious.setToNaN();

      for (int i = 0; i < ITERATIONS; i++)
      {
         EuclidCoreRandomTools.randomizeAxisAngle(random, axisAngle);
         assertTrue(Math.abs(axisAngle.getAngle()) < Math.PI);
         double uNorm = Math.sqrt(axisAngle.getX() * axisAngle.getX() + axisAngle.getY() * axisAngle.getY() + axisAngle.getZ() * axisAngle.getZ());
         assertTrue(Math.abs(uNorm - 1.0) < EPSILON);
         boolean hasChanged = axisAngle.getX() != axisAnglePrevious.getX();
         hasChanged &= axisAngle.getY() != axisAnglePrevious.getY();
         hasChanged &= axisAngle.getZ() != axisAnglePrevious.getZ();
         hasChanged &= axisAngle.getAngle() != axisAnglePrevious.getAngle();
         assertTrue(hasChanged);

         axisAnglePrevious.setX(axisAngle.getX());
         axisAnglePrevious.setY(axisAngle.getY());
         axisAnglePrevious.setZ(axisAngle.getZ());
         axisAnglePrevious.setAngle(axisAngle.getAngle());
      }
   }

   @Test
   public void testNextMatrix3D() throws Exception
   {
      Random random = new Random(23452);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test nextMatrix3D(Random random)
         Matrix3D matrix3D = EuclidCoreRandomTools.nextMatrix3D(random);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               assertTrue(matrix3D.getElement(row, column) <= 1.0);
               assertTrue(matrix3D.getElement(row, column) >= -1.0);
               assertTrue(matrix3D.getElement(row, column) != 0.0);
               assertTrue(Double.isFinite(matrix3D.getElement(row, column)));
            }
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test nextMatrix3D(Random random, double minMaxValue)
         double minMaxValue = EuclidCoreRandomTools.nextDouble(random, 0.0, 100.0);
         Matrix3D matrix3D = EuclidCoreRandomTools.nextMatrix3D(random, minMaxValue);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               assertTrue(matrix3D.getElement(row, column) <= minMaxValue);
               assertTrue(matrix3D.getElement(row, column) >= -minMaxValue);
               assertTrue(matrix3D.getElement(row, column) != 0.0);
               assertTrue(Double.isFinite(matrix3D.getElement(row, column)));
            }
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test nextMatrix3D(Random random, double minValue, double maxValue)
         double minValue = EuclidCoreRandomTools.nextDouble(random, 100.0);
         double maxValue = EuclidCoreRandomTools.nextDouble(random, minValue, 100.0);
         Matrix3D matrix3D = EuclidCoreRandomTools.nextMatrix3D(random, minValue, maxValue);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               assertTrue(matrix3D.getElement(row, column) <= maxValue);
               assertTrue(matrix3D.getElement(row, column) >= minValue);
               assertTrue(matrix3D.getElement(row, column) != 0.0);
               assertTrue(Double.isFinite(matrix3D.getElement(row, column)));
            }
         }
      }

      // Test exceptions:
      try
      {
         EuclidCoreRandomTools.nextMatrix3D(random, -0.1);
         fail("Should have thrown an exception.");
      }
      catch (RuntimeException e)
      {
         // Good
      }

      try
      {
         EuclidCoreRandomTools.nextMatrix3D(random, 0.1, 0.05);
         fail("Should have thrown an exception.");
      }
      catch (RuntimeException e)
      {
         // Good
      }

      try
      {
         EuclidCoreRandomTools.nextMatrix3D(random, -0.1, -0.15);
         fail("Should have thrown an exception.");
      }
      catch (RuntimeException e)
      {
         // Good
      }
   }

   @Test
   public void testNextDiagonalMatrix3D() throws Exception
   {
      Random random = new Random(23452);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test nextDiagonalMatrix3D(Random random)
         Matrix3D matrix3D = EuclidCoreRandomTools.nextDiagonalMatrix3D(random);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               if (row != column)
                  assertTrue(matrix3D.getElement(row, column) == 0.0);
               else
               {
                  assertTrue(matrix3D.getElement(row, column) <= 1.0);
                  assertTrue(matrix3D.getElement(row, column) >= -1.0);
                  assertTrue(matrix3D.getElement(row, column) != 0.0);
                  assertTrue(Double.isFinite(matrix3D.getElement(row, column)));
               }
            }
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test nextDiagonalMatrix3D(Random random, double minMaxValue)
         double minMaxValue = EuclidCoreRandomTools.nextDouble(random, 0.0, 100.0);
         Matrix3D matrix3D = EuclidCoreRandomTools.nextDiagonalMatrix3D(random, minMaxValue);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               if (row != column)
                  assertTrue(matrix3D.getElement(row, column) == 0.0);
               else
               {
                  assertTrue(matrix3D.getElement(row, column) <= minMaxValue);
                  assertTrue(matrix3D.getElement(row, column) >= -minMaxValue);
                  assertTrue(matrix3D.getElement(row, column) != 0.0);
                  assertTrue(Double.isFinite(matrix3D.getElement(row, column)));
               }
            }
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test nextDiagonalMatrix3D(Random random, double minValue, double maxValue)
         double minValue = EuclidCoreRandomTools.nextDouble(random, 100.0);
         double maxValue = EuclidCoreRandomTools.nextDouble(random, minValue, 100.0);
         Matrix3D matrix3D = EuclidCoreRandomTools.nextDiagonalMatrix3D(random, minValue, maxValue);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               if (row != column)
                  assertTrue(matrix3D.getElement(row, column) == 0.0);
               else
               {
                  assertTrue(matrix3D.getElement(row, column) <= maxValue);
                  assertTrue(matrix3D.getElement(row, column) >= minValue);
                  assertTrue(matrix3D.getElement(row, column) != 0.0);
                  assertTrue(Double.isFinite(matrix3D.getElement(row, column)));
               }
            }
         }
      }

      // Test exceptions:
      try
      {
         EuclidCoreRandomTools.nextDiagonalMatrix3D(random, -0.1);
         fail("Should have thrown an exception.");
      }
      catch (RuntimeException e)
      {
         // Good
      }

      try
      {
         EuclidCoreRandomTools.nextDiagonalMatrix3D(random, 0.1, 0.05);
         fail("Should have thrown an exception.");
      }
      catch (RuntimeException e)
      {
         // Good
      }

      try
      {
         EuclidCoreRandomTools.nextDiagonalMatrix3D(random, -0.1, -0.15);
         fail("Should have thrown an exception.");
      }
      catch (RuntimeException e)
      {
         // Good
      }
   }
}
