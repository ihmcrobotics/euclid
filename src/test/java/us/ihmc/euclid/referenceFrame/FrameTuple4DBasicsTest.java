package us.ihmc.euclid.referenceFrame;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Random;
import java.util.function.Predicate;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.RandomMatrices;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPIDefaultConfiguration;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.FrameTypeCopier;
import us.ihmc.euclid.referenceFrame.api.RandomFramelessTypeBuilder;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple4DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple4DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple4D.Tuple4DBasicsTest;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

public abstract class FrameTuple4DBasicsTest<F extends FrameTuple4DBasics> extends FrameTuple4DReadOnlyTest<F>
{
   public static final double EPSILON = 1e-10;

   public abstract Tuple4DBasics createRandomFramelessTuple(Random random);

   public final F createTuple(ReferenceFrame referenceFrame)
   {
      return createFrameTuple(referenceFrame, 0.0, 0.0, 0.0, 0.0);
   }

   public final F createTuple(ReferenceFrame referenceFrame, Tuple4DReadOnly tuple)
   {
      return createFrameTuple(referenceFrame, tuple.getX(), tuple.getY(), tuple.getZ(), tuple.getS());
   }

   public final F createTuple(F frameTuple)
   {
      return createTuple(frameTuple.getReferenceFrame(), frameTuple);
   }

   @Override
   public final F createRandomFrameTuple(Random random, ReferenceFrame referenceFrame)
   {
      return createFrameTuple(referenceFrame, random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
   }

   @Test
   public void testSet() throws Exception
   {
      Random random = new Random(234234L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests set(ReferenceFrame referenceFrame, Tuple4DReadOnly tuple4DReadOnly)
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);

         Tuple4DBasics expected = createRandomFramelessTuple(random);

         int initialFrameIndex = random.nextInt(referenceFrames.length);
         ReferenceFrame initialFrame = referenceFrames[initialFrameIndex];
         F actual = createRandomFrameTuple(random, initialFrame);

         assertFalse(expected.epsilonEquals(actual, EPSILON));

         actual.set(initialFrame, expected);

         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPSILON);
         assertEquals(initialFrame, actual.getReferenceFrame());

         actual.set(createRandomFramelessTuple(random));

         assertFalse(expected.epsilonEquals(actual, EPSILON));

         expected.set(actual);

         int differenceFrameIndex = initialFrameIndex + random.nextInt(referenceFrames.length - 1) + 1;
         differenceFrameIndex %= referenceFrames.length;
         ReferenceFrame differentFrame = referenceFrames[differenceFrameIndex];

         try
         {
            actual.set(differentFrame, createRandomFramelessTuple(random));
            fail("Should have thrown a ReferenceFrameMismatchException");
         }
         catch (ReferenceFrameMismatchException e)
         {
            // good
            EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPSILON);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests set(ReferenceFrame referenceFrame, double x, double y, double z)
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);

         Tuple4DBasics expected = createRandomFramelessTuple(random);

         int initialFrameIndex = random.nextInt(referenceFrames.length);
         ReferenceFrame initialFrame = referenceFrames[initialFrameIndex];
         F actual = createRandomFrameTuple(random, initialFrame);

         assertFalse(expected.epsilonEquals(actual, EPSILON));

         actual.set(initialFrame, expected.getX(), expected.getY(), expected.getZ(), expected.getS());

         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPSILON);
         assertEquals(initialFrame, actual.getReferenceFrame());

         actual.set(createRandomFramelessTuple(random));

         assertFalse(expected.epsilonEquals(actual, EPSILON));

         expected.set(actual);

         int differenceFrameIndex = initialFrameIndex + random.nextInt(referenceFrames.length - 1) + 1;
         differenceFrameIndex %= referenceFrames.length;
         ReferenceFrame differentFrame = referenceFrames[differenceFrameIndex];

         try
         {
            actual.set(differentFrame, random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
            fail("Should have thrown a ReferenceFrameMismatchException");
         }
         catch (ReferenceFrameMismatchException e)
         {
            // good
            EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPSILON);
         }
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(234234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);

         Tuple4DBasics expectedGeometryObject = createRandomFramelessTuple(random);
         expectedGeometryObject.setToZero();

         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         F frameGeometryObject = createRandomFrameTuple(random, initialFrame);
         assertEquals(initialFrame, frameGeometryObject.getReferenceFrame());
         assertFalse(expectedGeometryObject.epsilonEquals(frameGeometryObject, EPSILON));
         frameGeometryObject.setToZero();
         EuclidCoreTestTools.assertTuple4DEquals(expectedGeometryObject, frameGeometryObject, EPSILON);

         frameGeometryObject = createRandomFrameTuple(random, initialFrame);
         ReferenceFrame newFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         assertEquals(initialFrame, frameGeometryObject.getReferenceFrame());
         assertFalse(expectedGeometryObject.epsilonEquals(frameGeometryObject, EPSILON));
         frameGeometryObject.setToZero(newFrame);
         assertEquals(newFrame, frameGeometryObject.getReferenceFrame());
         EuclidCoreTestTools.assertTuple4DEquals(expectedGeometryObject, frameGeometryObject, EPSILON);
      }
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(574);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);

         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         F frameGeometryObject = createRandomFrameTuple(random, initialFrame);
         assertEquals(initialFrame, frameGeometryObject.getReferenceFrame());
         assertFalse(frameGeometryObject.containsNaN());
         frameGeometryObject.setToNaN();
         EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(frameGeometryObject);

         frameGeometryObject = createRandomFrameTuple(random, initialFrame);
         ReferenceFrame newFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         assertEquals(initialFrame, frameGeometryObject.getReferenceFrame());
         assertFalse(frameGeometryObject.containsNaN());
         frameGeometryObject.setToNaN(newFrame);
         assertEquals(newFrame, frameGeometryObject.getReferenceFrame());
         EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(frameGeometryObject);
      }
   }

   @Test
   public void testSetIncludingFrame() throws Exception
   {
      Random random = new Random(2342);

      ReferenceFrame initialFrame = ReferenceFrame.getWorldFrame();

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, double x, double y, double z, double s)
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         double s = random.nextDouble();
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         F frameTuple = createRandomFrameTuple(random, initialFrame);
         Tuple4DBasics tuple = new Vector4D();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         frameTuple.setIncludingFrame(newFrame, x, y, z, s);
         tuple.set(x, y, z, s);
         assertEquals(newFrame, frameTuple.getReferenceFrame());
         EuclidCoreTestTools.assertTuple4DEquals(tuple, frameTuple, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, Tuple4DReadOnly tuple4DReadOnly)
         Tuple4DReadOnly input = EuclidCoreRandomTools.nextQuaternion(random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         F frameTuple = createRandomFrameTuple(random, initialFrame);
         Tuple4DBasics tuple = new Vector4D();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         frameTuple.setIncludingFrame(newFrame, input);
         tuple.set(input);
         assertEquals(newFrame, frameTuple.getReferenceFrame());
         EuclidCoreTestTools.assertTuple4DEquals(tuple, frameTuple, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, double[] tupleArray)
         double[] input = new double[random.nextInt(20)];
         for (int j = 0; j < input.length; j++)
            input[j] = random.nextDouble();
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         F frameTuple = createRandomFrameTuple(random, initialFrame);

         Tuple4DBasics tuple = new Vector4D();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         Exception expectedException = null;

         try
         {
            tuple.set(input);
         }
         catch (Exception e)
         {
            expectedException = e;
         }
         try
         {
            frameTuple.setIncludingFrame(newFrame, input);
            if (expectedException != null)
               throw new AssertionError("Should have thrown an exception.");

            assertEquals(newFrame, frameTuple.getReferenceFrame());
            EuclidCoreTestTools.assertTuple4DEquals(tuple, frameTuple, EPSILON);
         }
         catch (Exception e)
         {
            if (expectedException == null)
               throw new AssertionError("Should not have thrown an exception.");
            /*
             * Note: For some reason, from time to time e.getMessage() == null. No idea why as when debugging it
             * is never null -_-' So for now, let's only assert the class of the exception.
             */
            if (!e.getClass().equals(expectedException.getClass()))// || !e.getMessage().equals(expectedException.getMessage()))
               throw new AssertionError("Unexpected exception:\nactual: " + e + "\nexpected: " + expectedException);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, int startIndex, double[] tupleArray)
         int startIndex = random.nextInt(10);
         double[] input = new double[random.nextInt(20)];
         for (int j = 0; j < input.length; j++)
            input[j] = random.nextDouble();
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         F frameTuple = createRandomFrameTuple(random, initialFrame);

         Tuple4DBasics tuple = new Vector4D();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         Exception expectedException = null;

         try
         {
            tuple.set(startIndex, input);
         }
         catch (Exception e)
         {
            expectedException = e;
         }
         try
         {
            frameTuple.setIncludingFrame(newFrame, startIndex, input);
            if (expectedException != null)
               throw new AssertionError("Should have thrown an exception.");

            assertEquals(newFrame, frameTuple.getReferenceFrame());
            EuclidCoreTestTools.assertTuple4DEquals(tuple, frameTuple, EPSILON);
         }
         catch (Exception e)
         {
            if (expectedException == null)
               throw new AssertionError("Should not have thrown an exception.");
            /*
             * Note: For some reason, from time to time e.getMessage() == null. No idea why as when debugging it
             * is never null -_-' So for now, let's only assert the class of the exception.
             */
            if (!e.getClass().equals(expectedException.getClass()))// || !e.getMessage().equals(expectedException.getMessage()))
               throw new AssertionError("Unexpected exception:\nactual: " + e + "\nexpected: " + expectedException);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, DenseMatrix64F tupleDenseMatrix)
         DenseMatrix64F input = RandomMatrices.createRandom(random.nextInt(20), random.nextInt(20), random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         F frameTuple = createRandomFrameTuple(random, initialFrame);

         Tuple4DBasics tuple = new Vector4D();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         Exception expectedException = null;

         try
         {
            tuple.set(input);
         }
         catch (Exception e)
         {
            expectedException = e;
         }
         try
         {
            frameTuple.setIncludingFrame(newFrame, input);
            if (expectedException != null)
               throw new AssertionError("Should have thrown an exception.");

            assertEquals(newFrame, frameTuple.getReferenceFrame());
            EuclidCoreTestTools.assertTuple4DEquals(tuple, frameTuple, EPSILON);
         }
         catch (Exception e)
         {
            if (expectedException == null)
               throw new AssertionError("Should not have thrown an exception.");
            if (!e.getClass().equals(expectedException.getClass()) || !e.getMessage().equals(expectedException.getMessage()))
               throw new AssertionError("Unexpected exception:\nactual: " + e + "\nexpected: " + expectedException);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, int startRow, DenseMatrix64F tupleDenseMatrix)
         int startRow = random.nextInt(10);
         DenseMatrix64F input = RandomMatrices.createRandom(random.nextInt(20), random.nextInt(20), random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         F frameTuple = createRandomFrameTuple(random, initialFrame);

         Tuple4DBasics tuple = new Vector4D();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         Exception expectedException = null;

         try
         {
            tuple.set(startRow, input);
         }
         catch (Exception e)
         {
            expectedException = e;
         }
         try
         {
            frameTuple.setIncludingFrame(newFrame, startRow, input);
            if (expectedException != null)
               throw new AssertionError("Should have thrown an exception.");

            assertEquals(newFrame, frameTuple.getReferenceFrame());
            EuclidCoreTestTools.assertTuple4DEquals(tuple, frameTuple, EPSILON);
         }
         catch (Exception e)
         {
            if (expectedException == null)
               throw new AssertionError("Should not have thrown an exception.");
            if (!e.getClass().equals(expectedException.getClass()) || !e.getMessage().equals(expectedException.getMessage()))
               throw new AssertionError("Unexpected exception:\nactual: " + e + "\nexpected: " + expectedException);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, int startRow, int column, DenseMatrix64F tupleDenseMatrix)
         int startRow = random.nextInt(10);
         int column = random.nextInt(10);
         DenseMatrix64F input = RandomMatrices.createRandom(random.nextInt(20), random.nextInt(20), random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         F frameTuple = createRandomFrameTuple(random, initialFrame);

         Tuple4DBasics tuple = new Vector4D();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         Exception expectedException = null;

         try
         {
            tuple.set(startRow, column, input);
         }
         catch (Exception e)
         {
            expectedException = e;
         }
         try
         {
            frameTuple.setIncludingFrame(newFrame, startRow, column, input);
            if (expectedException != null)
               throw new AssertionError("Should have thrown an exception.");

            assertEquals(newFrame, frameTuple.getReferenceFrame());
            EuclidCoreTestTools.assertTuple4DEquals(tuple, frameTuple, EPSILON);
         }
         catch (Exception e)
         {
            if (expectedException == null)
               throw new AssertionError("Should not have thrown an exception.");
            if (!e.getClass().equals(expectedException.getClass()) || !e.getMessage().equals(expectedException.getMessage()))
               throw new AssertionError("Unexpected exception:\nactual: " + e + "\nexpected: " + expectedException);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(FrameTuple4DReadOnly other)
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame(random);

         FrameTuple4DReadOnly frameTupleReadOnly = createRandomFrameTuple(random, frameA);
         F frameTuple = createRandomFrameTuple(random, frameB);

         frameTuple.setIncludingFrame(frameTupleReadOnly);
         assertTrue(frameTupleReadOnly.getReferenceFrame() == frameTuple.getReferenceFrame());
         EuclidCoreTestTools.assertTuple4DEquals(frameTuple, frameTupleReadOnly, EPSILON);
         EuclidFrameTestTools.assertFrameTuple4DEquals(frameTuple, frameTupleReadOnly, EPSILON);
      }
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Predicate<Method> methodFilter = m -> !m.getName().equals("equals") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(this::createRandomFrameTuple,
                                                                    methodFilter,
                                                                    EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testConsistencyWithTuple4D() throws Exception
   {
      FrameTypeCopier frameTypeBuilder = (frame, tuple) -> createTuple(frame, (Tuple4DReadOnly) tuple);
      RandomFramelessTypeBuilder framelessTypeBuilber = this::createRandomFramelessTuple;
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode");
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder,
                                                                  framelessTypeBuilber,
                                                                  methodFilter,
                                                                  EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertOverloadingWithFrameObjects(FrameTuple4DBasics.class, Tuple4DBasics.class, true, 1);
   }

   @Test
   public void testTuple4DBasicsFeatures() throws Exception
   {
      Tuple4DBasicsTest<F> tuple4dBasicsTest = new Tuple4DBasicsTest<F>()
      {
         @Override
         public void testSetDoubles()
         {

         }

         @Override
         public F createEmptyTuple()
         {
            return FrameTuple4DBasicsTest.this.createEmptyFrameTuple();
         }

         @Override
         public F createTuple(double x, double y, double z, double s)
         {
            return FrameTuple4DBasicsTest.this.createFrameTuple(x, y, z, s);
         }

         @Override
         public F createRandomTuple(Random random)
         {
            return FrameTuple4DBasicsTest.this.createRandomFrameTuple(random);
         }

         @Override
         public double getEpsilon()
         {
            return EPSILON;
         }
      };

      for (Method testMethod : tuple4dBasicsTest.getClass().getMethods())
      {
         if (!testMethod.getName().startsWith("test"))
            continue;
         if (!Modifier.isPublic(testMethod.getModifiers()))
            continue;
         if (Modifier.isStatic(testMethod.getModifiers()))
            continue;

         testMethod.invoke(tuple4dBasicsTest);
      }
   }
}
