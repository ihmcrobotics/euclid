package us.ihmc.euclid.referenceFrame;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.function.Predicate;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.RandomMatrices;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester.FrameTypeBuilder;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester.GenericTypeBuilder;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Tuple2DBasicsTest;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public abstract class FrameTuple2DBasicsTest<F extends FrameTuple2DBasics> extends FrameTuple2DReadOnlyTest<F>
{
   public abstract Tuple2DBasics createRandomFramelessTuple(Random random);

   @Test
   public void testSet() throws Exception
   {
      Random random = new Random(5472);

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests set(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);

         Tuple2DBasics expected = createRandomFramelessTuple(random);

         int initialFrameIndex = random.nextInt(referenceFrames.length);
         ReferenceFrame initialFrame = referenceFrames[initialFrameIndex];
         F actual = createRandomFrameTuple(random, initialFrame);

         assertFalse(expected.epsilonEquals(actual, EPSILON));

         actual.set(initialFrame, expected);

         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);
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
            EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests set(ReferenceFrame referenceFrame, double x, double y)
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);

         Tuple2DBasics expected = createRandomFramelessTuple(random);

         int initialFrameIndex = random.nextInt(referenceFrames.length);
         ReferenceFrame initialFrame = referenceFrames[initialFrameIndex];
         F actual = createRandomFrameTuple(random, initialFrame);

         assertFalse(expected.epsilonEquals(actual, EPSILON));

         actual.set(initialFrame, expected.getX(), expected.getY());

         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);
         assertEquals(initialFrame, actual.getReferenceFrame());

         actual.set(createRandomFramelessTuple(random));

         assertFalse(expected.epsilonEquals(actual, EPSILON));

         expected.set(actual);

         int differenceFrameIndex = initialFrameIndex + random.nextInt(referenceFrames.length - 1) + 1;
         differenceFrameIndex %= referenceFrames.length;
         ReferenceFrame differentFrame = referenceFrames[differenceFrameIndex];

         try
         {
            actual.set(differentFrame, random.nextDouble(), random.nextDouble());
            fail("Should have thrown a ReferenceFrameMismatchException");
         }
         catch (ReferenceFrameMismatchException e)
         {
            // good
            EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);
         }
      }
   }

   @Test
   public void testSetIncludingFrame() throws Exception
   {
      Random random = new Random(2342);

      ReferenceFrame initialFrame = ReferenceFrame.getWorldFrame();

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, double x, double y)
         double x = random.nextDouble();
         double y = random.nextDouble();
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         F frameTuple = createRandomFrameTuple(random, initialFrame);
         Tuple2DBasics tuple = new Vector2D();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         frameTuple.setIncludingFrame(newFrame, x, y);
         tuple.set(x, y);
         assertEquals(newFrame, frameTuple.getReferenceFrame());
         EuclidCoreTestTools.assertTuple2DEquals(tuple, frameTuple, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
         Tuple2DReadOnly input = EuclidCoreRandomTools.nextPoint2D(random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         F frameTuple = createRandomFrameTuple(random, initialFrame);
         Tuple2DBasics tuple = new Vector2D();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         frameTuple.setIncludingFrame(newFrame, input);
         tuple.set(input);
         assertEquals(newFrame, frameTuple.getReferenceFrame());
         EuclidCoreTestTools.assertTuple2DEquals(tuple, frameTuple, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
         Tuple3DReadOnly input = EuclidCoreRandomTools.nextPoint3D(random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         F frameTuple = createRandomFrameTuple(random, initialFrame);
         Tuple2DBasics tuple = new Vector2D();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         frameTuple.setIncludingFrame(newFrame, input);
         tuple.set(input);
         assertEquals(newFrame, frameTuple.getReferenceFrame());
         EuclidCoreTestTools.assertTuple2DEquals(tuple, frameTuple, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, double[] tupleArray)
         double[] input = new double[random.nextInt(20)];
         for (int j = 0; j < input.length; j++)
            input[j] = random.nextDouble();
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         F frameTuple = createRandomFrameTuple(random, initialFrame);

         Tuple2DBasics tuple = new Vector2D();
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
            EuclidCoreTestTools.assertTuple2DEquals(tuple, frameTuple, EPSILON);
         }
         catch (Exception e)
         {
            if (expectedException == null)
               throw new AssertionError("Should not have thrown an exception.");
            if (!e.getClass().equals(expectedException.getClass()) || e.getMessage() != null && !e.getMessage().equals(expectedException.getMessage()))
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

         Tuple2DBasics tuple = new Vector2D();
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
            EuclidCoreTestTools.assertTuple2DEquals(tuple, frameTuple, EPSILON);
         }
         catch (Exception e)
         {
            if (expectedException == null)
               throw new AssertionError("Should not have thrown an exception.");
            if (!e.getClass().equals(expectedException.getClass()) || e.getMessage() != null && !e.getMessage().equals(expectedException.getMessage()))
               throw new AssertionError("Unexpected exception:\nactual: " + e + "\nexpected: " + expectedException);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, DenseMatrix64F tupleDenseMatrix)
         DenseMatrix64F input = RandomMatrices.createRandom(random.nextInt(20), random.nextInt(20), random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         F frameTuple = createRandomFrameTuple(random, initialFrame);

         Tuple2DBasics tuple = new Vector2D();
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
            EuclidCoreTestTools.assertTuple2DEquals(tuple, frameTuple, EPSILON);
         }
         catch (Exception e)
         {
            if (expectedException == null)
               throw new AssertionError("Should not have thrown an exception.");
            if (!e.getClass().equals(expectedException.getClass()) || e.getMessage() != null && !e.getMessage().equals(expectedException.getMessage()))
               throw new AssertionError("Unexpected exception:\nactual: " + e + "\nexpected: " + expectedException);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, int startRow, DenseMatrix64F tupleDenseMatrix)
         int startRow = random.nextInt(10);
         DenseMatrix64F input = RandomMatrices.createRandom(random.nextInt(20), random.nextInt(20), random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         F frameTuple = createRandomFrameTuple(random, initialFrame);

         Tuple2DBasics tuple = new Vector2D();
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
            EuclidCoreTestTools.assertTuple2DEquals(tuple, frameTuple, EPSILON);
         }
         catch (Exception e)
         {
            if (expectedException == null)
               throw new AssertionError("Should not have thrown an exception.");
            if (!e.getClass().equals(expectedException.getClass()) || e.getMessage() != null && !e.getMessage().equals(expectedException.getMessage()))
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

         Tuple2DBasics tuple = new Vector2D();
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
            EuclidCoreTestTools.assertTuple2DEquals(tuple, frameTuple, EPSILON);
         }
         catch (Exception e)
         {
            if (expectedException == null)
               throw new AssertionError("Should not have thrown an exception.");
            if (!e.getClass().equals(expectedException.getClass()) || e.getMessage() != null && !e.getMessage().equals(expectedException.getMessage()))
               throw new AssertionError("Unexpected exception:\nactual: " + e + "\nexpected: " + expectedException);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(FrameTuple2DReadOnly other)
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameTuple2DReadOnly input = EuclidFrameRandomTools.nextFramePoint2D(random, newFrame);
         F frameTuple = createRandomFrameTuple(random, initialFrame);
         Tuple2DBasics tuple = new Vector2D();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         frameTuple.setIncludingFrame(input);
         tuple.set(input);
         assertEquals(newFrame, frameTuple.getReferenceFrame());
         EuclidCoreTestTools.assertTuple2DEquals(tuple, frameTuple, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(FrameTuple3DReadOnly other)
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameTuple3DReadOnly input = EuclidFrameRandomTools.nextFramePoint3D(random, newFrame);
         F frameTuple = createRandomFrameTuple(random, initialFrame);
         Tuple2DBasics tuple = new Vector2D();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         frameTuple.setIncludingFrame(input);
         tuple.set(input);
         assertEquals(newFrame, frameTuple.getReferenceFrame());
         EuclidCoreTestTools.assertTuple2DEquals(tuple, frameTuple, EPSILON);
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(234234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);

         Tuple2DBasics expectedGeometryObject = createRandomFramelessTuple(random);
         expectedGeometryObject.setToZero();

         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         F frameGeometryObject = createRandomFrameTuple(random, initialFrame);
         assertEquals(initialFrame, frameGeometryObject.getReferenceFrame());
         assertFalse(expectedGeometryObject.epsilonEquals(frameGeometryObject, EPSILON));
         frameGeometryObject.setToZero();
         EuclidCoreTestTools.assertTuple2DEquals(expectedGeometryObject, frameGeometryObject, EPSILON);

         frameGeometryObject = createRandomFrameTuple(random, initialFrame);
         ReferenceFrame newFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         assertEquals(initialFrame, frameGeometryObject.getReferenceFrame());
         assertFalse(expectedGeometryObject.epsilonEquals(frameGeometryObject, EPSILON));
         frameGeometryObject.setToZero(newFrame);
         assertEquals(newFrame, frameGeometryObject.getReferenceFrame());
         EuclidCoreTestTools.assertTuple2DEquals(expectedGeometryObject, frameGeometryObject, EPSILON);
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
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(frameGeometryObject);

         frameGeometryObject = createRandomFrameTuple(random, initialFrame);
         ReferenceFrame newFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         assertEquals(initialFrame, frameGeometryObject.getReferenceFrame());
         assertFalse(frameGeometryObject.containsNaN());
         frameGeometryObject.setToNaN(newFrame);
         assertEquals(newFrame, frameGeometryObject.getReferenceFrame());
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(frameGeometryObject);
      }
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FrameTuple2DBasics.class, Tuple2DBasics.class, true, 1, framelessMethodsToIgnore);
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Predicate<Method> methodFilter = m -> !m.getName().contains("IncludingFrame") && !m.getName().contains("MatchingFrame") && !m.getName().equals("equals")
            && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(this::createRandomFrameTuple, methodFilter);
   }

   @Test
   public void testConsistencyWithTuple2D() throws Exception
   {
      FrameTypeBuilder<? extends ReferenceFrameHolder> frameTypeBuilder = (frame, tuple) -> createFrameTuple(frame, (Tuple2DReadOnly) tuple);
      GenericTypeBuilder framelessTypeBuilber = this::createRandomFramelessTuple;
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode");
      EuclidFrameAPITester.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, framelessTypeBuilber, methodFilter);
   }

   @Test
   public void testTuple2DBasicsFeatures() throws Exception
   {
      Tuple2DBasicsTest<F> tuple2dBasicsTest = new Tuple2DBasicsTest<F>()
      {
         @Override
         public F createEmptyTuple()
         {
            return FrameTuple2DBasicsTest.this.createEmptyFrameTuple();
         }

         @Override
         public F createTuple(double x, double y)
         {
            return FrameTuple2DBasicsTest.this.createFrameTuple(x, y);
         }

         @Override
         public F createRandomTuple(Random random)
         {
            return FrameTuple2DBasicsTest.this.createRandomFrameTuple(random);
         }

         @Override
         public double getEpsilon()
         {
            return EPSILON;
         }
      };

      for (Method testMethod : tuple2dBasicsTest.getClass().getMethods())
      {
         if (!testMethod.getName().startsWith("test"))
            continue;
         if (!Modifier.isPublic(testMethod.getModifiers()))
            continue;
         if (Modifier.isStatic(testMethod.getModifiers()))
            continue;

         testMethod.invoke(tuple2dBasicsTest);
      }
   }
}
