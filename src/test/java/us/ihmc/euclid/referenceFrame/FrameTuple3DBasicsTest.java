package us.ihmc.euclid.referenceFrame;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.EuclidTestConstants.*;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.function.Predicate;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.RandomMatrices;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools.FrameTypeBuilder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools.GenericTypeBuilder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Tuple3DBasicsTest;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public abstract class FrameTuple3DBasicsTest<F extends FrameTuple3DBasics> extends FrameTuple3DReadOnlyTest<F>
{
   public abstract Tuple3DBasics createRandomFramelessTuple(Random random);

   @Test
   public void testSet() throws Exception
   {
      Random random = new Random(5472);

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests set(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);

         Tuple3DBasics expected = createRandomFramelessTuple(random);

         int initialFrameIndex = random.nextInt(referenceFrames.length);
         ReferenceFrame initialFrame = referenceFrames[initialFrameIndex];
         F actual = createRandomFrameTuple(random, initialFrame);

         assertFalse(expected.epsilonEquals(actual, EPSILON));

         actual.set(initialFrame, expected);

         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
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
            EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests set(ReferenceFrame referenceFrame, double x, double y, double z)
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);

         Tuple3DBasics expected = createRandomFramelessTuple(random);

         int initialFrameIndex = random.nextInt(referenceFrames.length);
         ReferenceFrame initialFrame = referenceFrames[initialFrameIndex];
         F actual = createRandomFrameTuple(random, initialFrame);

         assertFalse(expected.epsilonEquals(actual, EPSILON));

         actual.set(initialFrame, expected.getX(), expected.getY(), expected.getZ());

         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
         assertEquals(initialFrame, actual.getReferenceFrame());

         actual.set(createRandomFramelessTuple(random));

         assertFalse(expected.epsilonEquals(actual, EPSILON));

         expected.set(actual);

         int differenceFrameIndex = initialFrameIndex + random.nextInt(referenceFrames.length - 1) + 1;
         differenceFrameIndex %= referenceFrames.length;
         ReferenceFrame differentFrame = referenceFrames[differenceFrameIndex];

         try
         {
            actual.set(differentFrame, random.nextDouble(), random.nextDouble(), random.nextDouble());
            fail("Should have thrown a ReferenceFrameMismatchException");
         }
         catch (ReferenceFrameMismatchException e)
         {
            // good
            EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
         }
      }
   }

   @Test
   public void testSetIncludingFrame() throws Exception
   {
      Random random = new Random(2342);

      ReferenceFrame initialFrame = ReferenceFrame.getWorldFrame();

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, double x, double y, double z)
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         F frameTuple = createRandomFrameTuple(random, initialFrame);
         Tuple3DBasics tuple = new Vector3D();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         frameTuple.setIncludingFrame(newFrame, x, y, z);
         tuple.set(x, y, z);
         assertEquals(newFrame, frameTuple.getReferenceFrame());
         EuclidCoreTestTools.assertTuple3DEquals(tuple, frameTuple, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly, double z)
         Tuple2DReadOnly input = EuclidCoreRandomTools.nextPoint2D(random);
         double z = random.nextDouble();
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         F frameTuple = createRandomFrameTuple(random, initialFrame);
         Tuple3DBasics tuple = new Vector3D();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         frameTuple.setIncludingFrame(newFrame, input, z);
         tuple.set(input, z);
         assertEquals(newFrame, frameTuple.getReferenceFrame());
         EuclidCoreTestTools.assertTuple3DEquals(tuple, frameTuple, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
         Tuple3DReadOnly input = EuclidCoreRandomTools.nextPoint3D(random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         F frameTuple = createRandomFrameTuple(random, initialFrame);
         Tuple3DBasics tuple = new Vector3D();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         frameTuple.setIncludingFrame(newFrame, input);
         tuple.set(input);
         assertEquals(newFrame, frameTuple.getReferenceFrame());
         EuclidCoreTestTools.assertTuple3DEquals(tuple, frameTuple, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, double[] tupleArray)
         double[] input = new double[random.nextInt(20)];
         for (int j = 0; j < input.length; j++)
            input[j] = random.nextDouble();
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         F frameTuple = createRandomFrameTuple(random, initialFrame);

         Tuple3DBasics tuple = new Vector3D();
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
            EuclidCoreTestTools.assertTuple3DEquals(tuple, frameTuple, EPSILON);
         }
         catch (Exception e)
         {
            if (expectedException == null)
               throw new AssertionError("Should not have thrown an exception.");
            String message = e.getMessage();
            if (!e.getClass().equals(expectedException.getClass()))
               throw new AssertionError("Unexpected exception:\nactual: " + e + "\nexpected: " + expectedException);
            else if (message != null && !message.equals(expectedException.getMessage())) // TODO Somehow it can happen (very rarely) that the message is null, which does not make much sense... Need to figure it out.
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

         Tuple3DBasics tuple = new Vector3D();
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
            EuclidCoreTestTools.assertTuple3DEquals(tuple, frameTuple, EPSILON);
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

         Tuple3DBasics tuple = new Vector3D();
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
            EuclidCoreTestTools.assertTuple3DEquals(tuple, frameTuple, EPSILON);
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

         Tuple3DBasics tuple = new Vector3D();
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
            EuclidCoreTestTools.assertTuple3DEquals(tuple, frameTuple, EPSILON);
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

         Tuple3DBasics tuple = new Vector3D();
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
            EuclidCoreTestTools.assertTuple3DEquals(tuple, frameTuple, EPSILON);
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
      { // Tests setIncludingFrame(FrameTuple2DReadOnly frameTuple2DReadOnly, double z)
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameTuple2DReadOnly input = EuclidFrameRandomTools.nextFramePoint2D(random, newFrame);
         double z = random.nextDouble();
         F frameTuple = createRandomFrameTuple(random, initialFrame);
         Tuple3DBasics tuple = new Vector3D();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         frameTuple.setIncludingFrame(input, z);
         tuple.set(input, z);
         assertEquals(newFrame, frameTuple.getReferenceFrame());
         EuclidCoreTestTools.assertTuple3DEquals(tuple, frameTuple, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(FrameTuple3DReadOnly other)
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameTuple3DReadOnly input = EuclidFrameRandomTools.nextFramePoint3D(random, newFrame);
         F frameTuple = createRandomFrameTuple(random, initialFrame);
         Tuple3DBasics tuple = new Vector3D();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         frameTuple.setIncludingFrame(input);
         tuple.set(input);
         assertEquals(newFrame, frameTuple.getReferenceFrame());
         EuclidCoreTestTools.assertTuple3DEquals(tuple, frameTuple, EPSILON);
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(234234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);

         Tuple3DBasics expectedGeometryObject = createRandomFramelessTuple(random);
         expectedGeometryObject.setToZero();

         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         F frameGeometryObject = createRandomFrameTuple(random, initialFrame);
         assertEquals(initialFrame, frameGeometryObject.getReferenceFrame());
         assertFalse(expectedGeometryObject.epsilonEquals(frameGeometryObject, EPSILON));
         frameGeometryObject.setToZero();
         EuclidCoreTestTools.assertTuple3DEquals(expectedGeometryObject, frameGeometryObject, EPSILON);

         frameGeometryObject = createRandomFrameTuple(random, initialFrame);
         ReferenceFrame newFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         assertEquals(initialFrame, frameGeometryObject.getReferenceFrame());
         assertFalse(expectedGeometryObject.epsilonEquals(frameGeometryObject, EPSILON));
         frameGeometryObject.setToZero(newFrame);
         assertEquals(newFrame, frameGeometryObject.getReferenceFrame());
         EuclidCoreTestTools.assertTuple3DEquals(expectedGeometryObject, frameGeometryObject, EPSILON);
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
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(frameGeometryObject);

         frameGeometryObject = createRandomFrameTuple(random, initialFrame);
         ReferenceFrame newFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         assertEquals(initialFrame, frameGeometryObject.getReferenceFrame());
         assertFalse(frameGeometryObject.containsNaN());
         frameGeometryObject.setToNaN(newFrame);
         assertEquals(newFrame, frameGeometryObject.getReferenceFrame());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(frameGeometryObject);
      }
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Random random = new Random(234);
      Predicate<Method> methodFilter = m -> !m.getName().contains("IncludingFrame") && !m.getName().contains("MatchingFrame")
            && !m.getName().contains("MatchingFrame") && !m.getName().equals("equals") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITestTools.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(frame -> createRandomFrameTuple(random, frame), methodFilter);
   }

   @Test
   public void testConsistencyWithTuple3D() throws Exception
   {
      Random random = new Random(3422);
      FrameTypeBuilder<? extends ReferenceFrameHolder> frameTypeBuilder = (frame, tuple) -> createFrameTuple(frame, (Tuple3DReadOnly) tuple);
      GenericTypeBuilder framelessTypeBuilber = () -> createRandomFramelessTuple(random);
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode");
      EuclidFrameAPITestTools.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, framelessTypeBuilber, methodFilter);
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameTuple3DBasics.class, Tuple3DBasics.class, true, 1, framelessMethodsToIgnore);
   }

   @Test
   public void testTuple3DBasicsFeatures() throws Exception
   {
      Tuple3DBasicsTest<F> tuple3dBasicsTest = new Tuple3DBasicsTest<F>()
      {
         @Override
         public F createEmptyTuple()
         {
            return FrameTuple3DBasicsTest.this.createEmptyFrameTuple();
         }

         @Override
         public F createTuple(double x, double y, double z)
         {
            return FrameTuple3DBasicsTest.this.createFrameTuple(x, y, z);
         }

         @Override
         public F createRandomTuple(Random random)
         {
            return FrameTuple3DBasicsTest.this.createRandomFrameTuple(random);
         }

         @Override
         public double getEpsilon()
         {
            return EPSILON;
         }
      };

      for (Method testMethod : tuple3dBasicsTest.getClass().getMethods())
      {
         if (!testMethod.getName().startsWith("test"))
            continue;
         if (!Modifier.isPublic(testMethod.getModifiers()))
            continue;
         if (Modifier.isStatic(testMethod.getModifiers()))
            continue;

         testMethod.invoke(tuple3dBasicsTest);
      }
   }
}
