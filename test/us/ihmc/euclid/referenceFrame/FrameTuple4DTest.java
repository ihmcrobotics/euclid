package us.ihmc.euclid.referenceFrame;

import static org.junit.Assert.assertEquals;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.function.Predicate;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.RandomMatrices;
import org.junit.Test;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools.FrameTypeBuilder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools.GenericTypeBuilder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple4D.Tuple4DBasicsTest;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

public abstract class FrameTuple4DTest<F extends FrameTuple4D<F, T>, T extends Tuple4DBasics & GeometryObject<T>> extends FrameTuple4DReadOnlyTest<F>
{
   public static final double EPSILON = 1e-10;

   public final F createTuple(ReferenceFrame referenceFrame)
   {
      return createTuple(referenceFrame, 0.0, 0.0, 0.0, 0.0);
   }

   public final F createTuple(ReferenceFrame referenceFrame, Tuple4DReadOnly tuple)
   {
      return createTuple(referenceFrame, tuple.getX(), tuple.getY(), tuple.getZ(), tuple.getS());
   }

   public final F createTuple(F frameTuple)
   {
      return createTuple(frameTuple.getReferenceFrame(), frameTuple);
   }

   public final F createRandomTuple(Random random, ReferenceFrame referenceFrame)
   {
      return createTuple(referenceFrame, random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
   }

   @Test
   public void testSetIncludingFrame() throws Exception
   {
      Random random = new Random(2342);

      ReferenceFrame initialFrame = ReferenceFrame.getWorldFrame();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, double x, double y, double z, double s)
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         double s = random.nextDouble();
         ReferenceFrame newFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         F frameTuple = createRandomTuple(random, initialFrame);
         Tuple4DBasics tuple = new Vector4D();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         frameTuple.setIncludingFrame(newFrame, x, y, z, s);
         tuple.set(x, y, z, s);
         assertEquals(newFrame, frameTuple.getReferenceFrame());
         EuclidCoreTestTools.assertTuple4DEquals(tuple, frameTuple, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, Tuple4DReadOnly tuple4DReadOnly)
         Tuple4DReadOnly input = EuclidCoreRandomTools.generateRandomQuaternion(random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         F frameTuple = createRandomTuple(random, initialFrame);
         Tuple4DBasics tuple = new Vector4D();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         frameTuple.setIncludingFrame(newFrame, input);
         tuple.set(input);
         assertEquals(newFrame, frameTuple.getReferenceFrame());
         EuclidCoreTestTools.assertTuple4DEquals(tuple, frameTuple, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, double[] tupleArray)
         double[] input = new double[random.nextInt(20)];
         for (int j = 0; j < input.length; j++)
            input[j] = random.nextDouble();
         ReferenceFrame newFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         F frameTuple = createRandomTuple(random, initialFrame);

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
            if (!e.getClass().equals(expectedException.getClass()) || !e.getMessage().equals(e.getMessage()))
               throw new AssertionError("Unexpected exception:\nactual: " + e + "\nexpected: " + expectedException);
         }
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, int startIndex, double[] tupleArray)
         int startIndex = random.nextInt(10);
         double[] input = new double[random.nextInt(20)];
         for (int j = 0; j < input.length; j++)
            input[j] = random.nextDouble();
         ReferenceFrame newFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         F frameTuple = createRandomTuple(random, initialFrame);

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
            if (!e.getClass().equals(expectedException.getClass()) || !e.getMessage().equals(e.getMessage()))
               throw new AssertionError("Unexpected exception:\nactual: " + e + "\nexpected: " + expectedException);
         }
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, DenseMatrix64F tupleDenseMatrix)
         DenseMatrix64F input = RandomMatrices.createRandom(random.nextInt(20), random.nextInt(20), random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         F frameTuple = createRandomTuple(random, initialFrame);

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
            if (!e.getClass().equals(expectedException.getClass()) || !e.getMessage().equals(e.getMessage()))
               throw new AssertionError("Unexpected exception:\nactual: " + e + "\nexpected: " + expectedException);
         }
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, int startRow, DenseMatrix64F tupleDenseMatrix)
         int startRow = random.nextInt(10);
         DenseMatrix64F input = RandomMatrices.createRandom(random.nextInt(20), random.nextInt(20), random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         F frameTuple = createRandomTuple(random, initialFrame);

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
            if (!e.getClass().equals(expectedException.getClass()) || !e.getMessage().equals(e.getMessage()))
               throw new AssertionError("Unexpected exception:\nactual: " + e + "\nexpected: " + expectedException);
         }
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, int startRow, int column, DenseMatrix64F tupleDenseMatrix)
         int startRow = random.nextInt(10);
         int column = random.nextInt(10);
         DenseMatrix64F input = RandomMatrices.createRandom(random.nextInt(20), random.nextInt(20), random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         F frameTuple = createRandomTuple(random, initialFrame);

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
            if (!e.getClass().equals(expectedException.getClass()) || !e.getMessage().equals(e.getMessage()))
               throw new AssertionError("Unexpected exception:\nactual: " + e + "\nexpected: " + expectedException);
         }
      }
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Random random = new Random(234);
      Predicate<Method> methodFilter = m -> !m.getName().contains("IncludingFrame") && !m.getName().equals("equals") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITestTools.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(frame -> createRandomTuple(random, frame), false, true, methodFilter);
   }

   @Test
   public void testConsistencyWithTuple4D() throws Exception
   {
      Random random = new Random(3422);
      FrameTypeBuilder<? extends ReferenceFrameHolder> frameTypeBuilder = (frame, tuple) -> createTuple(frame, (Tuple4DReadOnly) tuple);
      GenericTypeBuilder framelessTypeBuilber = () -> createRandomTuple(random).getGeometryObject();
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode");
      EuclidFrameAPITestTools.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, framelessTypeBuilber, methodFilter);
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameTuple4D.class, Tuple4DBasics.class, true, 1, framelessMethodsToIgnore);
   }

   @Test
   public void testFrameGeometryObjectFeatures() throws Throwable
   {
      FrameGeometryObjectTest<F, T> frameGeometryObjectTest = new FrameGeometryObjectTest<F, T>()
      {
         @Override
         public T createEmptyGeometryObject()
         {
            return createEmptyTuple().getGeometryObject();
         }

         @Override
         public T createRandomGeometryObject(Random random)
         {
            return createRandomTuple(random).getGeometryObject();
         }

         @Override
         public F createEmptyFrameGeometryObject(ReferenceFrame referenceFrame)
         {
            return createEmptyTuple(referenceFrame);
         }

         @Override
         public F createFrameGeometryObject(ReferenceFrame referenceFrame, T geometryObject)
         {
            return createTuple(referenceFrame, geometryObject);
         }

         @Override
         public F createRandomFrameGeometryObject(Random random, ReferenceFrame referenceFrame)
         {
            return createRandomTuple(random, referenceFrame);
         }
      };

      for (Method testMethod : frameGeometryObjectTest.getClass().getMethods())
      {
         if (!testMethod.getName().startsWith("test"))
            continue;
         if (!Modifier.isPublic(testMethod.getModifiers()))
            continue;
         if (Modifier.isStatic(testMethod.getModifiers()))
            continue;

         try
         {
            testMethod.invoke(frameGeometryObjectTest);
         }
         catch (InvocationTargetException e)
         {
            throw e.getCause();
         }
      }
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
            return FrameTuple4DTest.this.createEmptyTuple();
         }

         @Override
         public F createTuple(double x, double y, double z, double s)
         {
            return FrameTuple4DTest.this.createTuple(x, y, z, s);
         }

         @Override
         public F createRandomTuple(Random random)
         {
            return FrameTuple4DTest.this.createRandomTuple(random);
         }

         @Override
         public double getEpsilon()
         {
            return FrameTuple4DTest.this.getEpsilon();
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
