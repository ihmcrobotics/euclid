package us.ihmc.euclid.referenceFrame;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.Vector4DBasicsTest;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

public class FrameVector4DTest extends FrameTuple4DTest<FrameVector4D, Vector4D>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Override
   public FrameVector4D createTuple(ReferenceFrame referenceFrame, double x, double y, double z, double s)
   {
      return new FrameVector4D(referenceFrame, x, y, z, s);
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-12;
   }

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(435345);

      { // Test FrameVector4D()
         FrameVector4D frameVector4D = new FrameVector4D();
         assertTrue(frameVector4D.referenceFrame == worldFrame);
         assertTrue(frameVector4D.getX() == 0.0);
         assertTrue(frameVector4D.getY() == 0.0);
         assertTrue(frameVector4D.getZ() == 0.0);
         assertTrue(frameVector4D.getS() == 0.0);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameVector4D(ReferenceFrame referenceFrame)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         FrameVector4D frameVector4D = new FrameVector4D(randomFrame);
         assertTrue(frameVector4D.referenceFrame == randomFrame);
         assertTrue(frameVector4D.getX() == 0.0);
         assertTrue(frameVector4D.getY() == 0.0);
         assertTrue(frameVector4D.getZ() == 0.0);
         assertTrue(frameVector4D.getS() == 0.0);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameVector4D(ReferenceFrame referenceFrame, double x, double y, double z, double s)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Vector4D randomTuple = EuclidCoreRandomTools.generateRandomVector4D(random);
         FrameVector4D frameVector4D = new FrameVector4D(randomFrame, randomTuple.getX(), randomTuple.getY(), randomTuple.getZ(), randomTuple.getS());
         assertTrue(frameVector4D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple4DEquals(randomTuple, frameVector4D, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameVector4D(ReferenceFrame referenceFrame, double[] pointArray)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Vector4D randomTuple = EuclidCoreRandomTools.generateRandomVector4D(random);
         double[] array = new double[4];
         randomTuple.get(array);
         FrameVector4D frameVector4D = new FrameVector4D(randomFrame, array);
         assertTrue(frameVector4D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple4DEquals(randomTuple, frameVector4D, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameVector4D(ReferenceFrame referenceFrame, Tuple4DReadOnly tuple3DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Vector4D randomTuple = EuclidCoreRandomTools.generateRandomVector4D(random);
         FrameVector4D frameVector4D = new FrameVector4D(randomFrame, randomTuple);
         assertTrue(frameVector4D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple4DEquals(randomTuple, frameVector4D, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameVector4D(FrameTuple4DReadOnly other)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         FrameVector4D randomTuple = EuclidFrameRandomTools.generateRandomFrameVector4D(random, randomFrame);
         FrameVector4D frameVector4D = new FrameVector4D(randomTuple);
         assertTrue(frameVector4D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple4DEquals(randomTuple, frameVector4D, EPSILON);
         EuclidFrameTestTools.assertFrameTuple4DEquals(randomTuple, frameVector4D, EPSILON);
      }
   }

   @Test
   public void testGetVector()
   {
      Random random = new Random(43535);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Vector4D expected = EuclidCoreRandomTools.generateRandomVector4D(random);
         FrameVector4D frameVector = new FrameVector4D(worldFrame, expected);
         Vector4DReadOnly actual = frameVector.getVector();
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPSILON);
         EuclidCoreTestTools.assertTuple4DEquals(frameVector, actual, EPSILON);
      }
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("set", new Class<?>[] {Vector4D.class});
      framelessMethodsToIgnore.put("epsilonEquals", new Class<?>[] {Vector4D.class, Double.TYPE});
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameVector4D.class, Vector4D.class, true, 1, framelessMethodsToIgnore);
   }

   @Test
   public void testGeometricallyEquals() {
      Random random = new Random(58722L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++) {
         double epsilon = random.nextDouble();

         ReferenceFrame referenceFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         FrameVector4D fv = EuclidFrameRandomTools.generateRandomFrameVector4D(random, referenceFrame);
         FrameVector4D fv0 = EuclidFrameRandomTools.generateRandomFrameVector4D(random, referenceFrame);

         if (fv.getVector().geometricallyEquals(fv0.getVector(), epsilon)) {
            assertTrue(fv.geometricallyEquals(fv0, epsilon));
         } else {
            assertFalse(fv.geometricallyEquals(fv0, epsilon));
         }
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++) {
         double epsilon = random.nextDouble();

         ReferenceFrame referenceFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         ReferenceFrame referenceFrame0 = EuclidFrameRandomTools.generateRandomReferenceFrame(random);

         FrameVector4D fv = EuclidFrameRandomTools.generateRandomFrameVector4D(random, referenceFrame);
         FrameVector4D fv0 = EuclidFrameRandomTools.generateRandomFrameVector4D(random, referenceFrame0);

         try {
            fv.geometricallyEquals(fv0, epsilon);
            fail();
         } catch (ReferenceFrameMismatchException ignored) {

         }
      }
   }

   @Test
   public void testFrameGeometryObjectFeatures() throws Throwable
   {
      FrameGeometryObjectTest<FrameVector4D, Vector4D> frameGeometryObjectTest = new FrameGeometryObjectTest<FrameVector4D, Vector4D>()
      {
         @Override
         public Vector4D createEmptyGeometryObject()
         {
            return createEmptyTuple().getGeometryObject();
         }

         @Override
         public Vector4D createRandomGeometryObject(Random random)
         {
            return createRandomTuple(random).getGeometryObject();
         }

         @Override
         public FrameVector4D createEmptyFrameGeometryObject(ReferenceFrame referenceFrame)
         {
            return createEmptyTuple(referenceFrame);
         }

         @Override
         public FrameVector4D createFrameGeometryObject(ReferenceFrame referenceFrame, Vector4D geometryObject)
         {
            return createTuple(referenceFrame, geometryObject);
         }

         @Override
         public FrameVector4D createRandomFrameGeometryObject(Random random, ReferenceFrame referenceFrame)
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
   public void testVectorBasicsFeatures() throws Exception
   {
      Vector4DBasicsTest<FrameVector4D> vectorBasicsTest = new Vector4DBasicsTest<FrameVector4D>()
      {
         @Override
         public FrameVector4D createEmptyTuple()
         {
            return FrameVector4DTest.this.createEmptyTuple();
         }

         @Override
         public FrameVector4D createRandomTuple(Random random)
         {
            return FrameVector4DTest.this.createRandomTuple(random);
         }

         @Override
         public FrameVector4D createTuple(double x, double y, double z, double s)
         {
            return FrameVector4DTest.this.createTuple(x, y, z, s);
         }

         @Override
         public double getEpsilon()
         {
            return FrameVector4DTest.this.getEpsilon();
         }
      };

      for (Method testMethod : vectorBasicsTest.getClass().getMethods())
      {
         if (!testMethod.getName().startsWith("test"))
            continue;
         if (!Modifier.isPublic(testMethod.getModifiers()))
            continue;
         if (Modifier.isStatic(testMethod.getModifiers()))
            continue;

         testMethod.invoke(vectorBasicsTest);
      }
   }
}
