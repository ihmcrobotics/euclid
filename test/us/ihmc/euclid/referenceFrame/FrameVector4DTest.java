package us.ihmc.euclid.referenceFrame;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.Vector4DBasicsTest;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DBasics;

public class FrameVector4DTest extends FrameTuple4DBasicsTest<FrameVector4D>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Override
   public Tuple4DBasics createRandomFramelessTuple(Random random)
   {
      return EuclidCoreRandomTools.nextVector4D(random);
   }

   @Override
   public FrameVector4D createFrameTuple(ReferenceFrame referenceFrame, double x, double y, double z, double s)
   {
      return new FrameVector4D(referenceFrame, x, y, z, s);
   }

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(435345);

      { // Test FrameVector4D()
         FrameVector4D frameVector4D = new FrameVector4D();
         assertTrue(frameVector4D.getReferenceFrame() == worldFrame);
         assertTrue(frameVector4D.getX() == 0.0);
         assertTrue(frameVector4D.getY() == 0.0);
         assertTrue(frameVector4D.getZ() == 0.0);
         assertTrue(frameVector4D.getS() == 0.0);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameVector4D(ReferenceFrame referenceFrame)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector4D frameVector4D = new FrameVector4D(randomFrame);
         assertTrue(frameVector4D.getReferenceFrame() == randomFrame);
         assertTrue(frameVector4D.getX() == 0.0);
         assertTrue(frameVector4D.getY() == 0.0);
         assertTrue(frameVector4D.getZ() == 0.0);
         assertTrue(frameVector4D.getS() == 0.0);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameVector4D(ReferenceFrame referenceFrame, double x, double y, double z, double s)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector4D randomTuple = EuclidCoreRandomTools.nextVector4D(random);
         FrameVector4D frameVector4D = new FrameVector4D(randomFrame, randomTuple.getX(), randomTuple.getY(), randomTuple.getZ(), randomTuple.getS());
         assertTrue(frameVector4D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple4DEquals(randomTuple, frameVector4D, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameVector4D(ReferenceFrame referenceFrame, double[] pointArray)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector4D randomTuple = EuclidCoreRandomTools.nextVector4D(random);
         double[] array = new double[4];
         randomTuple.get(array);
         FrameVector4D frameVector4D = new FrameVector4D(randomFrame, array);
         assertTrue(frameVector4D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple4DEquals(randomTuple, frameVector4D, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameVector4D(ReferenceFrame referenceFrame, Tuple4DReadOnly tuple3DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector4D randomTuple = EuclidCoreRandomTools.nextVector4D(random);
         FrameVector4D frameVector4D = new FrameVector4D(randomFrame, randomTuple);
         assertTrue(frameVector4D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple4DEquals(randomTuple, frameVector4D, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameVector4D(FrameTuple4DReadOnly other)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector4D randomTuple = EuclidFrameRandomTools.nextFrameVector4D(random, randomFrame);
         FrameVector4D frameVector4D = new FrameVector4D(randomTuple);
         assertTrue(frameVector4D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple4DEquals(randomTuple, frameVector4D, EPSILON);
         EuclidFrameTestTools.assertFrameTuple4DEquals(randomTuple, frameVector4D, EPSILON);
      }
   }

   @Test
   public void testChangeFrame() throws Exception
   {
      Random random = new Random(43563);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         ReferenceFrameUtils.clearWorldFrameTree();
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         ReferenceFrame anotherFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         Vector4D expected = EuclidCoreRandomTools.nextVector4D(random);
         FrameVector4D actual = new FrameVector4D(initialFrame, expected);

         RigidBodyTransform transform = initialFrame.getTransformToDesiredFrame(anotherFrame);
         expected.applyTransform(transform);

         actual.changeFrame(anotherFrame);
         assertTrue(anotherFrame == actual.getReferenceFrame());
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPSILON);

         ReferenceFrame differentRootFrame = ReferenceFrame.constructARootFrame("anotherRootFrame");
         try
         {
            actual.changeFrame(differentRootFrame);
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            // good
         }
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Random random = new Random(32120);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         FrameVector4D frameVector1 = EuclidFrameRandomTools.nextFrameVector4D(random, worldFrame);
         FrameVector4D frameVector2 = new FrameVector4D(worldFrame);
         double epsilon = random.nextDouble();
         Vector4D difference;

         difference = EuclidCoreRandomTools.nextVector4D(random);
         difference.scale(0.99 * epsilon / difference.norm());
         frameVector2.add(frameVector1, difference);
         assertTrue(frameVector1.geometricallyEquals(frameVector2, epsilon));

         difference = EuclidCoreRandomTools.nextVector4D(random);
         difference.scale(1.01 * epsilon / difference.norm());
         frameVector2.add(frameVector1, difference);
         assertFalse(frameVector1.geometricallyEquals(frameVector2, epsilon));
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(763);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Vector4D expected = EuclidCoreRandomTools.nextVector4D(random);
         expected.scale(1.0e15);
         FrameVector4D actual = new FrameVector4D(worldFrame, expected);

         assertEquals(expected.hashCode(), actual.hashCode());
      }
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("set", new Class<?>[] {Vector4D.class});
      framelessMethodsToIgnore.put("epsilonEquals", new Class<?>[] {Vector4D.class, Double.TYPE});
      framelessMethodsToIgnore.put("geometricallyEquals", new Class<?>[] {Vector4D.class, Double.TYPE});
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameVector4D.class, Vector4D.class, true, 1, framelessMethodsToIgnore);
   }

   @Test
   public void testVectorBasicsFeatures() throws Exception
   {
      Vector4DBasicsTest<FrameVector4D> vectorBasicsTest = new Vector4DBasicsTest<FrameVector4D>()
      {
         @Override
         public FrameVector4D createEmptyTuple()
         {
            return FrameVector4DTest.this.createEmptyFrameTuple();
         }

         @Override
         public FrameVector4D createRandomTuple(Random random)
         {
            return FrameVector4DTest.this.createRandomFrameTuple(random);
         }

         @Override
         public FrameVector4D createTuple(double x, double y, double z, double s)
         {
            return FrameVector4DTest.this.createFrameTuple(x, y, z, s);
         }

         @Override
         public double getEpsilon()
         {
            return EPSILON;
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
