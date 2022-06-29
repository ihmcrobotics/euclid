package us.ihmc.euclid.referenceFrame;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPIDefaultConfiguration;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
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

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameVector4D(ReferenceFrame referenceFrame)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector4D frameVector4D = new FrameVector4D(randomFrame);
         assertTrue(frameVector4D.getReferenceFrame() == randomFrame);
         assertTrue(frameVector4D.getX() == 0.0);
         assertTrue(frameVector4D.getY() == 0.0);
         assertTrue(frameVector4D.getZ() == 0.0);
         assertTrue(frameVector4D.getS() == 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameVector4D(ReferenceFrame referenceFrame, double x, double y, double z, double s)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector4D randomTuple = EuclidCoreRandomTools.nextVector4D(random);
         FrameVector4D frameVector4D = new FrameVector4D(randomFrame, randomTuple.getX(), randomTuple.getY(), randomTuple.getZ(), randomTuple.getS());
         assertTrue(frameVector4D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomTuple, frameVector4D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameVector4D(ReferenceFrame referenceFrame, double[] pointArray)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector4D randomTuple = EuclidCoreRandomTools.nextVector4D(random);
         double[] array = new double[4];
         randomTuple.get(array);
         FrameVector4D frameVector4D = new FrameVector4D(randomFrame, array);
         assertTrue(frameVector4D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomTuple, frameVector4D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameVector4D(ReferenceFrame referenceFrame, Tuple4DReadOnly tuple3DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector4D randomTuple = EuclidCoreRandomTools.nextVector4D(random);
         FrameVector4D frameVector4D = new FrameVector4D(randomFrame, randomTuple);
         assertTrue(frameVector4D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomTuple, frameVector4D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameVector4D(FrameTuple4DReadOnly other)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector4D randomTuple = EuclidFrameRandomTools.nextFrameVector4D(random, randomFrame);
         FrameVector4D frameVector4D = new FrameVector4D(randomTuple);
         assertTrue(frameVector4D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomTuple, frameVector4D, EPSILON);
         EuclidCoreTestTools.assertEquals(randomTuple, frameVector4D, EPSILON);
      }
   }

   @Test
   public void testSetMatchingFrame()
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertSetMatchingFramePreserveFunctionality(EuclidFrameRandomTools::nextFrameVector4D, EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }

   @Override
   @Test
   public void testSetIncludingFrame()
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertSetIncludingFramePreserveFunctionality(EuclidFrameRandomTools::nextFrameVector4D, EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }

   @Test
   public void testChangeFrame() throws Exception
   {
      Random random = new Random(43563);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         ReferenceFrame anotherFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         Vector4D expected = EuclidCoreRandomTools.nextVector4D(random);
         FrameVector4D actual = new FrameVector4D(initialFrame, expected);

         RigidBodyTransform transform = initialFrame.getTransformToDesiredFrame(anotherFrame);
         expected.applyTransform(transform);

         actual.changeFrame(anotherFrame);
         assertTrue(anotherFrame == actual.getReferenceFrame());
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);

         ReferenceFrame differentRootFrame = ReferenceFrameTools.constructARootFrame("anotherRootFrame");
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

      for (int i = 0; i < ITERATIONS; i++)
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
      Random random = new Random(621541L);
      ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random, 100);

      FrameVector4D tuple = new FrameVector4D();

      tuple.setX(random.nextDouble());
      tuple.setY(random.nextDouble());
      tuple.setZ(random.nextDouble());
      tuple.setS(random.nextDouble());
      tuple.setReferenceFrame(frames[random.nextInt(frames.length)]);

      int newHashCode, previousHashCode;
      newHashCode = tuple.hashCode();
      assertEquals(newHashCode, tuple.hashCode());

      previousHashCode = tuple.hashCode();

      for (int i = 0; i < ITERATIONS; i++)
      {
         tuple.setElement(i % 4, random.nextDouble());
         newHashCode = tuple.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;

         ReferenceFrame oldFrame = tuple.getReferenceFrame();
         ReferenceFrame newFrame = frames[random.nextInt(frames.length)];
         tuple.setReferenceFrame(newFrame);
         newHashCode = tuple.hashCode();
         if (oldFrame != newFrame)
            assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("set", Vector4D.class));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", Vector4D.class, Double.TYPE));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", Vector4D.class, Double.TYPE));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertOverloadingWithFrameObjects(FrameVector4D.class, Vector4D.class, true, 1, methodFilter);
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
