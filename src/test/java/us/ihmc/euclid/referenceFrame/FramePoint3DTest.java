package us.ihmc.euclid.referenceFrame;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPIDefaultConfiguration;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;

public class FramePoint3DTest extends FrameTuple3DBasicsTest<FramePoint3D>
{
   public static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Override
   public Tuple3DBasics createRandomFramelessTuple(Random random)
   {
      return EuclidCoreRandomTools.nextPoint3D(random);
   }

   @Override
   public FramePoint3D createFrameTuple(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      return new FramePoint3D(referenceFrame, x, y, z);
   }

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(435345);

      { // Test FramePoint3D()
         FramePoint3D framePoint3D = new FramePoint3D();
         assertTrue(framePoint3D.getReferenceFrame() == worldFrame);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(framePoint3D);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FramePoint3D(ReferenceFrame referenceFrame)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FramePoint3D framePoint3D = new FramePoint3D(randomFrame);
         assertTrue(framePoint3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(framePoint3D);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FramePoint3D(ReferenceFrame referenceFrame, double x, double y, double z)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Point3D randomTuple = EuclidCoreRandomTools.nextPoint3D(random);
         FramePoint3D framePoint3D = new FramePoint3D(randomFrame, randomTuple.getX(), randomTuple.getY(), randomTuple.getZ());
         assertTrue(framePoint3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomTuple, framePoint3D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FramePoint3D(ReferenceFrame referenceFrame, double[] pointArray)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Point3D randomTuple = EuclidCoreRandomTools.nextPoint3D(random);
         double[] array = new double[3];
         randomTuple.get(array);
         FramePoint3D framePoint3D = new FramePoint3D(randomFrame, array);
         assertTrue(framePoint3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomTuple, framePoint3D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FramePoint3D(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Point3D randomTuple = EuclidCoreRandomTools.nextPoint3D(random);
         FramePoint3D framePoint3D = new FramePoint3D(randomFrame, randomTuple);
         assertTrue(framePoint3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomTuple, framePoint3D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FramePoint3D(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Point2D randomTuple2D = EuclidCoreRandomTools.nextPoint2D(random);
         FramePoint3D framePoint3D = new FramePoint3D(randomFrame, randomTuple2D);
         assertTrue(framePoint3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomTuple2D, new Point2D(framePoint3D), EPSILON);
         assertTrue(framePoint3D.getZ() == 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FramePoint3D(FrameTuple2DReadOnly frameTuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FramePoint2D randomFrameTuple2D = EuclidFrameRandomTools.nextFramePoint2D(random, randomFrame);
         FramePoint3D framePoint3D = new FramePoint3D(randomFrameTuple2D);
         assertTrue(framePoint3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomFrameTuple2D, new Point2D(framePoint3D), EPSILON);
         assertTrue(framePoint3D.getZ() == 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FramePoint3D(FrameTuple3DReadOnly other)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FramePoint3D randomTuple = EuclidFrameRandomTools.nextFramePoint3D(random, randomFrame);
         FramePoint3D framePoint3D = new FramePoint3D(randomTuple);
         assertTrue(framePoint3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomTuple, framePoint3D, EPSILON);
         EuclidFrameTestTools.assertEquals(randomTuple, framePoint3D, EPSILON);
      }
   }

   @Test
   public void testSetMatchingFrame() throws Exception
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertSetMatchingFramePreserveFunctionality(EuclidFrameRandomTools::nextFramePoint3D, EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

      Random random = new Random(544354);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setMatchingFrame(FrameTuple3DReadOnly other)
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);

         FrameTuple3DReadOnly source = EuclidFrameRandomTools.nextFramePoint3D(random, sourceFrame);
         FramePoint3D actual = createEmptyFrameTuple(destinationFrame);

         actual.setMatchingFrame(source);

         FramePoint3D expected = new FramePoint3D(source);
         expected.changeFrame(destinationFrame);

         EuclidFrameTestTools.assertEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setMatchingFrame(FrameTuple2DReadOnly other, double z)
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);

         FrameTuple2DReadOnly source = EuclidFrameRandomTools.nextFramePoint2D(random, sourceFrame);
         double z = EuclidCoreRandomTools.nextDouble(random);
         FramePoint3D actual = createEmptyFrameTuple(destinationFrame);

         actual.setMatchingFrame(source, z);

         FramePoint3D expected = new FramePoint3D();
         expected.setIncludingFrame(source, z);
         expected.changeFrame(destinationFrame);

         EuclidFrameTestTools.assertEquals(expected, actual, EPSILON);
      }
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

         Point3D expectedPoint = EuclidCoreRandomTools.nextPoint3D(random);
         FramePoint3D framePoint = new FramePoint3D(initialFrame, expectedPoint);

         RigidBodyTransform transform = initialFrame.getTransformToDesiredFrame(anotherFrame);
         expectedPoint.applyTransform(transform);

         framePoint.changeFrame(anotherFrame);
         assertTrue(anotherFrame == framePoint.getReferenceFrame());
         EuclidCoreTestTools.assertEquals(expectedPoint, framePoint, 10.0 * EPSILON);

         ReferenceFrame differentRootFrame = ReferenceFrameTools.constructARootFrame("anotherRootFrame");
         try
         {
            framePoint.changeFrame(differentRootFrame);
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            // good
         }
      }
   }

   @Test
   public void testSetFromReferenceFrame() throws Exception
   {
      Random random = new Random(6572);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         ReferenceFrame anotherFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         FramePoint3D expected = createEmptyFrameTuple(anotherFrame);
         expected.changeFrame(initialFrame);

         FramePoint3D actual = createRandomFrameTuple(random, initialFrame);
         actual.setFromReferenceFrame(anotherFrame);
         assertTrue(initialFrame == actual.getReferenceFrame());
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Random random = new Random(32120);

      for (int i = 0; i < ITERATIONS; i++)
      {
         FramePoint3D framePoint1 = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame);
         FramePoint3D framePoint2 = new FramePoint3D(worldFrame);
         double epsilon = random.nextDouble();
         Vector3D difference;

         difference = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.99 * epsilon);
         framePoint2.add(framePoint1, difference);
         assertTrue(framePoint1.geometricallyEquals(framePoint2, epsilon));

         difference = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.01 * epsilon);
         framePoint2.add(framePoint1, difference);
         assertFalse(framePoint1.geometricallyEquals(framePoint2, epsilon));
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random, 100);

      FramePoint3D tuple = new FramePoint3D();

      tuple.setX(random.nextDouble());
      tuple.setY(random.nextDouble());
      tuple.setZ(random.nextDouble());
      tuple.setReferenceFrame(frames[random.nextInt(frames.length)]);

      int newHashCode, previousHashCode;
      newHashCode = tuple.hashCode();
      assertEquals(newHashCode, tuple.hashCode());

      previousHashCode = tuple.hashCode();

      for (int i = 0; i < ITERATIONS; i++)
      {
         tuple.setElement(i % 3, random.nextDouble());
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

   @Test
   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("set", Point3D.class));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", Point3D.class, Double.TYPE));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", Point3D.class, Double.TYPE));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertOverloadingWithFrameObjects(FramePoint3D.class, Point3D.class, true, 1, methodFilter);
   }

   @Override
   @Test
   public void testSetIncludingFrame()
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertSetIncludingFramePreserveFunctionality(EuclidFrameRandomTools::nextFramePoint3D, EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }
}
