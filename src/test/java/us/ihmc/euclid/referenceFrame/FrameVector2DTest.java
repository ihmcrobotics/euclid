package us.ihmc.euclid.referenceFrame;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple3D.Vector3D;

public class FrameVector2DTest extends FrameTuple2DBasicsTest<FrameVector2D>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Override
   public Tuple2DBasics createRandomFramelessTuple(Random random)
   {
      return EuclidCoreRandomTools.nextVector2D(random);
   }

   @Override
   public FrameVector2D createFrameTuple(ReferenceFrame referenceFrame, double x, double y)
   {
      return new FrameVector2D(referenceFrame, x, y);
   }

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(435345);

      { // Test FrameVector2D()
         FrameVector2D frameVector2D = new FrameVector2D();
         assertTrue(frameVector2D.getReferenceFrame() == worldFrame);
         EuclidCoreTestTools.assertTuple2DIsSetToZero(frameVector2D);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameVector2D(ReferenceFrame referenceFrame)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector2D frameVector2D = new FrameVector2D(randomFrame);
         assertTrue(frameVector2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DIsSetToZero(frameVector2D);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameVector2D(ReferenceFrame referenceFrame, double x, double y)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector2D randomTuple = EuclidCoreRandomTools.nextVector2D(random);
         FrameVector2D frameVector2D = new FrameVector2D(randomFrame, randomTuple.getX(), randomTuple.getY());
         assertTrue(frameVector2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomTuple, frameVector2D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameVector2D(ReferenceFrame referenceFrame, double[] pointArray)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector2D randomTuple = EuclidCoreRandomTools.nextVector2D(random);
         double[] array = new double[3];
         randomTuple.get(array);
         FrameVector2D frameVector2D = new FrameVector2D(randomFrame, array);
         assertTrue(frameVector2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomTuple, frameVector2D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameVector2D(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D randomTuple = EuclidCoreRandomTools.nextVector3D(random);
         FrameVector2D frameVector2D = new FrameVector2D(randomFrame, randomTuple);
         assertTrue(frameVector2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(new Vector2D(randomTuple), frameVector2D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameVector2D(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector2D randomTuple2D = EuclidCoreRandomTools.nextVector2D(random);
         FrameVector2D frameVector2D = new FrameVector2D(randomFrame, randomTuple2D);
         assertTrue(frameVector2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomTuple2D, frameVector2D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameVector2D(FrameTuple2DReadOnly frameTuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector2D randomFrameTuple2D = EuclidFrameRandomTools.nextFrameVector2D(random, randomFrame);
         FrameVector2D frameVector2D = new FrameVector2D(randomFrameTuple2D);
         assertTrue(frameVector2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomFrameTuple2D, frameVector2D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameVector2D(FrameTuple3DReadOnly other)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector3D randomTuple = EuclidFrameRandomTools.nextFrameVector3D(random, randomFrame);
         FrameVector2D frameVector2D = new FrameVector2D(randomTuple);
         assertTrue(frameVector2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(new Vector2D(randomTuple), frameVector2D, EPSILON);
      }
   }

   @Test
   public void testSetMatchingFrame() throws Exception
   {
      EuclidFrameAPITester.assertSetMatchingFramePreserveFunctionality(EuclidFrameRandomTools::nextFrameVector2D,
                                                                       EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

      Random random = new Random(544354);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setMatchingFrame(FrameTuple2DReadOnly other)
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);

         FrameTuple2DReadOnly source = EuclidFrameRandomTools.nextFramePoint2D(random, sourceFrame);
         FrameVector2D actual = createEmptyFrameTuple(destinationFrame);

         actual.setMatchingFrame(source);

         FrameVector2D expected = new FrameVector2D(source);
         expected.changeFrame(destinationFrame);

         EuclidFrameTestTools.assertFrameTuple2DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testSetIncludingFrame()
   {
      EuclidFrameAPITester.assertSetIncludingFramePreserveFunctionality(EuclidFrameRandomTools::nextFrameVector2D,
                                                                        EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }

   @Test
   public void testChangeFrame() throws Exception
   {
      Random random = new Random(43563);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random, true);
         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         ReferenceFrame anotherFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         FrameVector2D actual = new FrameVector2D(initialFrame, expected);

         RigidBodyTransform transform = initialFrame.getTransformToDesiredFrame(anotherFrame);
         expected.applyTransform(transform);

         actual.changeFrame(anotherFrame);
         assertTrue(anotherFrame == actual.getReferenceFrame());
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);

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
   public void testChangeFrameAndProjectToXYPlane() throws Exception
   {
      Random random = new Random(345345);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame initialFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         FrameVector2D actual = new FrameVector2D(initialFrame, expected);

         expected.applyTransform(initialFrame.getTransformToDesiredFrame(newFrame), false);
         actual.changeFrameAndProjectToXYPlane(newFrame);

         assertTrue(newFrame == actual.getReferenceFrame());
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Random random = new Random(32120);

      for (int i = 0; i < ITERATIONS; i++)
      {
         FrameVector2D frameVector1 = EuclidFrameRandomTools.nextFrameVector2D(random, worldFrame);
         FrameVector2D frameVector2 = new FrameVector2D(worldFrame);
         double epsilon = random.nextDouble();
         Vector2D difference;

         difference = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 0.99 * epsilon);
         frameVector2.add(frameVector1, difference);
         assertTrue(frameVector1.geometricallyEquals(frameVector2, epsilon));

         difference = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.01 * epsilon);
         frameVector2.add(frameVector1, difference);
         assertFalse(frameVector1.geometricallyEquals(frameVector2, epsilon));
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random, 100);

      FrameVector2D tuple = new FrameVector2D();

      tuple.setX(random.nextDouble());
      tuple.setY(random.nextDouble());
      tuple.setReferenceFrame(frames[random.nextInt(frames.length)]);

      int newHashCode, previousHashCode;
      newHashCode = tuple.hashCode();
      assertEquals(newHashCode, tuple.hashCode());

      previousHashCode = tuple.hashCode();

      for (int i = 0; i < ITERATIONS; i++)
      {
         tuple.setElement(i % 2, random.nextDouble());
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
      signaturesToIgnore.add(new MethodSignature("set", Vector2D.class));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", Vector2D.class, Double.TYPE));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", Vector2D.class, Double.TYPE));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FrameVector2D.class, Vector2D.class, true, 1, methodFilter);
   }
}
