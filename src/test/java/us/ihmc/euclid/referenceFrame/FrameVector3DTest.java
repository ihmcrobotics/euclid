package us.ihmc.euclid.referenceFrame;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.HashMap;
import java.util.Map;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;

public class FrameVector3DTest extends FrameTuple3DBasicsTest<FrameVector3D>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Override
   public Tuple3DBasics createRandomFramelessTuple(Random random)
   {
      return EuclidCoreRandomTools.nextVector3D(random);
   }

   @Override
   public FrameVector3D createFrameTuple(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      return new FrameVector3D(referenceFrame, x, y, z);
   }

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(435345);

      { // Test FrameVector3D()
         FrameVector3D frameVector3D = new FrameVector3D();
         assertTrue(frameVector3D.getReferenceFrame() == worldFrame);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(frameVector3D);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameVector3D(ReferenceFrame referenceFrame)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector3D frameVector3D = new FrameVector3D(randomFrame);
         assertTrue(frameVector3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(frameVector3D);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameVector3D(ReferenceFrame referenceFrame, double x, double y, double z)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D randomTuple = EuclidCoreRandomTools.nextVector3D(random);
         FrameVector3D frameVector3D = new FrameVector3D(randomFrame, randomTuple.getX(), randomTuple.getY(), randomTuple.getZ());
         assertTrue(frameVector3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomTuple, frameVector3D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameVector3D(ReferenceFrame referenceFrame, double[] pointArray)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D randomTuple = EuclidCoreRandomTools.nextVector3D(random);
         double[] array = new double[3];
         randomTuple.get(array);
         FrameVector3D frameVector3D = new FrameVector3D(randomFrame, array);
         assertTrue(frameVector3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomTuple, frameVector3D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameVector3D(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D randomTuple = EuclidCoreRandomTools.nextVector3D(random);
         FrameVector3D frameVector3D = new FrameVector3D(randomFrame, randomTuple);
         assertTrue(frameVector3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomTuple, frameVector3D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameVector3D(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector2D randomTuple2D = EuclidCoreRandomTools.nextVector2D(random);
         FrameVector3D frameVector3D = new FrameVector3D(randomFrame, randomTuple2D);
         assertTrue(frameVector3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomTuple2D, new Vector2D(frameVector3D), EPSILON);
         assertTrue(frameVector3D.getZ() == 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameVector3D(FrameTuple2DReadOnly frameTuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector2D randomFrameTuple2D = EuclidFrameRandomTools.nextFrameVector2D(random, randomFrame);
         FrameVector3D frameVector3D = new FrameVector3D(randomFrameTuple2D);
         assertTrue(frameVector3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomFrameTuple2D, new Vector2D(frameVector3D), EPSILON);
         assertTrue(frameVector3D.getZ() == 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameVector3D(FrameTuple3DReadOnly other)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector3D randomTuple = EuclidFrameRandomTools.nextFrameVector3D(random, randomFrame);
         FrameVector3D frameVector3D = new FrameVector3D(randomTuple);
         assertTrue(frameVector3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomTuple, frameVector3D, EPSILON);
         EuclidFrameTestTools.assertFrameTuple3DEquals(randomTuple, frameVector3D, EPSILON);
      }
   }

   @Test
   public void testSetMatchingFrame() throws Exception
   {
      Random random = new Random(544354);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setMatchingFrame(FrameTuple3DReadOnly other)
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);

         FrameTuple3DReadOnly source = EuclidFrameRandomTools.nextFrameVector3D(random, sourceFrame);
         FrameVector3D actual = createEmptyFrameTuple(destinationFrame);

         actual.setMatchingFrame(source);

         FrameVector3D expected = new FrameVector3D(source);
         expected.changeFrame(destinationFrame);

         EuclidFrameTestTools.assertFrameTuple3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setMatchingFrame(FrameTuple2DReadOnly other, double z)
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);

         FrameTuple2DReadOnly source = EuclidFrameRandomTools.nextFrameVector2D(random, sourceFrame);
         double z = EuclidCoreRandomTools.nextDouble(random);
         FrameVector3D actual = createEmptyFrameTuple(destinationFrame);

         actual.setMatchingFrame(source, z);

         FrameVector3D expected = new FrameVector3D();
         expected.setIncludingFrame(source, z);
         expected.changeFrame(destinationFrame);

         EuclidFrameTestTools.assertFrameTuple3DEquals(expected, actual, EPSILON);
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

         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         FrameVector3D actual = new FrameVector3D(initialFrame, expected);

         RigidBodyTransform transform = initialFrame.getTransformToDesiredFrame(anotherFrame);
         expected.applyTransform(transform);

         actual.changeFrame(anotherFrame);
         assertTrue(anotherFrame == actual.getReferenceFrame());
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);

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
         FrameVector3D frameVector1 = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);
         FrameVector3D frameVector2 = new FrameVector3D(worldFrame);
         double epsilon = random.nextDouble();
         Vector3D difference;

         difference = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.99 * epsilon);
         frameVector2.add(frameVector1, difference);
         assertTrue(frameVector1.geometricallyEquals(frameVector2, epsilon));

         difference = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.01 * epsilon);
         frameVector2.add(frameVector1, difference);
         assertFalse(frameVector1.geometricallyEquals(frameVector2, epsilon));
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random, 100);

      FrameVector3D tuple = new FrameVector3D();

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

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("set", new Class<?>[] {Vector3D.class});
      framelessMethodsToIgnore.put("epsilonEquals", new Class<?>[] {Vector3D.class, Double.TYPE});
      framelessMethodsToIgnore.put("geometricallyEquals", new Class<?>[] {Vector3D.class, Double.TYPE});
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FrameVector3D.class, Vector3D.class, true, 1, framelessMethodsToIgnore);
   }
}
