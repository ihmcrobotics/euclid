package us.ihmc.euclid.referenceFrame;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.HashMap;
import java.util.Map;
import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
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

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FramePoint3D(ReferenceFrame referenceFrame)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FramePoint3D framePoint3D = new FramePoint3D(randomFrame);
         assertTrue(framePoint3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(framePoint3D);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FramePoint3D(ReferenceFrame referenceFrame, double x, double y, double z)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Point3D randomTuple = EuclidCoreRandomTools.nextPoint3D(random);
         FramePoint3D framePoint3D = new FramePoint3D(randomFrame, randomTuple.getX(), randomTuple.getY(), randomTuple.getZ());
         assertTrue(framePoint3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomTuple, framePoint3D, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FramePoint3D(ReferenceFrame referenceFrame, double[] pointArray)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Point3D randomTuple = EuclidCoreRandomTools.nextPoint3D(random);
         double[] array = new double[3];
         randomTuple.get(array);
         FramePoint3D framePoint3D = new FramePoint3D(randomFrame, array);
         assertTrue(framePoint3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomTuple, framePoint3D, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FramePoint3D(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Point3D randomTuple = EuclidCoreRandomTools.nextPoint3D(random);
         FramePoint3D framePoint3D = new FramePoint3D(randomFrame, randomTuple);
         assertTrue(framePoint3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomTuple, framePoint3D, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FramePoint3D(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Point2D randomTuple2D = EuclidCoreRandomTools.nextPoint2D(random);
         FramePoint3D framePoint3D = new FramePoint3D(randomFrame, randomTuple2D);
         assertTrue(framePoint3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomTuple2D, new Point2D(framePoint3D), EPSILON);
         assertTrue(framePoint3D.getZ() == 0.0);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FramePoint3D(FrameTuple2DReadOnly frameTuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FramePoint2D randomFrameTuple2D = EuclidFrameRandomTools.nextFramePoint2D(random, randomFrame);
         FramePoint3D framePoint3D = new FramePoint3D(randomFrameTuple2D);
         assertTrue(framePoint3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomFrameTuple2D, new Point2D(framePoint3D), EPSILON);
         assertTrue(framePoint3D.getZ() == 0.0);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FramePoint3D(FrameTuple3DReadOnly other)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FramePoint3D randomTuple = EuclidFrameRandomTools.nextFramePoint3D(random, randomFrame);
         FramePoint3D framePoint3D = new FramePoint3D(randomTuple);
         assertTrue(framePoint3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomTuple, framePoint3D, EPSILON);
         EuclidFrameTestTools.assertFrameTuple3DEquals(randomTuple, framePoint3D, EPSILON);
      }
   }

   @Test
   public void testSetMatchingFrame() throws Exception
   {
      Random random = new Random(544354);
      
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test setMatchingFrame(FrameTuple3DReadOnly other)
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);

         FrameTuple3DReadOnly source = EuclidFrameRandomTools.nextFramePoint3D(random, sourceFrame);
         FramePoint3D actual = createEmptyFrameTuple(destinationFrame);
         
         actual.setMatchingFrame(source);
         
         FramePoint3D expected = new FramePoint3D(source);
         expected.changeFrame(destinationFrame);
         
         EuclidFrameTestTools.assertFrameTuple3DEquals(expected, actual, EPSILON);
      }
      
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
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
         
         EuclidFrameTestTools.assertFrameTuple3DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testChangeFrame() throws Exception
   {
      Random random = new Random(43563);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         ReferenceFrameTools.clearWorldFrameTree();
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         ReferenceFrame anotherFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         Point3D expectedPoint = EuclidCoreRandomTools.nextPoint3D(random);
         FramePoint3D framePoint = new FramePoint3D(initialFrame, expectedPoint);

         RigidBodyTransform transform = initialFrame.getTransformToDesiredFrame(anotherFrame);
         expectedPoint.applyTransform(transform);

         framePoint.changeFrame(anotherFrame);
         assertTrue(anotherFrame == framePoint.getReferenceFrame());
         EuclidCoreTestTools.assertTuple3DEquals(expectedPoint, framePoint, EPSILON);

         ReferenceFrame differentRootFrame = ReferenceFrame.constructARootFrame("anotherRootFrame");
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

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         ReferenceFrameTools.clearWorldFrameTree();
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         ReferenceFrame anotherFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         FramePoint3D expected = createEmptyFrameTuple(anotherFrame);
         expected.changeFrame(initialFrame);

         FramePoint3D actual = createRandomFrameTuple(random, initialFrame);
         actual.setFromReferenceFrame(anotherFrame);
         assertTrue(initialFrame == actual.getReferenceFrame());
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Random random = new Random(32120);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
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
      Random random = new Random(763);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Point3D expected = EuclidCoreRandomTools.nextPoint3D(random, -1.0e15, 1.0e15);
         FramePoint3D actual = new FramePoint3D(worldFrame, expected);

         assertEquals(expected.hashCode(), actual.hashCode());
      }
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("set", new Class<?>[] {Point3D.class});
      framelessMethodsToIgnore.put("epsilonEquals", new Class<?>[] {Point3D.class, Double.TYPE});
      framelessMethodsToIgnore.put("geometricallyEquals", new Class<?>[] {Point3D.class, Double.TYPE});
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FramePoint3D.class, Point3D.class, true, 1, framelessMethodsToIgnore);
   }
}
