package us.ihmc.euclid.referenceFrame;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.HashMap;
import java.util.Map;
import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;

public class FramePoint2DTest extends FrameTuple2DTest<FramePoint2D, Point2D>
{
   public static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Override
   public FramePoint2D createTuple(ReferenceFrame referenceFrame, double x, double y)
   {
      return new FramePoint2D(referenceFrame, x, y);
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-15;
   }

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(435345);

      { // Test FramePoint2D()
         FramePoint2D framePoint2D = new FramePoint2D();
         assertTrue(framePoint2D.referenceFrame == worldFrame);
         EuclidCoreTestTools.assertTuple2DIsSetToZero(framePoint2D);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FramePoint2D(ReferenceFrame referenceFrame)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         FramePoint2D framePoint2D = new FramePoint2D(randomFrame);
         assertTrue(framePoint2D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple2DIsSetToZero(framePoint2D);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FramePoint2D(ReferenceFrame referenceFrame, double x, double y)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Point2D randomTuple = EuclidCoreRandomTools.generateRandomPoint2D(random);
         FramePoint2D framePoint2D = new FramePoint2D(randomFrame, randomTuple.getX(), randomTuple.getY());
         assertTrue(framePoint2D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomTuple, framePoint2D, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FramePoint2D(ReferenceFrame referenceFrame, double[] pointArray)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Point2D randomTuple = EuclidCoreRandomTools.generateRandomPoint2D(random);
         double[] array = new double[3];
         randomTuple.get(array);
         FramePoint2D framePoint2D = new FramePoint2D(randomFrame, array);
         assertTrue(framePoint2D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomTuple, framePoint2D, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FramePoint2D(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Point2D randomTuple = EuclidCoreRandomTools.generateRandomPoint2D(random);
         FramePoint2D framePoint2D = new FramePoint2D(randomFrame, randomTuple);
         assertTrue(framePoint2D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomTuple, framePoint2D, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FramePoint2D(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Point3D randomTuple3D = EuclidCoreRandomTools.generateRandomPoint3D(random);
         FramePoint2D framePoint2D = new FramePoint2D(randomFrame, randomTuple3D);
         assertTrue(framePoint2D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(new Point2D(randomTuple3D), framePoint2D, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FramePoint3D(FrameTuple3DReadOnly frameTuple3DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         FramePoint3D randomFrameTuple3D = EuclidFrameRandomTools.generateRandomFramePoint3D(random, randomFrame);
         FramePoint2D framePoint2D = new FramePoint2D(randomFrameTuple3D);
         assertTrue(framePoint2D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(new Point2D(randomFrameTuple3D), framePoint2D, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FramePoint2D(FrameTuple2DReadOnly other)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         FramePoint2D randomTuple = EuclidFrameRandomTools.generateRandomFramePoint2D(random, randomFrame);
         FramePoint2D framePoint2D = new FramePoint2D(randomTuple);
         assertTrue(framePoint2D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomTuple, framePoint2D, EPSILON);
         EuclidFrameTestTools.assertFrameTuple2DEquals(randomTuple, framePoint2D, EPSILON);
      }
   }

   @Test
   public void testGetPoint()
   {
      Random random = new Random(43535);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Vector2D expected = EuclidCoreRandomTools.generateRandomVector2D(random);
         FramePoint2D frameVector = new FramePoint2D(worldFrame, expected);
         Point2D actual = frameVector.getPoint();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);
         EuclidCoreTestTools.assertTuple2DEquals(frameVector, actual, EPSILON);
      }
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("set", new Class<?>[] {Point2D.class});
      framelessMethodsToIgnore.put("epsilonEquals", new Class<?>[] {Point2D.class, Double.TYPE});
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FramePoint2D.class, Point2D.class, true, 1, framelessMethodsToIgnore);
   }

   @Test
   public void testGeometricallyEquals()
   {
      Random random = new Random(58722L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double epsilon = random.nextDouble();

         ReferenceFrame referenceFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         FramePoint2D framePoint1 = EuclidFrameRandomTools.generateRandomFramePoint2D(random, referenceFrame);
         FramePoint2D framePoint2 = EuclidFrameRandomTools.generateRandomFramePoint2D(random, referenceFrame);

         boolean expectedAnswer = framePoint1.getPoint().geometricallyEquals(framePoint2, epsilon);
         boolean actualAnswer = framePoint1.geometricallyEquals(framePoint2, epsilon);
         assertEquals(expectedAnswer, actualAnswer);
      }
   }
}
