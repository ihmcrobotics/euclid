package us.ihmc.euclid.referenceFrame;

import static org.junit.Assert.*;

import java.util.HashMap;
import java.util.Map;
import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class FrameVector2DTest extends FrameTuple2DTest<FrameVector2D, Vector2D>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Override
   public FrameVector2D createTuple(ReferenceFrame referenceFrame, double x, double y)
   {
      return new FrameVector2D(referenceFrame, x, y);
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

      { // Test FrameVector2D()
         FrameVector2D frameVector2D = new FrameVector2D();
         assertTrue(frameVector2D.referenceFrame == worldFrame);
         EuclidCoreTestTools.assertTuple2DIsSetToZero(frameVector2D);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameVector2D(ReferenceFrame referenceFrame)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         FrameVector2D frameVector2D = new FrameVector2D(randomFrame);
         assertTrue(frameVector2D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple2DIsSetToZero(frameVector2D);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameVector2D(ReferenceFrame referenceFrame, double x, double y)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Vector2D randomTuple = EuclidCoreRandomTools.generateRandomVector2D(random);
         FrameVector2D frameVector2D = new FrameVector2D(randomFrame, randomTuple.getX(), randomTuple.getY());
         assertTrue(frameVector2D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomTuple, frameVector2D, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameVector2D(ReferenceFrame referenceFrame, double[] pointArray)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Vector2D randomTuple = EuclidCoreRandomTools.generateRandomVector2D(random);
         double[] array = new double[3];
         randomTuple.get(array);
         FrameVector2D frameVector2D = new FrameVector2D(randomFrame, array);
         assertTrue(frameVector2D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomTuple, frameVector2D, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameVector2D(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Vector3D randomTuple = EuclidCoreRandomTools.generateRandomVector3D(random);
         FrameVector2D frameVector2D = new FrameVector2D(randomFrame, randomTuple);
         assertTrue(frameVector2D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(new Vector2D(randomTuple), frameVector2D, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameVector2D(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Vector2D randomTuple2D = EuclidCoreRandomTools.generateRandomVector2D(random);
         FrameVector2D frameVector2D = new FrameVector2D(randomFrame, randomTuple2D);
         assertTrue(frameVector2D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomTuple2D, frameVector2D, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameVector2D(FrameTuple2DReadOnly frameTuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         FrameVector2D randomFrameTuple2D = EuclidFrameRandomTools.generateRandomFrameVector2D(random, randomFrame);
         FrameVector2D frameVector2D = new FrameVector2D(randomFrameTuple2D);
         assertTrue(frameVector2D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomFrameTuple2D, frameVector2D, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameVector2D(FrameTuple3DReadOnly other)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         FrameVector3D randomTuple = EuclidFrameRandomTools.generateRandomFrameVector3D(random, randomFrame);
         FrameVector2D frameVector2D = new FrameVector2D(randomTuple);
         assertTrue(frameVector2D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(new Vector2D(randomTuple), frameVector2D, EPSILON);
      }
   }

   @Test
   public void testGetVector()
   {
      Random random = new Random(43535);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Vector2D expected = EuclidCoreRandomTools.generateRandomVector2D(random);
         FrameVector2D frameVector = new FrameVector2D(worldFrame, expected);
         Vector2D actual = frameVector.getVector();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);
         EuclidCoreTestTools.assertTuple2DEquals(frameVector, actual, EPSILON);
      }
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("set", new Class<?>[]{Vector2D.class});
      framelessMethodsToIgnore.put("epsilonEquals", new Class<?>[]{Vector2D.class, Double.TYPE});
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameVector2D.class, Vector2D.class, true, 1, framelessMethodsToIgnore);
   }
}
