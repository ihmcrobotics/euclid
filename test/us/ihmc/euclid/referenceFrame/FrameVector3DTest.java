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
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class FrameVector3DTest extends FrameTuple3DTest<FrameVector3D, Vector3D>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Override
   public FrameVector3D createTuple(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      return new FrameVector3D(referenceFrame, x, y, z);
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

      { // Test FrameVector3D()
         FrameVector3D frameVector3D = new FrameVector3D();
         assertTrue(frameVector3D.referenceFrame == worldFrame);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(frameVector3D);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameVector3D(ReferenceFrame referenceFrame)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         FrameVector3D frameVector3D = new FrameVector3D(randomFrame);
         assertTrue(frameVector3D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(frameVector3D);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameVector3D(ReferenceFrame referenceFrame, double x, double y, double z)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Vector3D randomTuple = EuclidCoreRandomTools.generateRandomVector3D(random);
         FrameVector3D frameVector3D = new FrameVector3D(randomFrame, randomTuple.getX(), randomTuple.getY(), randomTuple.getZ());
         assertTrue(frameVector3D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomTuple, frameVector3D, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameVector3D(ReferenceFrame referenceFrame, double[] pointArray)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Vector3D randomTuple = EuclidCoreRandomTools.generateRandomVector3D(random);
         double[] array = new double[3];
         randomTuple.get(array);
         FrameVector3D frameVector3D = new FrameVector3D(randomFrame, array);
         assertTrue(frameVector3D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomTuple, frameVector3D, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameVector3D(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Vector3D randomTuple = EuclidCoreRandomTools.generateRandomVector3D(random);
         FrameVector3D frameVector3D = new FrameVector3D(randomFrame, randomTuple);
         assertTrue(frameVector3D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomTuple, frameVector3D, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameVector3D(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Vector2D randomTuple2D = EuclidCoreRandomTools.generateRandomVector2D(random);
         FrameVector3D frameVector3D = new FrameVector3D(randomFrame, randomTuple2D);
         assertTrue(frameVector3D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomTuple2D, new Vector2D(frameVector3D), EPSILON);
         assertTrue(frameVector3D.getZ() == 0.0);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameVector3D(FrameTuple2DReadOnly frameTuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         FrameVector2D randomFrameTuple2D = EuclidFrameRandomTools.generateRandomFrameVector2D(random, randomFrame);
         FrameVector3D frameVector3D = new FrameVector3D(randomFrameTuple2D);
         assertTrue(frameVector3D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomFrameTuple2D, new Vector2D(frameVector3D), EPSILON);
         assertTrue(frameVector3D.getZ() == 0.0);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameVector3D(FrameTuple3DReadOnly other)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         FrameVector3D randomTuple = EuclidFrameRandomTools.generateRandomFrameVector3D(random, randomFrame);
         FrameVector3D frameVector3D = new FrameVector3D(randomTuple);
         assertTrue(frameVector3D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomTuple, frameVector3D, EPSILON);
         EuclidFrameTestTools.assertFrameTuple3DEquals(randomTuple, frameVector3D, EPSILON);
      }
   }

   @Test
   public void testGetVector()
   {
      Random random = new Random(43535);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Vector3D expected = EuclidCoreRandomTools.generateRandomVector3D(random);
         FrameVector3D frameVector = new FrameVector3D(worldFrame, expected);
         Vector3D actual = frameVector.getVector();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(frameVector, actual, EPSILON);
      }
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("set", new Class<?>[] {Vector3D.class});
      framelessMethodsToIgnore.put("epsilonEquals", new Class<?>[] {Vector3D.class, Double.TYPE});
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameVector3D.class, Vector3D.class, true, 1, framelessMethodsToIgnore);
   }

   @Test
   public void testGeometricallyEquals()
   {
      Random random = new Random(58722L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double epsilon = random.nextDouble();

         ReferenceFrame referenceFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         FrameVector3D frameVector1 = EuclidFrameRandomTools.generateRandomFrameVector3D(random, referenceFrame);
         FrameVector3D frameVector2 = EuclidFrameRandomTools.generateRandomFrameVector3D(random, referenceFrame);

         boolean expectedAnswer = frameVector1.getVector().geometricallyEquals(frameVector2, epsilon);
         boolean actualAnswer = frameVector1.geometricallyEquals(frameVector2, epsilon);
         assertEquals(expectedAnswer, actualAnswer);
      }
   }
}
