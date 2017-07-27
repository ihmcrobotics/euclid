package us.ihmc.euclid.referenceFrame;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class FramePoint3DTest extends FrameTuple3DTest<FramePoint3D, Point3D>
{
   public static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Override
   public FramePoint3D createTuple(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      return new FramePoint3D(referenceFrame, x, y, z);
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

      { // Test FramePoint3D()
         FramePoint3D framePoint3D = new FramePoint3D();
         assertTrue(framePoint3D.referenceFrame == worldFrame);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(framePoint3D);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FramePoint3D(ReferenceFrame referenceFrame)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         FramePoint3D framePoint3D = new FramePoint3D(randomFrame);
         assertTrue(framePoint3D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(framePoint3D);
      }
      
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FramePoint3D(ReferenceFrame referenceFrame, double x, double y, double z)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Point3D randomTuple = EuclidCoreRandomTools.generateRandomPoint3D(random);
         FramePoint3D framePoint3D = new FramePoint3D(randomFrame, randomTuple.getX(), randomTuple.getY(), randomTuple.getZ());
         assertTrue(framePoint3D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomTuple, framePoint3D, EPSILON);
      }
      
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FramePoint3D(ReferenceFrame referenceFrame, double[] pointArray)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Point3D randomTuple = EuclidCoreRandomTools.generateRandomPoint3D(random);
         double[] array = new double[3];
         randomTuple.get(array);
         FramePoint3D framePoint3D = new FramePoint3D(randomFrame, array);
         assertTrue(framePoint3D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomTuple, framePoint3D, EPSILON);
      }
      
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FramePoint3D(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Point3D randomTuple = EuclidCoreRandomTools.generateRandomPoint3D(random);
         FramePoint3D framePoint3D = new FramePoint3D(randomFrame, randomTuple);
         assertTrue(framePoint3D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomTuple, framePoint3D, EPSILON);
      }
      
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FramePoint3D(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Point2D randomTuple2D = EuclidCoreRandomTools.generateRandomPoint2D(random);
         FramePoint3D framePoint3D = new FramePoint3D(randomFrame, randomTuple2D);
         assertTrue(framePoint3D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomTuple2D, new Point2D(framePoint3D), EPSILON);
         assertTrue(framePoint3D.getZ() == 0.0);
      }
      
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FramePoint3D(FrameTuple2DReadOnly frameTuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         FramePoint2D randomFrameTuple2D = EuclidFrameRandomTools.generateRandomFramePoint2D(random, randomFrame);
         FramePoint3D framePoint3D = new FramePoint3D(randomFrameTuple2D);
         assertTrue(framePoint3D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomFrameTuple2D, new Point2D(framePoint3D), EPSILON);
         assertTrue(framePoint3D.getZ() == 0.0);
      }
      
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FramePoint3D(FrameTuple3DReadOnly other)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         FramePoint3D randomTuple = EuclidFrameRandomTools.generateRandomFramePoint3D(random, randomFrame);
         FramePoint3D framePoint3D = new FramePoint3D(randomTuple);
         assertTrue(framePoint3D.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomTuple, framePoint3D, EPSILON);
         EuclidFrameTestTools.assertFrameTuple3DEquals(randomTuple, framePoint3D, EPSILON);
      }
   }

   @Test
   public void testGetPoint()
   {
      Random random = new Random(43535);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Vector3D expected = EuclidCoreRandomTools.generateRandomVector3D(random);
         FramePoint3D frameVector = new FramePoint3D(worldFrame, expected);
         Point3D actual = frameVector.getPoint();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(frameVector, actual, EPSILON);
      }
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      assertSuperMethodsAreOverloaded(FrameTuple3DReadOnly.class, Tuple3DReadOnly.class, FramePoint3D.class, Point3DBasics.class);
   }
}
