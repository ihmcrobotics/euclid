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
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPIDefaultConfiguration;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;

public class FrameConvexPolygon2DTest extends FrameConvexPolyong2DBasicsTest<FrameConvexPolygon2D>
{
   @Override
   public FrameConvexPolygon2D createFrameConvexPolygon2D(ReferenceFrame referenceFrame, Vertex2DSupplier vertex2DSupplier)
   {
      return new FrameConvexPolygon2D(referenceFrame, vertex2DSupplier);
   }

   /**
    * {@link FrameConvexPolygon2D#setIncludingFrame(us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier)}
    * given an empty polygon does actually not set the frame.
    */
   @Test
   public void testIssue16()
   {
      Random random = new Random(342);

      {
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame(random);

         FrameConvexPolygon2D firstPolygon = EuclidFrameRandomTools.nextFrameConvexPolygon2D(random, frameA, 1.0, 10);
         FrameConvexPolygon2D secondPolygon = new FrameConvexPolygon2D(frameB);
         firstPolygon.setIncludingFrame(secondPolygon);

         EuclidFrameTestTools.assertFrameConvexPolygon2DEquals(firstPolygon, secondPolygon, EPSILON);
      }

      {
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame(random);

         FrameConvexPolygon2D firstPolygon = EuclidFrameRandomTools.nextFrameConvexPolygon2D(random, frameA, 1.0, 10);
         FrameConvexPolygon2D secondPolygon = new FrameConvexPolygon2D(frameB);
         FrameConvexPolygon2D thirdPolygon = new FrameConvexPolygon2D(frameB);
         firstPolygon.setIncludingFrame(secondPolygon, thirdPolygon);
         EuclidFrameTestTools.assertFrameConvexPolygon2DEquals(firstPolygon, secondPolygon, EPSILON);
      }
   }

   @Test
   public void testIssue17() throws Exception
   {
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame frame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FramePoint2D pointA = EuclidFrameRandomTools.nextFramePoint2D(random, frame, 10.0);
         FramePoint2D pointB = EuclidFrameRandomTools.nextFramePoint2D(random, frame, 10.0);
         FrameConvexPolygon2D polygon = new FrameConvexPolygon2D(frame);
         polygon.addVertex(pointA);
         polygon.addVertex(pointA);
         polygon.addVertex(pointB);
         polygon.addVertex(pointB);

         assertFalse(pointA.epsilonEquals(pointB, 1.0e-7));

         polygon.update();

         assertEquals(2, polygon.getNumberOfVertices());
         if (polygon.getVertex(0).equals(pointA))
         {
            assertEquals(polygon.getVertex(1), pointB);
         }
         else
         {
            assertEquals(polygon.getVertex(0), pointB);
            assertEquals(polygon.getVertex(1), pointA);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame frame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FramePoint2D pointA = EuclidFrameRandomTools.nextFramePoint2D(random, frame, 10.0);
         FramePoint2D pointB = EuclidFrameRandomTools.nextFramePoint2D(random, frame, 10.0);
         FrameConvexPolygon2D polygon = new FrameConvexPolygon2D(frame);
         polygon.addVertex(pointA);
         polygon.addVertex(pointB);
         polygon.addVertex(pointA);
         polygon.addVertex(pointB);

         assertFalse(pointA.epsilonEquals(pointB, 1.0e-7));

         polygon.update();

         assertEquals(2, polygon.getNumberOfVertices());
         if (polygon.getVertex(0).equals(pointA))
         {
            assertEquals(polygon.getVertex(1), pointB);
         }
         else
         {
            assertEquals(polygon.getVertex(0), pointB);
            assertEquals(polygon.getVertex(1), pointA);
         }
      }
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("set", ConvexPolygon2D.class));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", ConvexPolygon2D.class, Double.TYPE));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", ConvexPolygon2D.class, Double.TYPE));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertOverloadingWithFrameObjects(FrameConvexPolygon2D.class, ConvexPolygon2D.class, true, 1, methodFilter);
   }

   @Test
   public void testAddVertexMatchingFrame() throws Exception
   {
      Random random = new Random(23423);

      for (int i = 0; i < ITERATIONS; i++)
      { // addVertexMatchingFrame(ReferenceFrame referenceFrame, Point2DReadOnly vertex)
         boolean useTransform2D = random.nextBoolean();
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame(random, useTransform2D);
         ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame(random, useTransform2D);

         FramePoint2D newVertex = EuclidFrameRandomTools.nextFramePoint2D(random, frameA);
         FrameConvexPolygon2D polygon = createEmptyFrameConvexPolygon2D(frameB);
         polygon.addVertexMatchingFrame(newVertex.getReferenceFrame(), newVertex, useTransform2D);

         FramePoint2D expected = new FramePoint2D(frameB);
         newVertex.changeFrameAndProjectToXYPlane(frameB);
         expected.set(newVertex);

         EuclidFrameTestTools.assertFramePoint2DGeometricallyEquals(expected, polygon.getVertexUnsafe(0), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // addVertexMatchingFrame(ReferenceFrame referenceFrame, Point3DReadOnly vertex)
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame(random);

         FramePoint3D newVertex = EuclidFrameRandomTools.nextFramePoint3D(random, frameA);
         FrameConvexPolygon2D polygon = createEmptyFrameConvexPolygon2D(frameB);
         polygon.addVertexMatchingFrame(newVertex.getReferenceFrame(), newVertex);

         FramePoint2D expected = new FramePoint2D(frameB);
         newVertex.changeFrame(frameB);
         expected.set(newVertex);

         EuclidFrameTestTools.assertFramePoint2DGeometricallyEquals(expected, polygon.getVertexUnsafe(0), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // addVertexMatchingFrame(FramePoint3DReadOnly vertex)
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame(random);

         FramePoint3D newVertex = EuclidFrameRandomTools.nextFramePoint3D(random, frameA);
         FrameConvexPolygon2D polygon = createEmptyFrameConvexPolygon2D(frameB);
         polygon.addVertexMatchingFrame(newVertex);

         FramePoint2D expected = new FramePoint2D(frameB);
         newVertex.changeFrame(frameB);
         expected.set(newVertex);

         EuclidFrameTestTools.assertFramePoint2DGeometricallyEquals(expected, polygon.getVertexUnsafe(0), EPSILON);
      }
   }

   @Test
   public void testSetMatchingFrame() throws Exception
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertSetMatchingFramePreserveFunctionality(EuclidFrameRandomTools::nextFrameConvexPolygon2D,
                                                         m -> true,
                                                         EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

      Random random = new Random(544354);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);

         FrameConvexPolygon2DReadOnly source = EuclidFrameRandomTools.nextFrameConvexPolygon2D(random, sourceFrame, 1.0, 10);
         FixedFrameConvexPolygon2DBasics actual = EuclidFrameRandomTools.nextFrameConvexPolygon2D(random, destinationFrame, 1.0, 10);

         actual.setMatchingFrame(source, true);

         FrameConvexPolygon2D expected = new FrameConvexPolygon2D(source);
         expected.changeFrame(destinationFrame);

         assertTrue(expected.epsilonEquals((FrameConvexPolygon2D) actual, EPSILON));
      }
   }

   @Test
   public void testSetIncludingFrame()
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertSetIncludingFramePreserveFunctionality(EuclidFrameRandomTools::nextFrameConvexPolygon2D,
                                                          EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }
}
