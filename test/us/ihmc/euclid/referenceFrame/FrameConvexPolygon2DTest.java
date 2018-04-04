package us.ihmc.euclid.referenceFrame;

import java.util.HashMap;
import java.util.Map;
import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;

public class FrameConvexPolygon2DTest extends FrameConvexPolyong2DBasicsTest<FrameConvexPolygon2D>
{
   @Override
   public FrameConvexPolygon2D createFrameConvexPolygon2D(ReferenceFrame referenceFrame, Vertex2DSupplier vertex2DSupplier)
   {
      return new FrameConvexPolygon2D(referenceFrame, vertex2DSupplier);
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("set", new Class<?>[] {ConvexPolygon2D.class});
      framelessMethodsToIgnore.put("epsilonEquals", new Class<?>[] {ConvexPolygon2D.class, Double.TYPE});
      framelessMethodsToIgnore.put("geometricallyEquals", new Class<?>[] {ConvexPolygon2D.class, Double.TYPE});
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameConvexPolygon2D.class, ConvexPolygon2D.class, true, 1, framelessMethodsToIgnore);
   }

   @Test
   public void testAddVertexMatchingFrame() throws Exception
   {
      Random random = new Random(23423);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
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

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
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

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
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
}
