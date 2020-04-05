package us.ihmc.euclid.referenceFrame;

import java.util.Collections;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPIDefaultConfiguration;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public abstract class FrameConvexPolygon2DReadOnlyTest<F extends FrameConvexPolygon2DReadOnly>
{
   public static final double EPSILON = 1.0e-15;

   public F createEmptyFrameConvexPolygon2D()
   {
      return createFrameConvexPolygon2D(ReferenceFrame.getWorldFrame(), Collections.emptyList());
   }

   public final F createEmptyFrameConvexPolygon2D(ReferenceFrame referenceFrame)
   {
      return createFrameConvexPolygon2D(referenceFrame, Collections.emptyList());
   }

   public F createRandomFrameConvexPolygon2D(Random random)
   {
      return createFrameConvexPolygon2D(ReferenceFrame.getWorldFrame(), EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, 1.0, 0.5, 10));
   }

   public final F createRandomFrameConvexPolygon2D(Random random, ReferenceFrame referenceFrame)
   {
      return createFrameConvexPolygon2D(referenceFrame, EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, 1.0, 0.5, 10));
   }

   public final F createFrameConvexPolygon2D(ReferenceFrame referenceFrame, ConvexPolygon2DReadOnly polygon)
   {
      return createFrameConvexPolygon2D(referenceFrame, polygon.getVertexBufferView());
   }

   public final F createFrameConvexPolygon2D(F frameConvexPolygon2D)
   {
      return createFrameConvexPolygon2D(frameConvexPolygon2D.getReferenceFrame(),
                                        frameConvexPolygon2D.getVertexBufferView().subList(0, frameConvexPolygon2D.getNumberOfVertices()));
   }

   public F createFrameConvexPolygon2D(List<? extends Point2DReadOnly> vertices)
   {
      return createFrameConvexPolygon2D(ReferenceFrame.getWorldFrame(), vertices);
   }

   public F createFrameConvexPolygon2D(ReferenceFrame referenceFrame, List<? extends Point2DReadOnly> vertices)
   {
      return createFrameConvexPolygon2D(referenceFrame, Vertex2DSupplier.asVertex2DSupplier(vertices));
   }

   public abstract F createFrameConvexPolygon2D(ReferenceFrame referenceFrame, Vertex2DSupplier vertex2DSupplier);

   @Test
   public void testOverloading() throws Exception
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertOverloadingWithFrameObjects(FrameConvexPolygon2DReadOnly.class, ConvexPolygon2DReadOnly.class, true);
   }
}
