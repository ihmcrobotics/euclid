package us.ihmc.euclid.referenceFrame.polytope;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FrameShapeSetupTest;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameConvexPolytope3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameFace3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameHalfEdge3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameVertex3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;

public class FrameConvexPolytope3DTest extends FrameShapeSetupTest
{
   @Test
   public void testAPIOverloading()
   {
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FrameConvexPolytope3DReadOnly.class, ConvexPolytope3DReadOnly.class, false, 1);
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FrameFace3DReadOnly.class, Face3DReadOnly.class, false, 1);
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FrameHalfEdge3DReadOnly.class, HalfEdge3DReadOnly.class, false, 1);
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FrameVertex3DReadOnly.class, Vertex3DReadOnly.class, false, 1);
   }

}
