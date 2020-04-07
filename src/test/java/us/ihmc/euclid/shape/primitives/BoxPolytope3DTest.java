package us.ihmc.euclid.shape.primitives;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.BoxPolytope3DView;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTestTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;

public class BoxPolytope3DTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testIntegrity()
   {
      Random random = new Random(897234);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         BoxPolytope3DView boxPolytope = box3D.asConvexPolytope();
         EuclidShapeTestTools.assertConvexPolytope3DGeneralIntegrity("Iteration " + i, boxPolytope);

         assertEquals(6, boxPolytope.getNumberOfFaces());
         assertEquals(12, boxPolytope.getNumberOfEdges());
         assertEquals(8, boxPolytope.getNumberOfVertices());

         for (Face3DReadOnly face : boxPolytope.getFaces())
            assertEquals(4, face.getNumberOfEdges());
         for (Vertex3DReadOnly vertex : boxPolytope.getVertices())
            assertEquals(3, vertex.getNumberOfAssociatedEdges());
      }
   }

   @Test
   public void testAgainstConvexPolytope()
   {
      Random random = new Random(987345);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         BoxPolytope3DView boxPolytope3D = box3D.asConvexPolytope();
         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(boxPolytope3D.getVertices()));

         assertEquals(boxPolytope3D.getVolume(), convexPolytope3D.getVolume(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(boxPolytope3D.getCentroid(), convexPolytope3D.getCentroid(), EPSILON);

         for (Face3DReadOnly boxFace : boxPolytope3D.getFaces())
         {
            Face3DReadOnly polytopeFace = convexPolytope3D.getClosestFace(boxFace.getCentroid());

            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(boxFace.getCentroid(), boxFace.getCentroid(), EPSILON);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(boxFace.getNormal(), boxFace.getNormal(), EPSILON);

            for (Vertex3DReadOnly boxVertex : boxFace.getVertices())
            {
               assertEquals(1L, polytopeFace.getVertices().stream().filter(polytopeVertex -> polytopeVertex.epsilonEquals(boxVertex, EPSILON)).count());
            }

         }
      }
   }
}
