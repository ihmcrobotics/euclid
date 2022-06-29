package us.ihmc.euclid.shape.primitives;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.BoxPolytope3DView;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;

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
      Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
      BoxPolytope3DView boxPolytope3D = box3D.asConvexPolytope();

      for (int i = 0; i < ITERATIONS; i++)
      {
         // By re-using the same box, we're ensuring that the polytope view is updating accordingly.
         switch (random.nextInt(3))
         {
            case 0:
               box3D.set(EuclidShapeRandomTools.nextBox3D(random));
               break;
            case 1:
               box3D.getPose().set(EuclidCoreRandomTools.nextRigidBodyTransform(random));
               break;
            case 2:
               box3D.getSize().set(EuclidCoreRandomTools.nextVector3D(random, 0.0, 10.0));
               break;
         }

         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D();

         for (int vertexIndex = 0; vertexIndex < 8; vertexIndex++)
         {
            Point3D vertex = new Point3D();
            vertex.set((vertexIndex & 1) == 0 ? box3D.getSizeX() : -box3D.getSizeX(),
                       (vertexIndex & 2) == 0 ? box3D.getSizeY() : -box3D.getSizeY(),
                       (vertexIndex & 4) == 0 ? box3D.getSizeZ() : -box3D.getSizeZ());
            vertex.scale(0.5);
            box3D.transformToWorld(vertex);
            convexPolytope3D.addVertex(vertex);
         }

         assertEquals(boxPolytope3D.getVolume(), convexPolytope3D.getVolume(), EPSILON);
         EuclidCoreTestTools.assertEquals(convexPolytope3D.getCentroid(), boxPolytope3D.getCentroid(), EPSILON);

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
