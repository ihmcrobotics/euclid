package us.ihmc.euclid.shape.primitives;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.RampPolytope3DView;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;

public class RampPolytope3DTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testIntegrity()
   {
      Random random = new Random(897234);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);
         RampPolytope3DView rampPolytope = ramp3D.asConvexPolytope();
         EuclidShapeTestTools.assertConvexPolytope3DGeneralIntegrity("Iteration " + i, rampPolytope);
         assertEquals(5, rampPolytope.getNumberOfFaces());
         assertEquals(9, rampPolytope.getNumberOfEdges());
         assertEquals(6, rampPolytope.getNumberOfVertices());
      }
   }

   @Test
   public void testAgainstConvexPolytope()
   {
      Random random = new Random(987345);
      Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);
      RampPolytope3DView rampPolytope3D = ramp3D.asConvexPolytope();

      for (int i = 0; i < ITERATIONS; i++)
      {
         // By re-using the same ramp, we're ensuring that the polytope view is updating accordingly.
         switch (random.nextInt(3))
         {
            case 0:
               ramp3D.set(EuclidShapeRandomTools.nextRamp3D(random));
               break;
            case 1:
               ramp3D.getPose().set(EuclidCoreRandomTools.nextRigidBodyTransform(random));
               break;
            case 2:
               ramp3D.getSize().set(EuclidCoreRandomTools.nextVector3D(random, 0.0, 10.0));
               break;
         }

         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D();

         for (int vertexIndex = 0; vertexIndex < 6; vertexIndex++)
         {
            Point3D vertex = new Point3D();
            vertex.set((vertexIndex & 2) == 0 ? ramp3D.getSizeX() : 0.0,
                       (vertexIndex & 1) == 0 ? 0.5 * ramp3D.getSizeY() : -0.5 * ramp3D.getSizeY(),
                       (vertexIndex & 4) == 0 ? 0.0 : ramp3D.getSizeZ());
            ramp3D.transformToWorld(vertex);
            convexPolytope3D.addVertex(vertex);
         }

         assertEquals(convexPolytope3D.getVolume(), rampPolytope3D.getVolume(), EPSILON);
         EuclidCoreTestTools.assertEquals(rampPolytope3D.getCentroid(), convexPolytope3D.getCentroid(), EPSILON);

         for (Face3DReadOnly rampFace : rampPolytope3D.getFaces())
         {
            Face3DReadOnly polytopeFace = convexPolytope3D.getClosestFace(rampFace.getCentroid());

            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(rampFace.getCentroid(), rampFace.getCentroid(), EPSILON);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(rampFace.getNormal(), rampFace.getNormal(), EPSILON);

            for (Vertex3DReadOnly rampVertex : rampFace.getVertices())
            {
               assertEquals(1L, polytopeFace.getVertices().stream().filter(polytopeVertex -> polytopeVertex.epsilonEquals(rampVertex, EPSILON)).count());
            }
         }
      }
   }
}
