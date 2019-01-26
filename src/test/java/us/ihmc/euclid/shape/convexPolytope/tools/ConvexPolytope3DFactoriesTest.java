package us.ihmc.euclid.shape.convexPolytope.tools;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.shape.convexPolytope.Vertex3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;

class ConvexPolytope3DFactoriesTest
{
   private static final int ITERATIONS = 1000;

   @Test
   void testNewFace3DFromVertexAndTwinEdge()
   {
      Random random = new Random(54356436);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vertex3D vertex = new Vertex3D(EuclidCoreRandomTools.nextPoint3D(random));

      }
   }

}
