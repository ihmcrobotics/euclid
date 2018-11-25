package us.ihmc.euclid.shape.convexPolytope;

import static org.junit.Assert.*;

import java.util.List;
import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeRandomTools;
import us.ihmc.euclid.testSuite.EuclidTestSuite;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class Face3DTest
{
   private static final int ITERATIONS = EuclidTestSuite.ITERATIONS;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void test()
   {
      Random random = new Random(3663);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D faceNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         List<Point3D> points = EuclidPolytopeRandomTools.nextCircleBasedConvexPolygon3D(random, 5.0, 1.0, 6, faceNormal);

         Face3D face3D = new Face3D();
         for (Point3D point : points)
            face3D.addVertex(new Vertex3D(point), 0.0);

         assertEquals(points.size(), face3D.getNumberOfEdges());
      }
   }

}
