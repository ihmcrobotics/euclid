package us.ihmc.euclid.shape.convexPolytope.tools;

import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.Face3D;
import us.ihmc.euclid.shape.convexPolytope.Vertex3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class EuclidPolytopeRandomTools
{
   public static Face3D nextCircleBasedFace3D(Random random, double centerMinMax, double maxEdgeLength, int numberOfVertices)
   {
      return nextCircleBasedFace3D(random, centerMinMax, maxEdgeLength, numberOfVertices, EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0));
   }

   public static Face3D nextCircleBasedFace3D(Random random, double centerMinMax, double maxEdgeLength, int numberOfVertices, Vector3DReadOnly faceNormal)
   {
      List<Point3D> vertices = nextCircleBasedConvexPolygon3D(random, centerMinMax, maxEdgeLength, numberOfVertices, faceNormal);
      Face3D face3D = new Face3D(faceNormal);
      vertices.forEach(vertex -> face3D.addVertex(new Vertex3D(vertex), 0.0));
      return face3D;
   }

   public static List<Point3D> nextCircleBasedConvexPolygon3D(Random random, double centerMinMax, double maxEdgeLength, int numberOfVertices, Vector3DReadOnly planeNormal)
   {
      List<Point2D> circleBasedConvexPolygon2D = EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, centerMinMax, maxEdgeLength, numberOfVertices);
      List<Point3D> circleBasedConvexPolygon3D = circleBasedConvexPolygon2D.stream().map(Point3D::new).collect(Collectors.toList());
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslationZ(EuclidCoreRandomTools.nextDouble(random, centerMinMax));
      transform.setRotation(EuclidGeometryTools.axisAngleFromZUpToVector3D(planeNormal));
      circleBasedConvexPolygon3D.forEach(transform::transform);
      return circleBasedConvexPolygon3D;
   }
}
