package us.ihmc.euclid.shape.convexPolytope.tools;

import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.Face3D;
import us.ihmc.euclid.shape.convexPolytope.Vertex3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.IcoSphereFactory.GeometryMesh3D;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class EuclidPolytopeRandomTools
{
   public static Face3D nextCircleBasedFace3D(Random random)
   {
      return nextCircleBasedFace3D(random, 5.0);
   }

   public static Face3D nextCircleBasedFace3D(Random random, double centerMinMax)
   {
      return nextCircleBasedFace3D(random, centerMinMax, 1.0, 15);
   }

   public static Face3D nextCircleBasedFace3D(Random random, double centerMinMax, double maxEdgeLength, int numberOfVertices)
   {
      return nextCircleBasedFace3D(random, centerMinMax, maxEdgeLength, numberOfVertices, EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0));
   }

   public static Face3D nextCircleBasedFace3D(Random random, double centerMinMax, double maxEdgeLength, int numberOfVertices, Vector3DReadOnly faceNormal)
   {
      List<Point3D> vertices = nextCircleBasedConvexPolygon3D(random, centerMinMax, maxEdgeLength, numberOfVertices, faceNormal);
      Face3D face3D = new Face3D(faceNormal);
      vertices.forEach(vertex -> face3D.addVertex(new Vertex3D(vertex)));
      return face3D;
   }

   public static List<Point3D> nextCircleBasedConvexPolygon3D(Random random, double centerMinMax, double maxEdgeLength, int numberOfVertices,
                                                              Vector3DReadOnly planeNormal)
   {
      List<Point2D> circleBasedConvexPolygon2D = EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, centerMinMax, maxEdgeLength,
                                                                                                          numberOfVertices);
      List<Point3D> circleBasedConvexPolygon3D = circleBasedConvexPolygon2D.stream().map(Point3D::new).collect(Collectors.toList());
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslationZ(EuclidCoreRandomTools.nextDouble(random, centerMinMax));
      transform.setRotation(EuclidGeometryTools.axisAngleFromZUpToVector3D(planeNormal));
      circleBasedConvexPolygon3D.forEach(transform::transform);
      return circleBasedConvexPolygon3D;
   }

   public static Point3D nextPoint3DOnFace3D(Random random, Face3DReadOnly face3D)
   {
      if (face3D.isEmpty())
      {
         return null;
      }
      else
      {
         HalfEdge3DReadOnly edge = face3D.getEdge(random.nextInt(face3D.getNumberOfEdges()));
         return EuclidShapeRandomTools.nextPoint3DInTriangle(random, face3D.getCentroid(), edge.getOrigin(), edge.getDestination());
      }
   }

   public static ConvexPolytope3D nextConvexPolytope3D(Random random)
   {
      switch (random.nextInt(7))
      {
      case 0:
         return nextConeConvexPolytope3D(random);
      case 1:
         return nextCubeConvexPolytope3D(random);
      case 2:
         return nextCylinderConvexPolytope3D(random);
      case 3:
         return nextIcosahedronBasedConvexPolytope3D(random);
      case 4:
         return nextIcoSphereBasedConvexPolytope3D(random);
      case 5:
         return nextPointCloudBasedConvexPolytope3D(random);
      case 6:
         return nextPyramidConvexPolytope3D(random);
      default:
         throw new RuntimeException("Unexpected state.");
      }
   }

   public static ConvexPolytope3D nextConvexPolytope3DWithEdgeCases(Random random)
   {
      switch (random.nextInt(8))
      {
      case 0:
         return nextConeConvexPolytope3D(random);
      case 1:
         return nextCubeConvexPolytope3D(random);
      case 2:
         return nextCylinderConvexPolytope3D(random);
      case 3:
         return nextIcosahedronBasedConvexPolytope3D(random);
      case 4:
         return nextIcoSphereBasedConvexPolytope3D(random);
      case 5:
         return nextPointCloudBasedConvexPolytope3D(random, 5.0, 5.0, random.nextInt(101));
      case 6:
         return nextPyramidConvexPolytope3D(random);
      case 7:
         return nextSingleEdgeConvexPolytope3D(random);
      default:
         throw new RuntimeException("Unexpected state.");
      }
   }

   public static ConvexPolytope3D nextConeConvexPolytope3D(Random random)
   {
      return nextConeConvexPolytope3D(random, 5.0);
   }

   public static ConvexPolytope3D nextConeConvexPolytope3D(Random random, double centerMinMax)
   {
      return nextConeConvexPolytope3D(random, centerMinMax, 0.1, 5.0, 0.1, 5.0, 3, 50);
   }

   public static ConvexPolytope3D nextConeConvexPolytope3D(Random random, double centerMinMax, double heightMin, double heightMax, double radiusMin,
                                                           double radiusMax, int divisionsMin, int divisionsMax)
   {
      List<Point3D> coneVertices = EuclidPolytopeFactories.newConeVertices(EuclidCoreRandomTools.nextDouble(random, heightMin, heightMax),
                                                                           EuclidCoreRandomTools.nextDouble(random, radiusMin, radiusMax),
                                                                           random.nextInt(divisionsMax - divisionsMin + 1) + divisionsMin);
      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      transform.setTranslation(EuclidCoreRandomTools.nextPoint3D(random, centerMinMax));
      coneVertices.forEach(transform::transform);
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(coneVertices));
   }

   public static ConvexPolytope3D nextCubeConvexPolytope3D(Random random)
   {
      return nextCubeConvexPolytope3D(random, 5.0);
   }

   public static ConvexPolytope3D nextCubeConvexPolytope3D(Random random, double centerMinMax)
   {
      return nextCubeConvexPolytope3D(random, centerMinMax, 0.1, 5.0);
   }

   public static ConvexPolytope3D nextCubeConvexPolytope3D(Random random, double centerMinMax, double edgeLengthMin, double edgeLengthMax)
   {
      List<Point3D> cubeVertices = EuclidPolytopeFactories.newCubeVertices(EuclidCoreRandomTools.nextDouble(random, edgeLengthMin, edgeLengthMax));
      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      transform.setTranslation(EuclidCoreRandomTools.nextPoint3D(random, centerMinMax));
      cubeVertices.forEach(transform::transform);
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(cubeVertices));
   }

   public static ConvexPolytope3D nextCylinderConvexPolytope3D(Random random)
   {
      return nextCylinderConvexPolytope3D(random, 5.0);
   }

   public static ConvexPolytope3D nextCylinderConvexPolytope3D(Random random, double centerMinMax)
   {
      return nextCylinderConvexPolytope3D(random, centerMinMax, 0.1, 5.0, 0.1, 5.0, 3, 50);
   }

   public static ConvexPolytope3D nextCylinderConvexPolytope3D(Random random, double centerMinMax, double lengthMin, double lengthMax, double radiusMin,
                                                               double radiusMax, int divisionsMin, int divisionsMax)
   {
      List<Point3D> cylinderVertices = EuclidPolytopeFactories.newCylinderVertices(EuclidCoreRandomTools.nextDouble(random, lengthMin, lengthMax),
                                                                                   EuclidCoreRandomTools.nextDouble(random, radiusMin, radiusMax),
                                                                                   random.nextInt(divisionsMax - divisionsMin + 1) + divisionsMin);
      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      transform.setTranslation(EuclidCoreRandomTools.nextPoint3D(random, centerMinMax));
      cylinderVertices.forEach(transform::transform);
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(cylinderVertices));
   }

   public static ConvexPolytope3D nextIcosahedronBasedConvexPolytope3D(Random random)
   {
      return nextIcosahedronBasedConvexPolytope3D(random, 5.0);
   }

   public static ConvexPolytope3D nextIcosahedronBasedConvexPolytope3D(Random random, double centerMinMax)
   {
      return nextIcosahedronBasedConvexPolytope3D(random, centerMinMax, 0.1, 5.0);
   }

   public static ConvexPolytope3D nextIcosahedronBasedConvexPolytope3D(Random random, double centerMinMax, double radiusMin, double radiusMax)
   {
      return nextIcoSphereBasedConvexPolytope3D(random, centerMinMax, 0, radiusMin, radiusMax);
   }

   public static ConvexPolytope3D nextIcoSphereBasedConvexPolytope3D(Random random)
   {
      return nextIcoSphereBasedConvexPolytope3D(random, 5.0);
   }

   public static ConvexPolytope3D nextIcoSphereBasedConvexPolytope3D(Random random, double centerMinMax)
   {
      return nextIcoSphereBasedConvexPolytope3D(random, centerMinMax, 0.1, 5.0);
   }

   public static ConvexPolytope3D nextIcoSphereBasedConvexPolytope3D(Random random, double centerMinMax, double radiusMin, double radiusMax)
   {
      return nextIcoSphereBasedConvexPolytope3D(random, centerMinMax, random.nextInt(3), radiusMin, radiusMax);
   }

   public static ConvexPolytope3D nextIcoSphereBasedConvexPolytope3D(Random random, double centerMinMax, int recursionLevel, double radiusMin, double radiusMax)
   {
      GeometryMesh3D icoSphere = IcoSphereFactory.newIcoSphere(recursionLevel);
      AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
      transform.setTranslation(EuclidCoreRandomTools.nextPoint3D(random, centerMinMax));
      transform.setScale(EuclidCoreRandomTools.nextDouble(random, radiusMin, radiusMax));
      icoSphere.applyTransform(transform);

      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(icoSphere.getVertices()));
   }

   public static ConvexPolytope3D nextPointCloudBasedConvexPolytope3D(Random random)
   {
      return nextPointCloudBasedConvexPolytope3D(random, 5.0);
   }

   public static ConvexPolytope3D nextPointCloudBasedConvexPolytope3D(Random random, double centerMinMax)
   {
      return nextPointCloudBasedConvexPolytope3D(random, centerMinMax, 5.0);
   }

   public static ConvexPolytope3D nextPointCloudBasedConvexPolytope3D(Random random, double centerMinMax, double minMax)
   {
      return nextPointCloudBasedConvexPolytope3D(random, centerMinMax, minMax, 100);
   }

   public static ConvexPolytope3D nextPointCloudBasedConvexPolytope3D(Random random, double centerMinMax, double minMax, int numberOfPossiblePoints)
   {
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(EuclidGeometryRandomTools.nextPointCloud3D(random, centerMinMax, minMax,
                                                                                                                 numberOfPossiblePoints)));
   }

   public static ConvexPolytope3D nextPyramidConvexPolytope3D(Random random)
   {
      return nextPyramidConvexPolytope3D(random, 5.0);
   }

   public static ConvexPolytope3D nextPyramidConvexPolytope3D(Random random, double centerMinMax)
   {
      return nextPyramidConvexPolytope3D(random, centerMinMax, 0.1, 5.0, 0.1, 5.0, 0.1, 5.0);
   }

   public static ConvexPolytope3D nextPyramidConvexPolytope3D(Random random, double centerMinMax, double heightMin, double heightMax, double baseLengthMin,
                                                              double baseLengthMax, double baseWidthMin, double baseWidthMax)
   {
      List<Point3D> pyramidVertices = EuclidPolytopeFactories.newPyramidVertices(EuclidCoreRandomTools.nextDouble(random, heightMin, heightMax),
                                                                                 EuclidCoreRandomTools.nextDouble(random, baseLengthMin, baseLengthMax),
                                                                                 EuclidCoreRandomTools.nextDouble(random, baseWidthMin, baseWidthMax));
      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      transform.setTranslation(EuclidCoreRandomTools.nextPoint3D(random, centerMinMax));
      pyramidVertices.forEach(transform::transform);
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pyramidVertices));
   }

   public static ConvexPolytope3D nextSingleEdgeConvexPolytope3D(Random random)
   {
      return nextSingleEdgeConvexPolytope3D(random, 5.0);
   }

   public static ConvexPolytope3D nextSingleEdgeConvexPolytope3D(Random random, double centerMinMax)
   {
      return nextSingleEdgeConvexPolytope3D(random, centerMinMax, 5.0);
   }

   public static ConvexPolytope3D nextSingleEdgeConvexPolytope3D(Random random, double centerMinMax, double minMax)
   {
      LineSegment3D lineSegment3D = EuclidGeometryRandomTools.nextLineSegment3D(random, minMax);
      lineSegment3D.translate(EuclidCoreRandomTools.nextPoint3D(random, centerMinMax));
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(lineSegment3D.getFirstEndpoint(), lineSegment3D.getSecondEndpoint()));
   }

   public static ConvexPolytope3D nextTetrahedronContainingPoint3D(Random random, Point3DReadOnly point)
   {
      return nextTetrahedronContainingPoint3D(random, point, 5.0);
   }

   public static ConvexPolytope3D nextTetrahedronContainingPoint3D(Random random, Point3DReadOnly point, double minMax)
   {
      List<Point3D> vertices = EuclidGeometryRandomTools.nextPointCloud3D(random, 0.0, minMax, 4);
      ConvexPolytope3D tetrahedron = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(vertices));

      assert tetrahedron.getNumberOfVertices() == 4;

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getTranslation()
               .sub(point, EuclidShapeRandomTools.nextPoint3DInTetrahedron(random, vertices.get(0), vertices.get(1), vertices.get(2), vertices.get(3)));
      tetrahedron.applyTransform(transform);

      assert tetrahedron.isPointInside(point);

      return tetrahedron;
   }
}
