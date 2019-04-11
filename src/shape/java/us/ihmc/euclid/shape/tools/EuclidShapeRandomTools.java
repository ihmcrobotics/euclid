package us.ihmc.euclid.shape.tools;

import java.util.Collection;
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
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeFactories;
import us.ihmc.euclid.shape.convexPolytope.tools.IcoSphereFactory;
import us.ihmc.euclid.shape.convexPolytope.tools.IcoSphereFactory.GeometryMesh3D;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.shape.primitives.Ellipsoid3D;
import us.ihmc.euclid.shape.primitives.PointShape3D;
import us.ihmc.euclid.shape.primitives.Ramp3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.Torus3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * This class provides random generators to generate random shapes.
 * <p>
 * The main application is for writing JUnit Tests.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class EuclidShapeRandomTools
{
   private EuclidShapeRandomTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Generates a random a box 3D.
    * <ul>
    * <li>{@code size}<sub>i</sub> &in; [0.0; 1.0].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code orientation.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    * 
    * @param random the random generator to use.
    * @return the random box 3D.
    */
   public static Box3D nextBox3D(Random random)
   {
      return nextBox3D(random, 0.0, 1.0);
   }

   /**
    * Generates a random a box 3D.
    * <ul>
    * <li>{@code size}<sub>i</sub> &in; [minSize; maxSize].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code orientation.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    * 
    * @param random the random generator to use.
    * @param minSize the minimum value for each component of the box size.
    * @param maxSize the maximum value for each component of the box size.
    * @return the random box 3D.
    * @throws RuntimeException if {@code minSize > maxSize}.
    */
   public static Box3D nextBox3D(Random random, double minSize, double maxSize)
   {
      return new Box3D(EuclidCoreRandomTools.nextRigidBodyTransform(random), EuclidCoreRandomTools.nextDouble(random, minSize, maxSize),
                       EuclidCoreRandomTools.nextDouble(random, minSize, maxSize), EuclidCoreRandomTools.nextDouble(random, minSize, maxSize));
   }

   public static Capsule3D nextCapsule3D(Random random)
   {
      return nextCapsule3D(random, 0.0, 1.0, 0.0, 1.0);
   }

   public static Capsule3D nextCapsule3D(Random random, double minLength, double maxLength, double minRadius, double maxRadius)
   {
      return new Capsule3D(EuclidCoreRandomTools.nextPoint3D(random), EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0),
                           EuclidCoreRandomTools.nextDouble(random, minLength, maxLength), EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius));
   }

   public static Cylinder3D nextCylinder3D(Random random)
   {
      return nextCylinder3D(random, 0.0, 1.0, 0.0, 1.0);
   }

   public static Cylinder3D nextCylinder3D(Random random, double minLength, double maxLength, double minRadius, double maxRadius)
   {
      return new Cylinder3D(EuclidCoreRandomTools.nextPoint3D(random), EuclidCoreRandomTools.nextVector3D(random),
                            EuclidCoreRandomTools.nextDouble(random, minLength, maxLength), EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius));
   }

   public static Ellipsoid3D nextEllipsoid3D(Random random)
   {
      return nextEllipsoid3D(random, 0.0, 1.0);
   }

   public static Ellipsoid3D nextEllipsoid3D(Random random, double minRadius, double maxRadius)
   {
      return new Ellipsoid3D(EuclidCoreRandomTools.nextRigidBodyTransform(random), EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius),
                             EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius), EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius));
   }

   public static PointShape3D nextPointShape3D(Random random)
   {
      return new PointShape3D(EuclidCoreRandomTools.nextPoint3D(random));
   }

   public static PointShape3D nextPointShape3D(Random random, double minMax)
   {
      return new PointShape3D(EuclidCoreRandomTools.nextPoint3D(random, minMax));
   }

   public static Ramp3D nextRamp3D(Random random)
   {
      return nextRamp3D(random, 0.0, 1.0);
   }

   public static Ramp3D nextRamp3D(Random random, double minSize, double maxSize)
   {
      return new Ramp3D(EuclidCoreRandomTools.nextRigidBodyTransform(random), EuclidCoreRandomTools.nextDouble(random, minSize, maxSize),
                        EuclidCoreRandomTools.nextDouble(random, minSize, maxSize), EuclidCoreRandomTools.nextDouble(random, minSize, maxSize));
   }

   public static Sphere3D nextSphere3D(Random random)
   {
      return nextSphere3D(random, 0.0, 1.0);
   }

   public static Sphere3D nextSphere3D(Random random, double minRadius, double maxRadius)
   {
      return new Sphere3D(EuclidCoreRandomTools.nextPoint3D(random), EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius));
   }

   public static Torus3D nextTorus3D(Random random)
   {
      return nextTorus3D(random, 0.5, 2.0, 0.0, 0.5);
   }

   public static Torus3D nextTorus3D(Random random, double minRadius, double maxRadius, double minTubeRadius, double maxTubeRadius)
   {
      return new Torus3D(EuclidCoreRandomTools.nextPoint3D(random), EuclidCoreRandomTools.nextVector3D(random),
                         EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius),
                         EuclidCoreRandomTools.nextDouble(random, minTubeRadius, maxTubeRadius));
   }

   public static Point2D nextPoint2DInTriangle(Random random, Point2DReadOnly a, Point2DReadOnly b, Point2DReadOnly c)
   {
      return new Point2D(nextPoint3DInTriangle(random, new Point3D(a), new Point3D(b), new Point3D(c)));
   }

   public static Point3D nextPoint3DInTriangle(Random random, Point3DReadOnly a, Point3DReadOnly b, Point3DReadOnly c)
   {
      // Generating random point using the method introduced: http://mathworld.wolfram.com/TrianglePointPicking.html
      double alpha0 = random.nextDouble();
      double alpha1 = random.nextDouble();

      if (alpha0 + alpha1 > 1.0)
      { // The generated would be outside the triangle. Instead of discarding this point, we're folding the parallelogram.
         alpha0 = 1.0 - alpha0;
         alpha1 = 1.0 - alpha1;
      }

      Vector3D v0 = new Vector3D();
      Vector3D v1 = new Vector3D();
      v0.sub(b, a);
      v1.sub(c, a);

      Point3D next = new Point3D(a);
      next.scaleAdd(alpha0, v0, next);
      next.scaleAdd(alpha1, v1, next);

      return next;
   }

   public static Point3D nextPoint3DInTetrahedron(Random random, Point3DReadOnly a, Point3DReadOnly b, Point3DReadOnly c, Point3DReadOnly d)
   {
      // Generating random point using the method introduced: http://vcg.isti.cnr.it/publications/papers/rndtetra_a.pdf
      double s = random.nextDouble();
      double t = random.nextDouble();
      double u = random.nextDouble();

      if (s + t > 1.0)
      {
         s = 1.0 - s;
         t = 1.0 - t;
      }

      if (s + t + u > 1.0)
      {
         if (t + u > 1.0)
         {
            double tOld = t;
            t = 1.0 - u;
            u = 1.0 - s - tOld;
         }
         else
         {
            double sOld = s;
            s = 1.0 - t - u;
            u = sOld + t + u - 1.0;
         }
      }

      Vector3D v0 = new Vector3D();
      Vector3D v1 = new Vector3D();
      Vector3D v2 = new Vector3D();
      v0.sub(b, a);
      v1.sub(c, a);
      v2.sub(d, a);

      Point3D next = new Point3D(a);
      next.scaleAdd(s, v0, next);
      next.scaleAdd(t, v1, next);
      next.scaleAdd(u, v2, next);

      return next;
   }

   // FIXME Generates points that are mostly sitting around the average of the points.
   public static Point3D nextWeightedAverage(Random random, Collection<? extends Point3DReadOnly> points)
   {
      return nextWeightedAverage(random, points.toArray(new Point3DReadOnly[points.size()]));
   }

   public static Point3D nextWeightedAverage(Random random, Point3DReadOnly[] points)
   {
      double[] weights = new double[points.length];
      double sum = 0.0;

      for (int j = 0; j < weights.length; j++)
         sum += weights[j] = random.nextDouble();

      Point3D next = new Point3D();

      for (int j = 0; j < weights.length; j++)
         next.scaleAdd(weights[j], points[j], next);

      next.scale(1.0 / sum);

      return next;
   }

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
         return nextPoint3DInTriangle(random, face3D.getCentroid(), edge.getOrigin(), edge.getDestination());
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
      transform.getTranslation().sub(point, nextPoint3DInTetrahedron(random, vertices.get(0), vertices.get(1), vertices.get(2), vertices.get(3)));
      tetrahedron.applyTransform(transform);

      assert tetrahedron.isPointInside(point);

      return tetrahedron;
   }
}
