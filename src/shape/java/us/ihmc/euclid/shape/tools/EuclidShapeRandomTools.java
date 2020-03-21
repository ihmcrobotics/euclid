package us.ihmc.euclid.shape.tools;

import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.Face3D;
import us.ihmc.euclid.shape.convexPolytope.Vertex3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeFactories;
import us.ihmc.euclid.shape.convexPolytope.tools.IcoSphereFactory;
import us.ihmc.euclid.shape.convexPolytope.tools.IcoSphereFactory.TriangleMesh3D;
import us.ihmc.euclid.shape.primitives.*;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DBasics;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
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

   public static Shape3DPose nextShape3DPose(Random random)
   {
      return new Shape3DPose(EuclidCoreRandomTools.nextRigidBodyTransform(random));
   }

   /**
    * Generates a random box 3D.
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
    * Generates a random box 3D.
    * <ul>
    * <li>{@code size}<sub>i</sub> &in; [{@code minSize}; {@code maxSize}].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code orientation.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    *
    * @param random  the random generator to use.
    * @param minSize the minimum value for each component of the box size.
    * @param maxSize the maximum value for each component of the box size.
    * @return the random box 3D.
    * @throws RuntimeException if {@code minSize > maxSize}.
    */
   public static Box3D nextBox3D(Random random, double minSize, double maxSize)
   {
      return new Box3D(EuclidCoreRandomTools.nextRigidBodyTransform(random),
                       EuclidCoreRandomTools.nextDouble(random, minSize, maxSize),
                       EuclidCoreRandomTools.nextDouble(random, minSize, maxSize),
                       EuclidCoreRandomTools.nextDouble(random, minSize, maxSize));
   }

   /**
    * Generates a random capsule 3D.
    * <ul>
    * <li>{@code length} &in; [0.0; 1.0].
    * <li>{@code radius} &in; [0.0; 1.0].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code axis} is generated using
    * {@link EuclidCoreRandomTools#nextVector3DWithFixedLength(Random, double)} with a length of 1.0.
    * </ul>
    *
    * @param random the random generator to use.
    * @return the random capsule 3D.
    */
   public static Capsule3D nextCapsule3D(Random random)
   {
      return nextCapsule3D(random, 0.0, 1.0, 0.0, 1.0);
   }

   /**
    * Generates a random capsule 3D.
    * <ul>
    * <li>{@code length} &in; [{@code minLength}; {@code maxLength}].
    * <li>{@code radius} &in; [{@code minRadius}; {@code maxRadius}].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code axis} is generated using
    * {@link EuclidCoreRandomTools#nextVector3DWithFixedLength(Random, double)} with a length of 1.0.
    * </ul>
    *
    * @param random    the random generator to use.
    * @param minLength the minimum value for the length.
    * @param maxLength the maximum value for the length.
    * @param minRadius the minimum value for the radius.
    * @param maxRadius the maximum value for the radius.
    * @return the random capsule 3D.
    * @throws RuntimeException if {@code minLength > maxLength} or {@code minRadius > maxRadius}.
    */
   public static Capsule3D nextCapsule3D(Random random, double minLength, double maxLength, double minRadius, double maxRadius)
   {
      return new Capsule3D(EuclidCoreRandomTools.nextPoint3D(random),
                           EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0),
                           EuclidCoreRandomTools.nextDouble(random, minLength, maxLength),
                           EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius));
   }

   /**
    * Generates a random cylinder 3D.
    * <ul>
    * <li>{@code length} &in; [0.0; 1.0].
    * <li>{@code radius} &in; [0.0; 1.0].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code axis} is generated using
    * {@link EuclidCoreRandomTools#nextVector3DWithFixedLength(Random, double)} with a length of 1.0.
    * </ul>
    *
    * @param random the random generator to use.
    * @return the random cylinder 3D.
    */
   public static Cylinder3D nextCylinder3D(Random random)
   {
      return nextCylinder3D(random, 0.0, 1.0, 0.0, 1.0);
   }

   /**
    * Generates a random cylinder 3D.
    * <ul>
    * <li>{@code length} &in; [{@code minLength}; {@code maxLength}].
    * <li>{@code radius} &in; [{@code minRadius}; {@code maxRadius}].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code axis} is generated using
    * {@link EuclidCoreRandomTools#nextVector3DWithFixedLength(Random, double)} with a length of 1.0.
    * </ul>
    *
    * @param random    the random generator to use.
    * @param minLength the minimum value for the length.
    * @param maxLength the maximum value for the length.
    * @param minRadius the minimum value for the radius.
    * @param maxRadius the maximum value for the radius.
    * @return the random cylinder 3D.
    * @throws RuntimeException if {@code minLength > maxLength} or {@code minRadius > maxRadius}.
    */
   public static Cylinder3D nextCylinder3D(Random random, double minLength, double maxLength, double minRadius, double maxRadius)
   {
      return new Cylinder3D(EuclidCoreRandomTools.nextPoint3D(random),
                            EuclidCoreRandomTools.nextVector3D(random),
                            EuclidCoreRandomTools.nextDouble(random, minLength, maxLength),
                            EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius));
   }

   /**
    * Generates a random ellipsoid 3D.
    * <ul>
    * <li>{@code radii}<sub>i</sub> &in; [0.0; 1.0].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code orientation.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    *
    * @param random the random generator to use.
    * @return the random radii 3D.
    */
   public static Ellipsoid3D nextEllipsoid3D(Random random)
   {
      return nextEllipsoid3D(random, 0.0, 1.0);
   }

   /**
    * Generates a random ellipsoid 3D.
    * <ul>
    * <li>{@code radii}<sub>i</sub> &in; [{@code minSize}; {@code maxSize}].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code orientation.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    *
    * @param random    the random generator to use.
    * @param minRadius the minimum value for each component of the ellipsoid radius.
    * @param maxRadius the maximum value for each component of the ellipsoid radius.
    * @return the random ellipsoid 3D.
    * @throws RuntimeException if {@code minRadius > maxRadius}.
    */
   public static Ellipsoid3D nextEllipsoid3D(Random random, double minRadius, double maxRadius)
   {
      return new Ellipsoid3D(EuclidCoreRandomTools.nextRigidBodyTransform(random),
                             EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius),
                             EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius),
                             EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius));
   }

   /**
    * Generates a random point shape.
    * <p>
    * {@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random point shape.
    */
   public static PointShape3D nextPointShape3D(Random random)
   {
      return new PointShape3D(EuclidCoreRandomTools.nextPoint3D(random));
   }

   /**
    * Generates a random point.
    * <ul>
    * <li>{@code position.x} &in; [-{@code minMax}; {@code minMax}].
    * <li>{@code position.y} &in; [-{@code minMax}; {@code minMax}].
    * <li>{@code position.z} &in; [-{@code minMax}; {@code minMax}].
    * </ul>
    *
    * @param random the random generator to use.
    * @param minMax the maximum absolute value for each coordinate.
    * @return the random point.
    * @throws RuntimeException if {@code minMax < 0}.
    */
   public static PointShape3D nextPointShape3D(Random random, double minMax)
   {
      return new PointShape3D(EuclidCoreRandomTools.nextPoint3D(random, minMax));
   }

   /**
    * Generates a random ramp 3D.
    * <ul>
    * <li>{@code size}<sub>i</sub> &in; [0.0; 1.0].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code orientation.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    *
    * @param random the random generator to use.
    * @return the random ramp 3D.
    */
   public static Ramp3D nextRamp3D(Random random)
   {
      return nextRamp3D(random, 0.0, 1.0);
   }

   /**
    * Generates a random ramp 3D.
    * <ul>
    * <li>{@code size}<sub>i</sub> &in; [{@code minSize}; {@code maxSize}].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code orientation.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    *
    * @param random  the random generator to use.
    * @param minSize the minimum value for each component of the ramp size.
    * @param maxSize the maximum value for each component of the ramp size.
    * @return the random ramp 3D.
    * @throws RuntimeException if {@code minSize > maxSize}.
    */
   public static Ramp3D nextRamp3D(Random random, double minSize, double maxSize)
   {
      return new Ramp3D(EuclidCoreRandomTools.nextRigidBodyTransform(random),
                        EuclidCoreRandomTools.nextDouble(random, minSize, maxSize),
                        EuclidCoreRandomTools.nextDouble(random, minSize, maxSize),
                        EuclidCoreRandomTools.nextDouble(random, minSize, maxSize));
   }

   /**
    * Generates a random sphere 3D.
    * <ul>
    * <li>{@code radius} &in; [0.0; 1.0].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    *
    * @param random the random generator to use.
    * @return the random sphere 3D.
    */
   public static Sphere3D nextSphere3D(Random random)
   {
      return nextSphere3D(random, 0.0, 1.0);
   }

   /**
    * Generates a random sphere 3D.
    * <ul>
    * <li>{@code radius} &in; [{@code minRadius}; {@code maxRadius}].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    *
    * @param random    the random generator to use.
    * @param minRadius the minimum value for the radius.
    * @param maxRadius the maximum value for the radius.
    * @return the random sphere 3D.
    * @throws RuntimeException if {@code minRadius > maxRadius}.
    */
   public static Sphere3D nextSphere3D(Random random, double minRadius, double maxRadius)
   {
      return new Sphere3D(EuclidCoreRandomTools.nextPoint3D(random), EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius));
   }

   /**
    * Generates a random torus 3D.
    * <ul>
    * <li>{@code radius} &in; [0.5; 2.0].
    * <li>{@code tubeRadius} &in; [0.0; 0.5].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code axis} is generated using
    * {@link EuclidCoreRandomTools#nextVector3DWithFixedLength(Random, double)} with a length of 1.0.
    * </ul>
    *
    * @param random the random generator to use.
    * @return the random capsule 3D.
    */
   public static Torus3D nextTorus3D(Random random)
   {
      return nextTorus3D(random, 0.5, 2.0, 0.0, 0.5);
   }

   /**
    * Generates a random capsule 3D.
    * <ul>
    * <li>{@code radius} &in; [{@code minRadius}; {@code maxRadius}].
    * <li>{@code tubeRadius} &in; [{@code minTubeRadius}; {@code maxTubeRadius}].
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code axis} is generated using
    * {@link EuclidCoreRandomTools#nextVector3DWithFixedLength(Random, double)} with a length of 1.0.
    * </ul>
    *
    * @param random        the random generator to use.
    * @param minRadius     the minimum value for the radius.
    * @param maxRadius     the maximum value for the radius.
    * @param minTubeRadius the minimum value for the tube radius.
    * @param maxTubeRadius the maximum value for the tube radius.
    * @return the random capsule 3D.
    * @throws RuntimeException if {@code minRadius > maxRadius} or
    *                          {@code minTubeRadius > maxTubeRadius}.
    */
   public static Torus3D nextTorus3D(Random random, double minRadius, double maxRadius, double minTubeRadius, double maxTubeRadius)
   {
      return new Torus3D(EuclidCoreRandomTools.nextPoint3D(random),
                         EuclidCoreRandomTools.nextVector3D(random),
                         EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius),
                         EuclidCoreRandomTools.nextDouble(random, minTubeRadius, maxTubeRadius));
   }

   /**
    * Generates a random face 3D by defining a random circle onto which the vertices are randomly
    * positioned.
    *
    * @param random the random generator to use.
    * @return the random face.
    */
   public static Face3D nextCircleBasedFace3D(Random random)
   {
      return nextCircleBasedFace3D(random, 5.0);
   }

   /**
    * Generates a random face 3D by defining a random circle onto which the vertices are randomly
    * positioned.
    *
    * @param random       the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinates of the circle's center.
    * @return the random face.
    */
   public static Face3D nextCircleBasedFace3D(Random random, double centerMinMax)
   {
      return nextCircleBasedFace3D(random, centerMinMax, 1.0, 15);
   }

   /**
    * Generates a random face 3D by defining a random circle onto which the vertices are randomly
    * positioned.
    *
    * @param random           the random generator to use.
    * @param centerMinMax     the maximum absolute value for each coordinates of the circle's center.
    * @param maxEdgeLength    maximum distance between two successive vertices constraining the size of
    *                         the random circle.
    * @param numberOfVertices the size of the convex polygon.
    * @return the random face.
    */
   public static Face3D nextCircleBasedFace3D(Random random, double centerMinMax, double maxEdgeLength, int numberOfVertices)
   {
      return nextCircleBasedFace3D(random, centerMinMax, maxEdgeLength, numberOfVertices, EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0));
   }

   /**
    * Generates a random face 3D by defining a random circle onto which the vertices are randomly
    * positioned.
    *
    * @param random           the random generator to use.
    * @param centerMinMax     the maximum absolute value for each coordinates of the circle's center.
    * @param maxEdgeLength    maximum distance between two successive vertices constraining the size of
    *                         the random circle.
    * @param numberOfVertices the size of the convex polygon.
    * @param faceNormal       the normal of the face and the axis of revolution of the circle. Not
    *                         modified.
    * @return the random face.
    */
   public static Face3D nextCircleBasedFace3D(Random random, double centerMinMax, double maxEdgeLength, int numberOfVertices, Vector3DReadOnly faceNormal)
   {
      List<Point3D> vertices = nextCircleBasedConvexPolygon3D(random, centerMinMax, maxEdgeLength, numberOfVertices, faceNormal);
      Face3D face3D = new Face3D(faceNormal);
      vertices.forEach(vertex -> face3D.addVertex(new Vertex3D(vertex)));
      return face3D;
   }

   /**
    * Generates a random convex polygon 3D by defining a random circle onto which the vertices are
    * randomly positioned.
    *
    * @param random           the random generator to use.
    * @param centerMinMax     the maximum absolute value for each coordinates of the circle's center.
    * @param maxEdgeLength    maximum distance between two successive vertices constraining the size of
    *                         the random circle.
    * @param numberOfVertices the size of the convex polygon.
    * @param planeNormal      defines the normal of the plane onto which the vertices are positioned.
    * @return the random convex polygon 3D.
    */
   public static List<Point3D> nextCircleBasedConvexPolygon3D(Random random, double centerMinMax, double maxEdgeLength, int numberOfVertices,
                                                              Vector3DReadOnly planeNormal)
   {
      List<Point2D> circleBasedConvexPolygon2D = EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random,
                                                                                                          centerMinMax,
                                                                                                          maxEdgeLength,
                                                                                                          numberOfVertices);
      List<Point3D> circleBasedConvexPolygon3D = circleBasedConvexPolygon2D.stream().map(Point3D::new).collect(Collectors.toList());
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslationZ(EuclidCoreRandomTools.nextDouble(random, centerMinMax));
      transform.setRotation(EuclidGeometryTools.axisAngleFromZUpToVector3D(planeNormal));
      circleBasedConvexPolygon3D.forEach(transform::transform);
      return circleBasedConvexPolygon3D;
   }

   /**
    * Generates a random point that is constrained to lie inside a given face.
    *
    * @param random the random generator to use.
    * @param face3D the bounding face. Not modified.
    * @return the random point 3D.
    */
   public static Point3D nextPoint3DOnFace3D(Random random, Face3DReadOnly face3D)
   {
      if (face3D.isEmpty())
      {
         return null;
      }
      else
      {
         HalfEdge3DReadOnly edge = face3D.getEdge(random.nextInt(face3D.getNumberOfEdges()));
         return EuclidGeometryRandomTools.nextPoint3DInTriangle(random, face3D.getCentroid(), edge.getOrigin(), edge.getDestination());
      }
   }

   /**
    * Generates a random convex polytope 3D.
    * <p>
    * The convex polytope is generated by picking at random one of the following generators:
    * <ul>
    * <li>{@link #nextConeConvexPolytope3D(Random)}.
    * <li>{@link #nextCubeConvexPolytope3D(Random)}.
    * <li>{@link #nextCylinderConvexPolytope3D(Random)}.
    * <li>{@link #nextIcosahedronBasedConvexPolytope3D(Random)}.
    * <li>{@link #nextIcoSphereBasedConvexPolytope3D(Random)}.
    * <li>{@link #nextPointCloudBasedConvexPolytope3D(Random)}.
    * <li>{@link #nextPyramidConvexPolytope3D(Random)}.
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random convex polytope 3D.
    */
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

   /**
    * Generates a random convex polytope 3D in the same way as
    * {@link #nextConeConvexPolytope3D(Random)} and in addition allows the generation of:
    * <ul>
    * <li>empty convex polytope.
    * <li>convex polytope with a single vertex.
    * <li>convex polytope with a single edge.
    * <li>convex polytope with a single face.
    * </ul>
    *
    * @param random the random generator to use.
    * @return the random convex polytope 3D.
    */
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

   /**
    * Generates a convex polytope by discretizing a randomly generated cone 3D.
    *
    * @param random the random generator to use.
    * @return the random convex polytope 3D.
    */
   public static ConvexPolytope3D nextConeConvexPolytope3D(Random random)
   {
      return nextConeConvexPolytope3D(random, 5.0);
   }

   /**
    * Generates a convex polytope by discretizing a randomly generated cone 3D.
    *
    * @param random       the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinate for the cone's base center.
    * @return the random convex polytope 3D.
    */
   public static ConvexPolytope3D nextConeConvexPolytope3D(Random random, double centerMinMax)
   {
      return nextConeConvexPolytope3D(random, centerMinMax, 0.1, 5.0, 0.1, 5.0, 3, 50);
   }

   /**
    * Generates a convex polytope by discretizing a randomly generated cone 3D.
    *
    * @param random       the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinate for the cone's base center.
    * @param heightMin    the minimum value for the height of the cone.
    * @param heightMax    the maximum value for the height of the cone.
    * @param radiusMin    the minimum value for the radius of the cone base.
    * @param radiusMax    the maximum value for the radius of the cone base.
    * @param divisionsMin the minimum number of divisions for discretizing the cone.
    * @param divisionsMax the maximum number of divisions for discretizing the cone.
    * @return the random convex polytope 3D.
    */
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

   /**
    * Generates a convex polytope from a randomly generated cube 3D.
    *
    * @param random the random generator to use.
    * @return the random convex polytope 3D.
    */
   public static ConvexPolytope3D nextCubeConvexPolytope3D(Random random)
   {
      return nextCubeConvexPolytope3D(random, 5.0);
   }

   /**
    * Generates a convex polytope from a randomly generated cube 3D.
    *
    * @param random       the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinate for the convex polytope's
    *                     centroid.
    * @return the random convex polytope 3D.
    */
   public static ConvexPolytope3D nextCubeConvexPolytope3D(Random random, double centerMinMax)
   {
      return nextCubeConvexPolytope3D(random, centerMinMax, 0.1, 5.0);
   }

   /**
    * Generates a convex polytope from a randomly generated cube 3D.
    *
    * @param random        the random generator to use.
    * @param centerMinMax  the maximum absolute value for each coordinate for the convex polytope's
    *                      centroid.
    * @param edgeLengthMin the minimum value for the cube's edge length.
    * @param edgeLengthMax the maximum value for the cube's edge length.
    * @return the random convex polytope 3D.
    */
   public static ConvexPolytope3D nextCubeConvexPolytope3D(Random random, double centerMinMax, double edgeLengthMin, double edgeLengthMax)
   {
      List<Point3D> cubeVertices = EuclidPolytopeFactories.newCubeVertices(EuclidCoreRandomTools.nextDouble(random, edgeLengthMin, edgeLengthMax));
      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      transform.setTranslation(EuclidCoreRandomTools.nextPoint3D(random, centerMinMax));
      cubeVertices.forEach(transform::transform);
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(cubeVertices));
   }

   /**
    * Generates a convex polytope from a randomly generated cylinder 3D.
    *
    * @param random the random generator to use.
    * @return the random convex polytope 3D.
    */
   public static ConvexPolytope3D nextCylinderConvexPolytope3D(Random random)
   {
      return nextCylinderConvexPolytope3D(random, 5.0);
   }

   /**
    * Generates a convex polytope from a randomly generated cylinder 3D.
    *
    * @param random       the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinate for the convex polytope's
    *                     centroid.
    * @return the random convex polytope 3D.
    */
   public static ConvexPolytope3D nextCylinderConvexPolytope3D(Random random, double centerMinMax)
   {
      return nextCylinderConvexPolytope3D(random, centerMinMax, 0.1, 5.0, 0.1, 5.0, 3, 50);
   }

   /**
    * Generates a convex polytope from a randomly generated cylinder 3D.
    *
    * @param random       the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinate for the convex polytope's
    *                     centroid.
    * @param lengthMin    the minimum value for the length of the cylinder.
    * @param lengthMax    the maximum value for the length of the cylinder.
    * @param radiusMin    the minimum value for the radius of the cylinder.
    * @param radiusMax    the maximum value for the radius of the cylinder.
    * @param divisionsMin the minimum number of divisions for discretizing the cylinder.
    * @param divisionsMax the maximum number of divisions for discretizing the cylinder.
    * @return the random convex polytope 3D.
    */
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

   /**
    * Generates a convex polytope from a randomly generated icosahedron 3D.
    *
    * @param random the random generator to use.
    * @return the random convex polytope 3D.
    */
   public static ConvexPolytope3D nextIcosahedronBasedConvexPolytope3D(Random random)
   {
      return nextIcosahedronBasedConvexPolytope3D(random, 5.0);
   }

   /**
    * Generates a convex polytope from a randomly generated icosahedron 3D.
    *
    * @param random       the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinate for the convex polytope's
    *                     centroid.
    * @return the random convex polytope 3D.
    */
   public static ConvexPolytope3D nextIcosahedronBasedConvexPolytope3D(Random random, double centerMinMax)
   {
      return nextIcosahedronBasedConvexPolytope3D(random, centerMinMax, 0.1, 5.0);
   }

   /**
    * Generates a convex polytope from a randomly generated icosahedron 3D.
    *
    * @param random       the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinate for the convex polytope's
    *                     centroid.
    * @param radiusMin    the minimum value for the radius of the circumscribed sphere of the
    *                     icosahedron.
    * @param radiusMax    the maximum value for the radius of the circumscribed sphere of the
    *                     icosahedron.
    * @return the random convex polytope 3D.
    */
   public static ConvexPolytope3D nextIcosahedronBasedConvexPolytope3D(Random random, double centerMinMax, double radiusMin, double radiusMax)
   {
      return nextIcoSphereBasedConvexPolytope3D(random, centerMinMax, 0, radiusMin, radiusMax);
   }

   /**
    * Generates a convex polytope from a randomly generated ico-sphere 3D.
    *
    * @param random the random generator to use.
    * @return the random convex polytope 3D.
    * @see IcoSphereFactory
    */
   public static ConvexPolytope3D nextIcoSphereBasedConvexPolytope3D(Random random)
   {
      return nextIcoSphereBasedConvexPolytope3D(random, 5.0);
   }

   /**
    * Generates a convex polytope from a randomly generated ico-sphere 3D.
    *
    * @param random       the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinate for the convex polytope's
    *                     centroid.
    * @return the random convex polytope 3D.
    * @see IcoSphereFactory
    */
   public static ConvexPolytope3D nextIcoSphereBasedConvexPolytope3D(Random random, double centerMinMax)
   {
      return nextIcoSphereBasedConvexPolytope3D(random, centerMinMax, 0.1, 5.0);
   }

   /**
    * Generates a convex polytope from a randomly generated ico-sphere 3D.
    *
    * @param random       the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinate for the convex polytope's
    *                     centroid.
    * @param radiusMin    the minimum value for the radius of the ico-sphere.
    * @param radiusMax    the maximum value for the radius of the ico-sphere.
    * @return the random convex polytope 3D.
    * @see IcoSphereFactory
    */
   public static ConvexPolytope3D nextIcoSphereBasedConvexPolytope3D(Random random, double centerMinMax, double radiusMin, double radiusMax)
   {
      return nextIcoSphereBasedConvexPolytope3D(random, centerMinMax, random.nextInt(3), radiusMin, radiusMax);
   }

   /**
    * Generates a convex polytope from a randomly generated ico-sphere 3D.
    *
    * @param random         the random generator to use.
    * @param centerMinMax   the maximum absolute value for each coordinate for the convex polytope's
    *                       centroid.
    * @param recursionLevel the recursion level that defines the resolution of the ico-sphere.
    * @param radiusMin      the minimum value for the radius of the ico-sphere.
    * @param radiusMax      the maximum value for the radius of the ico-sphere.
    * @return the random convex polytope 3D.
    * @see IcoSphereFactory
    */
   public static ConvexPolytope3D nextIcoSphereBasedConvexPolytope3D(Random random, double centerMinMax, int recursionLevel, double radiusMin, double radiusMax)
   {
      TriangleMesh3D icoSphere = IcoSphereFactory.newIcoSphere(recursionLevel);
      AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
      transform.setTranslation(EuclidCoreRandomTools.nextPoint3D(random, centerMinMax));
      transform.setScale(EuclidCoreRandomTools.nextDouble(random, radiusMin, radiusMax));
      icoSphere.applyTransform(transform);

      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(icoSphere.getVertices()));
   }

   /**
    * Generates a convex polytope from a randomly generated pointcloud 3D.
    *
    * @param random the random generator to use.
    * @return the random convex polytope 3D.
    */
   public static ConvexPolytope3D nextPointCloudBasedConvexPolytope3D(Random random)
   {
      return nextPointCloudBasedConvexPolytope3D(random, 5.0);
   }

   /**
    * Generates a convex polytope from a randomly generated pointcloud 3D.
    *
    * @param random       the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinate for the convex polytope's
    *                     centroid.
    * @return the random convex polytope 3D.
    */
   public static ConvexPolytope3D nextPointCloudBasedConvexPolytope3D(Random random, double centerMinMax)
   {
      return nextPointCloudBasedConvexPolytope3D(random, centerMinMax, 5.0);
   }

   /**
    * Generates a convex polytope from a randomly generated pointcloud 3D.
    *
    * @param random       the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinate for the convex polytope's
    *                     centroid.
    * @param minMax       the range of the point cloud in the three directions.
    * @return the random convex polytope 3D.
    */
   public static ConvexPolytope3D nextPointCloudBasedConvexPolytope3D(Random random, double centerMinMax, double minMax)
   {
      return nextPointCloudBasedConvexPolytope3D(random, centerMinMax, minMax, 100);
   }

   /**
    * Generates a convex polytope from a randomly generated pointcloud 3D.
    *
    * @param random                 the random generator to use.
    * @param centerMinMax           the maximum absolute value for each coordinate for the convex
    *                               polytope's centroid.
    * @param minMax                 the range of the point cloud in the three directions.
    * @param numberOfPossiblePoints the size of the point cloud to generate that is used for computing
    *                               the random convex polytope. The size of the resulting convex
    *                               polytope will be less than {@code numberOfPossiblePoints}.
    * @return the random convex polytope 3D.
    */
   public static ConvexPolytope3D nextPointCloudBasedConvexPolytope3D(Random random, double centerMinMax, double minMax, int numberOfPossiblePoints)
   {
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(EuclidGeometryRandomTools.nextPointCloud3D(random,
                                                                                                                 centerMinMax,
                                                                                                                 minMax,
                                                                                                                 numberOfPossiblePoints)));
   }

   /**
    * Generates a convex polytope from a randomly generated pyramid.
    *
    * @param random the random generator to use.
    * @return the random convex polytope 3D.
    */
   public static ConvexPolytope3D nextPyramidConvexPolytope3D(Random random)
   {
      return nextPyramidConvexPolytope3D(random, 5.0);
   }

   /**
    * Generates a convex polytope from a randomly generated pyramid.
    *
    * @param random       the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinate of the pyramid's base center.
    * @return the random convex polytope 3D.
    */
   public static ConvexPolytope3D nextPyramidConvexPolytope3D(Random random, double centerMinMax)
   {
      return nextPyramidConvexPolytope3D(random, centerMinMax, 0.1, 5.0, 0.1, 5.0, 0.1, 5.0);
   }

   /**
    * Generates a convex polytope from a randomly generated pyramid.
    *
    * @param random        the random generator to use.
    * @param centerMinMax  the maximum absolute value for each coordinate of the pyramid's base center.
    * @param heightMin     the minimum value for the height of the pyramid.
    * @param heightMax     the maximum value for the height of the pyramid.
    * @param baseLengthMin the minimum value for the length of the pyramid's base.
    * @param baseLengthMax the maximum value for the length of the pyramid's base.
    * @param baseWidthMin  the minimum value for the width of the pyramid's base.
    * @param baseWidthMax  the maximum value for the width of the pyramid's base.
    * @return the random convex polytope 3D.
    */
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

   /**
    * Generates a convex polytope from a single randomly generated edge.
    *
    * @param random the random generator to use.
    * @return the random convex polytope 3D.
    */
   public static ConvexPolytope3D nextSingleEdgeConvexPolytope3D(Random random)
   {
      return nextSingleEdgeConvexPolytope3D(random, 5.0);
   }

   /**
    * Generates a convex polytope from a single randomly generated edge.
    *
    * @param random       the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinate of the edge's center.
    * @return the random convex polytope 3D.
    */
   public static ConvexPolytope3D nextSingleEdgeConvexPolytope3D(Random random, double centerMinMax)
   {
      return nextSingleEdgeConvexPolytope3D(random, centerMinMax, 5.0);
   }

   /**
    * Generates a convex polytope from a single randomly generated edge.
    *
    * @param random       the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinate of the edge's center.
    * @param minMax       the range of the egde in the three directions.
    * @return the random convex polytope 3D.
    */
   public static ConvexPolytope3D nextSingleEdgeConvexPolytope3D(Random random, double centerMinMax, double minMax)
   {
      LineSegment3D lineSegment3D = EuclidGeometryRandomTools.nextLineSegment3D(random, minMax);
      Point3DBasics midpoint = lineSegment3D.midpoint();
      midpoint.negate();
      lineSegment3D.translate(midpoint);
      lineSegment3D.translate(EuclidCoreRandomTools.nextPoint3D(random, centerMinMax));
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(lineSegment3D.getFirstEndpoint(), lineSegment3D.getSecondEndpoint()));
   }

   /**
    * Generates a convex polytope from a randomly generated tetrahedron that contains the given point.
    *
    * @param random the random generator to use.
    * @param point  the point the tetrahedron must contain. Not modified.
    * @return the random convex polytope 3D.
    */
   public static ConvexPolytope3D nextTetrahedronContainingPoint3D(Random random, Point3DReadOnly point)
   {
      return nextTetrahedronContainingPoint3D(random, point, 5.0);
   }

   /**
    * Generates a convex polytope from a randomly generated tetrahedron that contains the given point.
    *
    * @param random the random generator to use.
    * @param point  the point the tetrahedron must contain. Not modified.
    * @param minMax the range of the tetrahedron in the three directions.
    * @return the random convex polytope 3D.
    */
   public static ConvexPolytope3D nextTetrahedronContainingPoint3D(Random random, Point3DReadOnly point, double minMax)
   {
      List<Point3D> vertices = EuclidGeometryRandomTools.nextPointCloud3D(random, 0.0, minMax, 4);
      ConvexPolytope3D tetrahedron = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(vertices));

      assert tetrahedron.getNumberOfVertices() == 4;

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getTranslation()
               .sub(point, EuclidGeometryRandomTools.nextPoint3DInTetrahedron(random, vertices.get(0), vertices.get(1), vertices.get(2), vertices.get(3)));
      tetrahedron.applyTransform(transform);

      assert tetrahedron.isPointInside(point);

      return tetrahedron;
   }

   /**
    * Generates a random shape 3D.
    * <p>
    * The shape is generated by picking at random one of the following generators:
    * <ul>
    * <li>{@link #nextBox3D(Random)}.
    * <li>{@link #nextCapsule3D(Random)}.
    * <li>{@link #nextConvexPolytope3D(Random)}.
    * <li>{@link #nextCylinder3D(Random)}.
    * <li>{@link #nextEllipsoid3D(Random)}.
    * <li>{@link #nextPointShape3D(Random)}.
    * <li>{@link #nextRamp3D(Random)}.
    * <li>{@link #nextSphere3D(Random)}.
    * <li>{@link #nextTorus3D(Random)}.
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random shape 3D.
    */
   public static Shape3DBasics nextShape3D(Random random)
   {
      switch (random.nextInt(9))
      {
         case 0:
            return nextBox3D(random);
         case 1:
            return nextCapsule3D(random);
         case 2:
            return nextConvexPolytope3D(random);
         case 3:
            return nextCylinder3D(random);
         case 4:
            return nextEllipsoid3D(random);
         case 5:
            return nextPointShape3D(random);
         case 6:
            return nextRamp3D(random);
         case 7:
            return nextSphere3D(random);
         case 8:
            return nextTorus3D(random);
         default:
            throw new RuntimeException("Unexpected state.");
      }
   }

   /**
    * Generates a random convex shape 3D.
    * <p>
    * The shape is generated by picking at random one of the following generators:
    * <ul>
    * <li>{@link #nextBox3D(Random)}.
    * <li>{@link #nextCapsule3D(Random)}.
    * <li>{@link #nextConvexPolytope3D(Random)}.
    * <li>{@link #nextCylinder3D(Random)}.
    * <li>{@link #nextEllipsoid3D(Random)}.
    * <li>{@link #nextPointShape3D(Random)}.
    * <li>{@link #nextRamp3D(Random)}.
    * <li>{@link #nextSphere3D(Random)}.
    * </ul>
    * </p>
    * <p>
    * This generator differs from {@link #nextShape3D(Random)} by excluding {@link Torus3D} that is a
    * concave shape.
    * </p>
    *
    * @param random the random generator to use.
    * @return the random convex shape 3D.
    */
   public static Shape3DBasics nextConvexShape3D(Random random)
   {
      switch (random.nextInt(8))
      {
         case 0:
            return nextBox3D(random);
         case 1:
            return nextCapsule3D(random);
         case 2:
            return nextConvexPolytope3D(random);
         case 3:
            return nextCylinder3D(random);
         case 4:
            return nextEllipsoid3D(random);
         case 5:
            return nextPointShape3D(random);
         case 6:
            return nextRamp3D(random);
         case 7:
            return nextSphere3D(random);
         default:
            throw new RuntimeException("Unexpected state.");
      }
   }

   /**
    * Generates a random collision result.
    * <p>
    * The random collision result does <b>not</b> ensure geometric consistency and is generated as
    * follows:
    * <ul>
    * <li>The shapes A and B are generated using {@link #nextShape3D(Random)}.
    * <li>The distance is within [0.0; 1.0] and its sign is negative when the shapes are colliding, and
    * positive otherwise. This does <b>not</b> represent the actual distance separating the two shapes
    * nor the actual penetration between the two shapes.
    * <li>The coordinates of both point on A and B are within [-1.0, 1.0]. The points are generated
    * independently from the shapes, i.e. they do <b>not</b> lie on the surface of their respective
    * shape.
    * <li>Both normal vector on A and B are random unit-length vectors. The normals are generated
    * independently from the shapes, i.e. they do <b>not</b> represent the surface normal of their
    * respective shape.
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random collision result.
    */
   public static EuclidShape3DCollisionResult nextEuclidShape3DCollisionResult(Random random)
   {
      EuclidShape3DCollisionResult next = new EuclidShape3DCollisionResult();
      next.setShapeA(nextShape3D(random));
      next.setShapeB(nextShape3D(random));
      next.setShapesAreColliding(random.nextBoolean());
      next.setSignedDistance(next.areShapesColliding() ? -random.nextDouble() : random.nextDouble());
      next.getPointOnA().set(EuclidCoreRandomTools.nextPoint3D(random));
      next.getPointOnB().set(EuclidCoreRandomTools.nextPoint3D(random));
      next.getNormalOnA().set(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0));
      next.getNormalOnB().set(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0));
      return next;
   }
}
