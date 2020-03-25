package us.ihmc.euclid.geometry.tools;

import static us.ihmc.euclid.tools.EuclidCoreRandomTools.*;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.LineSegment1D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.Orientation2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.Triangle3D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * This class provides random generators to generate random geometry objects.
 * <p>
 * The main application is for writing JUnit Tests.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class EuclidGeometryRandomTools
{
   private EuclidGeometryRandomTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Generates a random line 2D.
    * <p>
    * <ul>
    * <li>{@code point}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code direction}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random line 2D.
    */
   public static Line2D nextLine2D(Random random)
   {
      return new Line2D(nextPoint2D(random), nextVector2D(random));
   }

   /**
    * Generates a random line 2D.
    * <p>
    * <ul>
    * <li>{@code point}<sub>i</sub> &in; [-pointMinMax; pointMinMax].
    * <li>{@code direction}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random      the random generator to use.
    * @param pointMinMax the maximum absolute value for each coordinate of the line's point.
    * @return the random line 2D.
    * @throws RuntimeException if {@code pointMinMax < 0}.
    */
   public static Line2D nextLine2D(Random random, double pointMinMax)
   {
      return new Line2D(nextPoint2D(random, pointMinMax), nextVector2D(random));
   }

   /**
    * Generates a random line 3D.
    * <p>
    * <ul>
    * <li>{@code point}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code direction}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random line 3D.
    */
   public static Line3D nextLine3D(Random random)
   {
      return new Line3D(nextPoint3D(random), nextVector3D(random));
   }

   /**
    * Generates a random line 3D.
    * <p>
    * <ul>
    * <li>{@code point}<sub>i</sub> &in; [-pointMinMax; pointMinMax].
    * <li>{@code direction}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random      the random generator to use.
    * @param pointMinMax the maximum absolute value for each coordinate of the line's point.
    * @return the random line 3D.
    * @throws RuntimeException if {@code pointMinMax < 0}.
    */
   public static Line3D nextLine3D(Random random, double pointMinMax)
   {
      return new Line3D(nextPoint3D(random, pointMinMax), nextVector3D(random));
   }

   /**
    * Generates a random line segment 1D.
    * <p>
    * <ul>
    * <li>{@code firstEndpoint} &in; [-1.0; 1.0].
    * <li>{@code secondEndpoint} &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random line segment 1D.
    */
   public static LineSegment1D nextLineSegment1D(Random random)
   {
      return new LineSegment1D(nextDouble(random), nextDouble(random));
   }

   /**
    * Generates a random line segment 1D.
    * <p>
    * <ul>
    * <li>{@code firstEndpoint} &in; [-minMax; minMax].
    * <li>{@code secondEndpoint} &in; [-minMax; minMax].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param minMax the maximum absolute value for each line segment's endpoints.
    * @return the random line segment 1D.
    * @throws RuntimeException if {@code minMax < 0}.
    */
   public static LineSegment1D nextLineSegment1D(Random random, double minMax)
   {
      return new LineSegment1D(nextDouble(random, minMax), nextDouble(random, minMax));
   }

   /**
    * Generates a random line segment 2D.
    * <p>
    * <ul>
    * <li>{@code firstEndpoint}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code secondEndpoint}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random line segment 2D.
    */
   public static LineSegment2D nextLineSegment2D(Random random)
   {
      return new LineSegment2D(nextPoint2D(random), nextPoint2D(random));
   }

   /**
    * Generates a random line segment 2D.
    * <p>
    * <ul>
    * <li>{@code firstEndpoint}<sub>i</sub> &in; [-minMax; minMax].
    * <li>{@code secondEndpoint}<sub>i</sub> &in; [-minMax; minMax].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param minMax the maximum absolute value for each coordinate of the line segment's endpoints.
    * @return the random line segment 2D.
    * @throws RuntimeException if {@code minMax < 0}.
    */
   public static LineSegment2D nextLineSegment2D(Random random, double minMax)
   {
      return new LineSegment2D(nextPoint2D(random, minMax), nextPoint2D(random, minMax));
   }

   /**
    * Generates a random line segment 3D.
    * <p>
    * <ul>
    * <li>{@code firstEndpoint}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code secondEndpoint}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random line segment 3D.
    */
   public static LineSegment3D nextLineSegment3D(Random random)
   {
      return new LineSegment3D(nextPoint3D(random), nextPoint3D(random));
   }

   /**
    * Generates a random line segment 3D.
    * <p>
    * <ul>
    * <li>{@code firstEndpoint}<sub>i</sub> &in; [-minMax; minMax].
    * <li>{@code secondEndpoint}<sub>i</sub> &in; [-minMax; minMax].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param minMax the maximum absolute value for each coordinate of the line segment's endpoints.
    * @return the random line segment 3D.
    * @throws RuntimeException if {@code minMax < 0}.
    */
   public static LineSegment3D nextLineSegment3D(Random random, double minMax)
   {
      return new LineSegment3D(nextPoint3D(random, minMax), nextPoint3D(random, minMax));
   }

   /**
    * Generates a random bounding box from random center location and random size.
    *
    * @param random the random generator to use.
    * @return the random bounding box.
    */
   public static BoundingBox2D nextBoundingBox2D(Random random)
   {
      return nextBoundingBox2D(random, 1.0, 1.0);
   }

   /**
    * Generates a random bounding box from random center location and random size.
    *
    * @param random       the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinate of the bounding box center.
    * @param sizeMax      the maximum size along each axis for the bounding box.
    * @return the random bounding box.
    * @throws RuntimeException if {@code centerMinMax < 0} or {@code sizeMax < 0}.
    */
   public static BoundingBox2D nextBoundingBox2D(Random random, double centerMinMax, double sizeMax)
   {
      Point2D center = nextPoint2D(random, centerMinMax);
      Vector2D halfSize = nextVector2D(random, 0.0, sizeMax / 2.0);
      return BoundingBox2D.createUsingCenterAndPlusMinusVector(center, halfSize);
   }

   /**
    * Generates a random bounding box from random center location and random size.
    *
    * @param random the random generator to use.
    * @return the random bounding box.
    */
   public static BoundingBox3D nextBoundingBox3D(Random random)
   {
      return nextBoundingBox3D(random, 1.0, 1.0);
   }

   /**
    * Generates a random bounding box from random center location and random size.
    *
    * @param random       the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinate of the bounding box center.
    * @param sizeMax      the maximum size along each axis for the bounding box.
    * @return the random bounding box.
    * @throws RuntimeException if {@code centerMinMax < 0} or {@code sizeMax < 0}.
    */
   public static BoundingBox3D nextBoundingBox3D(Random random, double centerMinMax, double sizeMax)
   {
      Point3D center = nextPoint3D(random, centerMinMax);
      Vector3D halfSize = nextVector3D(random, 0.0, sizeMax / 2.0);
      return BoundingBox3D.createUsingCenterAndPlusMinusVector(center, halfSize);
   }

   /**
    * Generates a random orientation 2D.
    * <p>
    * <ul>
    * <li>{@code yaw} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random orientation 2D.
    */
   public static Orientation2D nextOrientation2D(Random random)
   {
      return new Orientation2D(nextDouble(random, Math.PI));
   }

   /**
    * Generates a random orientation 2D.
    * <p>
    * <ul>
    * <li>{@code yaw} &in; [-{@code minMax}; {@code minMax}].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param minMax the maximum absolute value orientation 2D's angle.
    * @return the random orientation 2D.
    * @throws RuntimeException if {@code pointMinMax < 0}.
    */
   public static Orientation2D nextOrientation2D(Random random, double minMax)
   {
      return new Orientation2D(nextDouble(random, minMax));
   }

   /**
    * Generates a random plane 3D from a random point and a random unit-vector.
    * <p>
    * Each coordinate of the random point are in [-1, 1].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random plane 3D.
    */
   public static Plane3D nextPlane3D(Random random)
   {
      return nextPlane3D(random, 1.0);
   }

   /**
    * Generates a random plane 3D from a random point and a random unit-vector.
    *
    * @param random      the random generator to use.
    * @param pointMinMax the maximum absolute value for each coordinate of the random point.
    * @return the random plane 3D.
    * @throws RuntimeException if {@code pointMinMax < 0}.
    */
   public static Plane3D nextPlane3D(Random random, double pointMinMax)
   {
      Point3D pointOnPlane = nextPoint3D(random, pointMinMax);
      Vector3D planeNormal = nextVector3DWithFixedLength(random, 1.0);
      return new Plane3D(pointOnPlane, planeNormal);
   }

   /**
    * Generates a random pose 2D.
    * <p>
    * <ul>
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code orientation} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random pose 2D.
    */
   public static Pose2D nextPose2D(Random random)
   {
      return new Pose2D(nextPoint2D(random), nextOrientation2D(random));
   }

   /**
    * Generates a random pose 2D.
    * <p>
    * <ul>
    * <li>{@code position}<sub>i</sub> &in; [-{@code positionMinMax}; {@code positionMinMax}].
    * <li>{@code orientation} &in; [-{@code orientationMinMax}; {@code orientationMinMax}].
    * </ul>
    * </p>
    *
    * @param random            the random generator to use.
    * @param positionMinMax    the maximum absolute value for each coordinate of the line's point.
    * @param orientationMinMax the maximum absolute value orientation 2D's angle.
    * @return the random pose 2D.
    * @throws RuntimeException if {@code positionMinMax < 0} or {@code orientationMinMax < 0}.
    */
   public static Pose2D nextPose2D(Random random, double positionMinMax, double orientationMinMax)
   {
      return new Pose2D(nextPoint2D(random, positionMinMax), nextOrientation2D(random, orientationMinMax));
   }

   /**
    * Generates a random pose 3D.
    * <p>
    * <ul>
    * <li>{@code position}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code orientation.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random pose 3D.
    */
   public static Pose3D nextPose3D(Random random)
   {
      return new Pose3D(nextPoint3D(random), nextQuaternion(random));
   }

   /**
    * Generates a random pose 3D.
    * <p>
    * <ul>
    * <li>{@code position}<sub>X</sub> &in; [-{@code maxAbsoluteX}; {@code maxAbsoluteX}].
    * <li>{@code position}<sub>Y</sub> &in; [-{@code maxAbsoluteY}; {@code maxAbsoluteY}].
    * <li>{@code position}<sub>Z</sub> &in; [-{@code maxAbsoluteZ}; {@code maxAbsoluteZ}].
    * <li>{@code orientation.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    * </p>
    *
    * @param random       the random generator to use.
    * @param maxAbsoluteX the maximum absolute value for the x-coordinate of the position part of the
    *                     pose 3D.
    * @param maxAbsoluteY the maximum absolute value for the y-coordinate of the position part of the
    *                     pose 3D.
    * @param maxAbsoluteZ the maximum absolute value for the z-coordinate of the position part of the
    *                     pose 3D.
    * @return the random pose 3D.
    */
   public static Pose3D nextPose3D(Random random, double maxAbsoluteX, double maxAbsoluteY, double maxAbsoluteZ)
   {
      return new Pose3D(nextPoint3D(random, maxAbsoluteX, maxAbsoluteY, maxAbsoluteZ), nextQuaternion(random));
   }

   /**
    * Generates a random pose 3D.
    * <p>
    * <ul>
    * <li>{@code position}<sub>i</sub> &in; [-{@code positionMinMax}; {@code positionMinMax}].
    * <li>{@code orientation.getAngle()} &in; [-{@code orientationMinMax}; {@code orientationMinMax}].
    * </ul>
    * </p>
    *
    * @param random            the random generator to use.
    * @param positionMinMax    the maximum absolute value for each coordinate of the pose's position.
    * @param orientationMinMax the maximum absolute value of the rotation angle for the pose's
    *                          orientation.
    * @return the random pose 3D.
    * @throws RuntimeException if {@code positionMinMax < 0} or {@code orientationMinMax < 0}.
    */
   public static Pose3D nextPose3D(Random random, double positionMinMax, double orientationMinMax)
   {
      return new Pose3D(nextPoint3D(random, positionMinMax), nextQuaternion(random, orientationMinMax));
   }

   /**
    * Generates a random convex polygon given the maximum absolute coordinate value of its vertices and
    * the size of the point cloud from which it is generated.
    *
    * @param random the random generator to use.
    * @return the random convex polygon.
    * @throws RuntimeException if {@code maxAbsoluteXY < 0}.
    */
   public static ConvexPolygon2D nextConvexPolygon2D(Random random)
   {
      return nextConvexPolygon2D(random, 1.0, 10);
   }

   /**
    * Generates a random convex polygon given the maximum absolute coordinate value of its vertices and
    * the size of the point cloud from which it is generated.
    *
    * @param random                 the random generator to use.
    * @param maxAbsoluteXY          the maximum absolute value for each coordinate of the vertices.
    * @param numberOfPossiblePoints the size of the point cloud to generate that is used for computing
    *                               the random convex polygon. The size of the resulting convex polygon
    *                               will be less than {@code numberOfPossiblePoints}.
    * @return the random convex polygon.
    * @throws RuntimeException if {@code maxAbsoluteXY < 0}.
    */
   public static ConvexPolygon2D nextConvexPolygon2D(Random random, double maxAbsoluteXY, int numberOfPossiblePoints)
   {
      List<Point2D> vertices = EuclidGeometryRandomTools.nextPointCloud2D(random, 0.0, maxAbsoluteXY, numberOfPossiblePoints);
      return new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(vertices));
   }

   /**
    * Generates a random 2D point cloud given a random average, range, and size.
    *
    * @param random         the random generator to use.
    * @param centerMinMax   the maximum absolute value for each coordinate of the random average.
    * @param minMax         the range of the point cloud in the three directions.
    * @param numberOfPoints the size of the point cloud to generate.
    * @return the random 2D point cloud.
    */
   public static List<Point2D> nextPointCloud2D(Random random, double centerMinMax, double minMax, int numberOfPoints)
   {
      List<Point2D> pointCloud2D = new ArrayList<>();

      Point2D center = nextPoint2D(random, centerMinMax);

      for (int i = 0; i < numberOfPoints; i++)
      {
         Point2D point = nextPoint2D(random, minMax);
         point.add(center);
         pointCloud2D.add(point);
      }

      return pointCloud2D;
   }

   /**
    * Generates a random 3D point cloud given a random average, range, and size.
    *
    * @param random         the random generator to use.
    * @param centerMinMax   the maximum absolute value for each coordinate of the random average.
    * @param minMax         the range of the point cloud in the three directions.
    * @param numberOfPoints the size of the point cloud to generate.
    * @return the random 3D point cloud.
    */
   public static List<Point3D> nextPointCloud3D(Random random, double centerMinMax, double minMax, int numberOfPoints)
   {
      List<Point3D> pointCloud3D = new ArrayList<>();

      Point3D center = nextPoint3D(random, centerMinMax);

      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D point = nextPoint3D(random, minMax);
         point.add(center);
         pointCloud3D.add(point);
      }

      return pointCloud3D;
   }

   /**
    * Generates a random convex polygon 2D by defining a random circle onto which the vertices are
    * randomly positioned.
    *
    * @param random           the random generator to use.
    * @param centerMinMax     the maximum absolute value for each coordinates of the circle's center.
    * @param maxEdgeLength    maximum distance between two successive vertices constraining the size of
    *                         the random circle.
    * @param numberOfVertices the size of the convex polygon.
    * @return the random convex polygon 2D.
    */
   public static List<Point2D> nextCircleBasedConvexPolygon2D(Random random, double centerMinMax, double maxEdgeLength, int numberOfVertices)
   {
      if (numberOfVertices == 0)
         return Collections.emptyList();
      if (numberOfVertices == 1)
         return Collections.singletonList(nextPoint2D(random, centerMinMax));
      if (numberOfVertices == 2)
      {
         Vector2D halfEdgeLentgh = nextVector2DWithFixedLength(random, 0.5 * maxEdgeLength * random.nextDouble());
         Point2D center = nextPoint2D(random, centerMinMax);
         Point2D a = new Point2D();
         Point2D b = new Point2D();
         a.add(center, halfEdgeLentgh);
         b.sub(center, halfEdgeLentgh);
         List<Point2D> points = new ArrayList<>();
         points.add(a);
         points.add(b);
         return points;
      }

      // Generating random angles from vertex to vertex
      double[] dTheta = new double[numberOfVertices];
      double sum = 0.0;

      for (int i = 0; i < numberOfVertices; i++)
      {
         dTheta[i] = nextDouble(random, 0.001, 1.0);
         sum += dTheta[i];
      }
      // Adding the angle for the last segment
      sum += random.nextDouble();

      // Re-scaling the all the angles such that sum is equal to 2*pi
      double scale = 2.0 * Math.PI / sum;

      for (int i = 0; i < numberOfVertices; i++)
         dTheta[i] *= scale;

      // Generating the clockwise ordered vertices distributed on a circle centered at (0, 0).
      List<Point2D> clockwiseVertices = new ArrayList<>();
      double theta = 0.0;
      // Add a random yaw angle on all the vertices
      double yaw = nextDouble(random, Math.PI);
      clockwiseVertices.add(new Point2D(EuclidCoreTools.cos(yaw), EuclidCoreTools.sin(yaw)));

      for (int i = 1; i < numberOfVertices; i++)
      {
         theta -= dTheta[i];
         double x = EuclidCoreTools.cos(theta + yaw);
         double y = EuclidCoreTools.sin(theta + yaw);
         clockwiseVertices.add(new Point2D(x, y));
      }

      // Re-scaling the vertices such that the max edge length is contained in [0, maxEdgeLength]
      double currentMaxEdgeLength = 0.0;
      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2D vertex = clockwiseVertices.get(i);
         Point2D nextVertex = clockwiseVertices.get((i + 1) % numberOfVertices);
         currentMaxEdgeLength = Math.max(currentMaxEdgeLength, vertex.distance(nextVertex));
      }

      // Limiting the min value from the random to prevent obtaining a polygon that is way too small
      scale = nextDouble(random, 0.5, 1.0) * maxEdgeLength / currentMaxEdgeLength;
      for (int i = 0; i < numberOfVertices; i++)
         clockwiseVertices.get(i).scale(scale);

      // By definition in this library, a convex polygon's first vertex is located has the min x coordinate.
      // If more than one vertex is located at min X, then it is the vertex with the max Y that is the first.
      int indexOfFirstVertex = 0;
      Point2D firstVertex = clockwiseVertices.get(0);

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2D currentVertex = clockwiseVertices.get(i);
         if (firstVertex.getX() > currentVertex.getX())
         {
            firstVertex = currentVertex;
            indexOfFirstVertex = i;
         }
         else if (firstVertex.getX() == currentVertex.getX() && firstVertex.getY() < currentVertex.getY())
         {
            firstVertex = currentVertex;
            indexOfFirstVertex = i;
         }
      }

      // Make a new list with the vertices properly ordered and shifted to be around a random center
      Point2D center = nextPoint2D(random, centerMinMax);
      List<Point2D> convexPolygon2D = new ArrayList<>();

      for (int i = 0; i < numberOfVertices; i++)
      {
         int indexInOtherList = (i + indexOfFirstVertex) % numberOfVertices;
         Point2D vertex = clockwiseVertices.get(indexInOtherList);
         vertex.add(center);
         convexPolygon2D.add(vertex);
      }

      return convexPolygon2D;
   }

   /**
    * Generates a random 3D triangle.
    * <p>
    * Each coordinate of each vertex is generated randomly within the range [-1, 1].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random triangle.
    */
   public static Triangle3D nextTriangle3D(Random random)
   {
      return nextTriangle3D(random, 1.0);
   }

   /**
    * Generates a random 3D triangle.
    * <p>
    * Each coordinate of each vertex is generated randomly within the range [-{@code minMax},
    * {@code minMax}].
    * </p>
    *
    * @param random the random generator to use.
    * @param minMax the maximum absolute value for each coordinate.
    * @return the random triangle.
    * @throws RuntimeException if {@code minMax < 0}.
    */
   public static Triangle3D nextTriangle3D(Random random, double minMax)
   {
      return new Triangle3D(nextPoint3D(random, minMax), nextPoint3D(random, minMax), nextPoint3D(random, minMax));
   }

   /**
    * Generates a fixed-size supplier of random vertex 2D.
    *
    * @param random the random generator to use.
    * @return the random supplier.
    */
   public static Vertex2DSupplier nextVertex2DSupplier(Random random)
   {
      return nextVertex2DSupplier(random, 20);
   }

   /**
    * Generates a fixed-size supplier of random vertex 2D.
    *
    * @param random           the random generator to use.
    * @param numberOfVertices the supplier's size.
    * @return the random supplier.
    */
   public static Vertex2DSupplier nextVertex2DSupplier(Random random, int numberOfVertices)
   {
      List<Point2D> vertices = IntStream.range(0, numberOfVertices).mapToObj(i -> nextPoint2D(random)).collect(Collectors.toList());
      return Vertex2DSupplier.asVertex2DSupplier(vertices);
   }

   /**
    * Generates a fixed-size supplier of random vertex 3D.
    *
    * @param random the random generator to use.
    * @return the random supplier.
    */
   public static Vertex3DSupplier nextVertex3DSupplier(Random random)
   {
      return nextVertex3DSupplier(random, 20);
   }

   /**
    * Generates a fixed-size supplier of random vertex 3D.
    *
    * @param random           the random generator to use.
    * @param numberOfVertices the supplier's size.
    * @return the random supplier.
    */
   public static Vertex3DSupplier nextVertex3DSupplier(Random random, int numberOfVertices)
   {
      List<Point3D> vertices = IntStream.range(0, numberOfVertices).mapToObj(i -> nextPoint3D(random)).collect(Collectors.toList());
      return Vertex3DSupplier.asVertex3DSupplier(vertices);
   }

   /**
    * Generates a random point that is constrained to lie inside a 2D triangle.
    *
    * @param random the random generator to use.
    * @param a      the first vertex of the bounding triangle. Not modified.
    * @param b      the second vertex of the bounding triangle. Not modified.
    * @param c      the third vertex of the bounding triangle. Not modified.
    * @return the random point 2D.
    */
   public static Point2D nextPoint2DInTriangle(Random random, Point2DReadOnly a, Point2DReadOnly b, Point2DReadOnly c)
   {
      return new Point2D(nextPoint3DInTriangle(random, new Point3D(a), new Point3D(b), new Point3D(c)));
   }

   /**
    * Generates a random point that is constrained to lie inside a 3D triangle.
    *
    * @param random the random generator to use.
    * @param a      the first vertex of the bounding triangle. Not modified.
    * @param b      the second vertex of the bounding triangle. Not modified.
    * @param c      the third vertex of the bounding triangle. Not modified.
    * @return the random point 3D.
    */
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

   /**
    * Generates a random point that is constrained to lie inside a 3D tetrahedron.
    *
    * @param random the random generator to use.
    * @param a      the first vertex of the bounding tetrahedron. Not modified.
    * @param b      the second vertex of the bounding tetrahedron. Not modified.
    * @param c      the third vertex of the bounding tetrahedron. Not modified.
    * @param d      the fourth vertex of the bounding tetrahedron. Not modified.
    * @return the random point 3D.
    */
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

   /**
    * Generates a random point 3D from random weighted average of the given points.
    * <p>
    * Note that this approach does not provide a uniform distribution across the 3D convex hull defined
    * by the given points. Instead, the resulting distribution is centered and concentrated about the
    * average of the points.
    * </p>
    *
    * @param random the random generator to use.
    * @param points the points to generate the random point from. Not modified.
    * @return the random point 3D.
    */
   public static Point3D nextWeightedAverage(Random random, Collection<? extends Point3DReadOnly> points)
   {
      return nextWeightedAverage(random, points.toArray(new Point3DReadOnly[points.size()]));
   }

   /**
    * Generates a random point 3D from random weighted average of the given points.
    * <p>
    * Note that this approach does not provide a uniform distribution across the 3D convex hull defined
    * by the given points. Instead, the resulting distribution is centered and concentrated about the
    * average of the points.
    * </p>
    *
    * @param random the random generator to use.
    * @param points the points to generate the random point from. Not modified.
    * @return the random point 3D.
    */
   public static Point3D nextWeightedAverage(Random random, Point3DReadOnly[] points)
   {
      double sum = 0.0;
      Point3D next = new Point3D();

      for (int j = 0; j < points.length; j++)
      {
         double weight = random.nextDouble();
         sum += weight;
         next.scaleAdd(weight, points[j], next);
      }

      next.scale(1.0 / sum);

      return next;
   }
}
